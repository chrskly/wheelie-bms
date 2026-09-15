/*
 * This file is part of the ev mustang bms project.
 *
 * Copyright (C) 2025 Christian Kelly <chrskly@chrskly.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*------------------------------------------------------------------------------

Snapshot handoff between the BMS worker task and the web server task, plus the
JSON serialiser the web server answers /api/status with.

Contains no Arduino, FreeRTOS or WiFi code on purpose: this is the part with
logic worth testing, and it builds and runs in the native test suite.

------------------------------------------------------------------------------*/

#include <atomic>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "webstatus.h"


/*==============================================================================
 * The seqlock
 *
 * `sequence` is even when the snapshot is stable and odd while it is being
 * written. A reader takes the sequence, copies, and takes it again; if either
 * read was odd or the two differ, the copy may be a mixture of two publishes
 * and is thrown away.
 *
 * The fences are what make this correct rather than merely usually correct:
 * without them the compiler is free to sink a store to `shared` past the
 * closing increment of `sequence`, and the ESP32-S3's two cores would then let
 * a reader observe a "stable" sequence over a half-written buffer.
 *============================================================================*/

static std::atomic<uint32_t> sequence(0);
static WebSnapshot shared;

/* A reader that loses this many races gives up. The writer publishes for a few
 * microseconds once every WEB_SNAPSHOT_INTERVAL_MS, so losing even twice in a
 * row means something is badly wrong rather than merely unlucky. */
#define WEBSTATUS_READ_RETRIES 8


void webstatus_publish(const WebSnapshot& snapshot) {
    sequence.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);

    shared = snapshot;
    shared.valid = true;

    std::atomic_thread_fence(std::memory_order_release);
    sequence.fetch_add(1, std::memory_order_relaxed);
}

bool webstatus_read(WebSnapshot& out) {
    /* Nothing has been published while the sequence is still 0. Derived from
     * the atomic rather than kept in a separate bool, which the writer would
     * have been storing to while the reader loaded it. */
    if ( sequence.load(std::memory_order_relaxed) == 0 ) {
        return false;
    }
    for ( int attempt = 0; attempt < WEBSTATUS_READ_RETRIES; attempt++ ) {
        const uint32_t before = sequence.load(std::memory_order_relaxed);
        if ( ( before & 1u ) != 0u ) {
            continue;  // a publish is in progress
        }
        std::atomic_thread_fence(std::memory_order_acquire);

        WebSnapshot candidate = shared;

        std::atomic_thread_fence(std::memory_order_acquire);
        const uint32_t after = sequence.load(std::memory_order_relaxed);
        if ( before == after ) {
            out = candidate;
            return out.valid;
        }
    }
    return false;
}


/*==============================================================================
 * JSON serialisation
 *
 * A tiny append-only writer that latches an overflow flag rather than
 * truncating. Every write goes through it, so a buffer that turns out to be too
 * small produces "no answer" instead of a JSON document that stops mid-array
 * and makes the browser's parser -- and whoever is debugging it -- guess.
 *============================================================================*/

namespace {

struct JsonWriter {
    char*  buf;
    size_t cap;      // total bytes available, including room for the NUL
    size_t len;
    bool   overflow;

    JsonWriter(char* _buf, size_t _cap) : buf(_buf), cap(_cap), len(0), overflow(false) {
        if ( cap > 0 ) {
            buf[0] = '\0';
        } else {
            overflow = true;
        }
    }

    void raw(const char* text) {
        if ( overflow ) {
            return;
        }
        const size_t textLen = strlen(text);
        if ( textLen + 1 > cap - len ) {
            overflow = true;
            return;
        }
        memcpy(buf + len, text, textLen + 1);
        len += textLen;
    }

    __attribute__((format(printf, 2, 3)))
    void fmt(const char* format, ...) {
        if ( overflow ) {
            return;
        }
        va_list args;
        va_start(args, format);
        const int written = vsnprintf(buf + len, cap - len, format, args);
        va_end(args);
        /* vsnprintf returns what it WOULD have written, so this catches
         * truncation as well as an outright encoding error. */
        if ( written < 0 || (size_t)written >= cap - len ) {
            overflow = true;
            buf[len] = '\0';
            return;
        }
        len += (size_t)written;
    }

    void key_u32(const char* key, uint32_t value)  { fmt("\"%s\":%u,", key, (unsigned int)value); }
    void key_i32(const char* key, int32_t value)   { fmt("\"%s\":%d,", key, (int)value); }
    void key_u64(const char* key, uint64_t value)  { fmt("\"%s\":%llu,", key, (unsigned long long)value); }
    void key_bool(const char* key, bool value)     { fmt("\"%s\":%s,", key, value ? "true" : "false"); }

    /* Only ever handed string literals and get_state_name()'s table entries, so
     * there is nothing to escape -- but check rather than assume, because a
     * stray quote would produce a document the browser silently drops. */
    void key_str(const char* key, const char* value) {
        if ( value == nullptr ) {
            fmt("\"%s\":null,", key);
            return;
        }
        for ( const char* c = value; *c != '\0'; c++ ) {
            if ( *c == '"' || *c == '\\' || (unsigned char)*c < 0x20 ) {
                fmt("\"%s\":null,", key);
                return;
            }
        }
        fmt("\"%s\":\"%s\",", key, value);
    }

    void open_object(const char* key) { fmt("\"%s\":{", key); }
    void open_array(const char* key)  { fmt("\"%s\":[", key); }

    /* Drops the separator the last member left behind, so no caller has to
     * know whether it is emitting the final element of anything. */
    void close(char bracket) {
        if ( overflow ) {
            return;
        }
        if ( len > 0 && buf[len - 1] == ',' ) {
            len--;
        }
        const char closing[2] = { bracket, '\0' };
        raw(closing);
        raw(",");
    }

    void close_object() { close('}'); }
    void close_array()  { close(']'); }
};

void render_module(JsonWriter& out, const WebModuleSnapshot& module) {
    out.raw("{");
    out.open_array("v");
    for ( int c = 0; c < CELLS_PER_MODULE; c++ ) {
        out.fmt("%u,", (unsigned int)module.cellVoltage[c]);
    }
    out.close_array();
    out.open_array("t");
    for ( int t = 0; t < TEMPS_PER_MODULE; t++ ) {
        out.fmt("%d,", (int)module.cellTemperature[t]);
    }
    out.close_array();
    out.key_u32("bal", module.balanceStatus);
    out.key_u32("err", module.errorStatus);
    out.key_bool("alive", module.alive);
    out.key_bool("pop", module.populated);
    out.close_object();
}

void render_pack(JsonWriter& out, const WebPackSnapshot& pack) {
    out.raw("{");
    out.key_u32("voltage", pack.voltage);
    out.key_u32("cellMin", pack.lowestCellVoltage);
    out.key_u32("cellMax", pack.highestCellVoltage);
    out.key_u32("cellDelta", pack.cellDelta);
    out.key_i32("tempMin", pack.lowestTemperature);
    out.key_i32("tempMax", pack.highestTemperature);
    out.key_bool("alive", pack.alive);
    out.key_bool("inhibited", pack.contactorsInhibited);
    out.key_bool("welded", pack.contactorsWelded);
    out.key_bool("balancing", pack.balancing);
    out.key_bool("deadCell", pack.hasDeadCell);
    out.key_u32("balanceTarget", pack.balanceTargetMv);
    out.key_u32("canTx", pack.canTxErrors);
    out.key_u32("canRx", pack.canRxErrors);
    out.open_array("modules");
    for ( int m = 0; m < MODULES_PER_PACK; m++ ) {
        render_module(out, pack.modules[m]);
    }
    out.close_array();
    out.close_object();
}

}  // namespace


size_t webstatus_render_json(const WebSnapshot& snapshot, char* buf, size_t bufLen) {
    if ( buf == nullptr ) {
        return 0;
    }
    /* A zero-length buffer is handled by JsonWriter's own cap check rather than
     * a second guard here, so there is one place that decides what "no room"
     * means. The overflow path below is careful not to write the terminator
     * when there is no byte to write it into. */

    JsonWriter out(buf, bufLen);
    out.raw("{");

    out.key_u64("uptime", snapshot.uptimeMs);
    out.key_str("state", snapshot.stateName);
    out.key_u64("timeInState", snapshot.timeInStateMs);
    out.key_u32("soc", snapshot.soc);
    out.key_u32("voltage", snapshot.voltage);
    out.key_u32("cellMin", snapshot.lowestCellVoltage);
    out.key_u32("cellMax", snapshot.highestCellVoltage);
    out.key_u32("cellDelta", snapshot.cellDelta);
    out.key_i32("tempMin", snapshot.lowestTemperature);
    out.key_i32("tempMax", snapshot.highestTemperature);
    out.key_bool("batteryAlive", snapshot.batteryAlive);
    out.key_u32("activePacks", snapshot.activePacks);
    out.key_u32("maxChargeCurrent", snapshot.maxChargeCurrent);
    out.key_u32("maxDischargeCurrent", snapshot.maxDischargeCurrent);
    out.key_bool("heater", snapshot.heaterOn);
    out.key_bool("ignition", snapshot.ignitionOn);
    out.key_bool("chargeEnable", snapshot.chargeEnabled);

    out.open_object("drive");
    out.key_bool("inhibited", snapshot.driveInhibited);
    out.key_u32("reasons", snapshot.driveInhibitReasons);
    out.close_object();

    out.open_object("charge");
    out.key_bool("inhibited", snapshot.chargeInhibited);
    out.key_u32("reasons", snapshot.chargeInhibitReasons);
    out.close_object();

    out.open_object("flags");
    out.key_bool("tooHot", snapshot.tooHot);
    out.key_bool("tooCold", snapshot.tooColdToCharge);
    out.key_bool("emptyCell", snapshot.hasEmptyCell);
    out.key_bool("fullCell", snapshot.hasFullCell);
    out.key_bool("deadCell", snapshot.hasDeadCell);
    out.key_bool("imbalanced", snapshot.packsImbalanced);
    out.close_object();

    out.open_object("fault");
    out.key_u32("internal", snapshot.internalErrorFlags);
    out.key_u32("errorByte", snapshot.errorByte);
    out.key_u32("statusByte", snapshot.statusByte);
    out.key_u32("weldingByte", snapshot.weldingByte);
    out.key_bool("illegalTransition", snapshot.illegalStateTransition);
    out.key_u32("invalidEvents", snapshot.invalidEventCount);
    out.key_bool("watchdogReboot", snapshot.watchdogReboot);
    out.close_object();

    out.open_object("shunt");
    out.key_bool("alive", snapshot.shuntAlive);
    out.key_i32("amps", snapshot.shuntAmps);
    out.key_i32("v1", snapshot.shuntVoltage1);
    out.key_i32("v2", snapshot.shuntVoltage2);
    out.key_i32("v3", snapshot.shuntVoltage3);
    out.key_i32("temp", snapshot.shuntTemperature);
    out.key_i32("watts", snapshot.shuntWatts);
    out.key_i32("as", snapshot.shuntAmpSeconds);
    out.key_i32("wh", snapshot.shuntWattHours);
    out.close_object();

    out.open_object("can");
    out.key_u32("tx", snapshot.mainCanTxErrors);
    out.key_u32("rx", snapshot.mainCanRxErrors);
    out.close_object();

    /* Thresholds travel with the data so the page colours cells against the
     * limits this firmware was actually built with, rather than a second copy
     * of the numbers that has to be kept in step by hand. */
    out.open_object("cfg");
    out.key_u32("packs", NUM_PACKS);
    out.key_u32("modules", MODULES_PER_PACK);
    out.key_u32("cells", CELLS_PER_MODULE);
    out.key_u32("temps", TEMPS_PER_MODULE);
    out.key_u32("cellEmpty", CELL_EMPTY_VOLTAGE);
    out.key_u32("cellFull", CELL_FULL_VOLTAGE);
    out.key_u32("cellDead", DEAD_CELL_VOLTAGE);
    out.key_u32("deltaWarn", CELL_DELTA_WARN_THRESHOLD);
    out.key_u32("deltaAlarm", CELL_DELTA_ALARM_THRESHOLD);
    out.key_i32("tempWarn", WARNING_TEMPERATURE);
    out.key_i32("tempMaxLimit", MAXIMUM_TEMPERATURE);
    out.key_i32("tempChargeMin", CHARGE_TEMPERATURE_MINIMUM);
    out.key_i32("noTemp", NO_TEMPERATURE_READING);
    out.key_u32("capacityWh", BATTERY_CAPACITY_WH);
    out.key_str("version", VERSION_STRING);
    out.close_object();

    out.open_array("packs");
    for ( int p = 0; p < NUM_PACKS; p++ ) {
        render_pack(out, snapshot.packs[p]);
    }
    out.close_array();

    out.close_object();

    if ( out.overflow ) {
        if ( bufLen > 0 ) {
            buf[0] = '\0';
        }
        return 0;
    }
    /* close_object() leaves the separator it would need if it were nested. */
    if ( out.len > 0 && buf[out.len - 1] == ',' ) {
        out.len--;
        buf[out.len] = '\0';
    }
    return out.len;
}

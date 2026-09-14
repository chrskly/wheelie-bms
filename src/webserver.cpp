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

WEB SERVER

A deliberately small, read-only HTTP/1.1 server. It serves the phone UI
(index.html, style.css, app.js) out of LittleFS and answers /api/status with a
JSON rendering of the snapshot the BMS worker publishes.

There are no endpoints that change anything. Nothing here can close a
contactor, clear a fault or edit a threshold.

This file is the transport glue and is ESP32-only -- the native test suite has
no WiFi stack. Everything with logic worth testing lives in webstatus.cpp,
which does build natively. The stubs at the bottom keep main.cpp linking there.

------------------------------------------------------------------------------*/

#include "settings.h"
#include "webserver.h"

#if defined(ESP32) && WEB_INTERFACE_ENABLED

#include <Arduino.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <stdio.h>
#include <string.h>

#include "util.h"
#include "webstatus.h"

static WiFiServer server(WEB_SERVER_PORT);

/* Static, not stack: the web task's stack has to hold the lwIP send path as
 * well, and these three together are the best part of 8 KB. Only ever touched
 * by the web task, which handles one client at a time. */
static char       jsonBuffer[WEB_JSON_BUFFER_BYTES];
static WebSnapshot snapshotCopy;
static uint8_t    fileBuffer[512];

static bool filesystemMounted = false;


/*==============================================================================
 * Responses
 *============================================================================*/

static void send_headers(WiFiClient& client, const char* status, const char* contentType,
                         size_t contentLength, const char* cacheControl) {
    char header[224];
    const int len = snprintf(header, sizeof(header),
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %u\r\n"
        "Cache-Control: %s\r\n"
        "Connection: close\r\n"
        "\r\n",
        status, contentType, (unsigned int)contentLength, cacheControl);
    if ( len > 0 && (size_t)len < sizeof(header) ) {
        client.write((const uint8_t*)header, (size_t)len);
    }
}

static void send_text(WiFiClient& client, const char* status, const char* contentType,
                      const char* body, const char* cacheControl) {
    const size_t bodyLength = strlen(body);
    send_headers(client, status, contentType, bodyLength, cacheControl);
    client.write((const uint8_t*)body, bodyLength);
}

/* Shown when LittleFS has no index.html, which in practice means the firmware
 * was flashed but `pio run -t uploadfs` was not run. Without this the browser
 * gets a bare 404 and the cause is not obvious. */
static void send_missing_filesystem_page(WiFiClient& client) {
    send_text(client, "200 OK", "text/html; charset=utf-8",
        "<!doctype html><meta name=viewport content=\"width=device-width,initial-scale=1\">"
        "<title>wheelie-bms</title>"
        "<body style=\"font:16px/1.5 system-ui,sans-serif;background:#512B81;color:#fff;margin:0;padding:24px\">"
        "<h1 style=\"font-size:22px\">Web UI files are missing</h1>"
        "<p>The firmware is running and <a style=\"color:#7ecdf5\" href=\"/api/status\">/api/status</a> works, "
        "but no <code>index.html</code> was found on the flash filesystem.</p>"
        "<p>Upload the contents of <code>data/</code> with:</p>"
        "<pre style=\"background:#232D3F;padding:12px;border-radius:8px;overflow-x:auto\">pio run -t uploadfs</pre>"
        "</body>", "no-store");
}

static const char* content_type_for(const char* path) {
    const char* dot = strrchr(path, '.');
    if ( dot == nullptr ) {
        return "application/octet-stream";
    }
    if ( strcmp(dot, ".html") == 0 ) { return "text/html; charset=utf-8"; }
    if ( strcmp(dot, ".css")  == 0 ) { return "text/css; charset=utf-8"; }
    if ( strcmp(dot, ".js")   == 0 ) { return "text/javascript; charset=utf-8"; }
    if ( strcmp(dot, ".json") == 0 ) { return "application/json; charset=utf-8"; }
    if ( strcmp(dot, ".svg")  == 0 ) { return "image/svg+xml"; }
    if ( strcmp(dot, ".png")  == 0 ) { return "image/png"; }
    if ( strcmp(dot, ".ico")  == 0 ) { return "image/x-icon"; }
    return "application/octet-stream";
}

static void send_file(WiFiClient& client, const char* path) {
    if ( !filesystemMounted ) {
        send_missing_filesystem_page(client);
        return;
    }
    File file = LittleFS.open(path, "r");
    if ( !file || file.isDirectory() ) {
        if ( file ) {
            file.close();
        }
        if ( strcmp(path, "/index.html") == 0 ) {
            send_missing_filesystem_page(client);
        } else {
            send_text(client, "404 Not Found", "text/plain; charset=utf-8", "Not found\n", "no-store");
        }
        return;
    }
    /* A short max-age rather than none: the page and its stylesheet are
     * re-requested every time the phone's screen wakes, and re-sending 20 KB
     * over a link the BMS is also using is pointless. Short enough that
     * re-uploading the filesystem shows up without clearing the cache. */
    send_headers(client, "200 OK", content_type_for(path), (size_t)file.size(), "max-age=60");
    while ( file.available() ) {
        const size_t chunk = file.read(fileBuffer, sizeof(fileBuffer));
        if ( chunk == 0 ) {
            break;
        }
        if ( client.write(fileBuffer, chunk) != chunk ) {
            break;  // client went away mid-transfer
        }
    }
    file.close();
}

static void send_status_json(WiFiClient& client) {
    if ( !webstatus_read(snapshotCopy) ) {
        /* Nothing published yet -- the BMS is still in its first half second.
         * 503 rather than an empty object, so the page can say "waiting" rather
         * than draw a battery with every reading at zero. */
        send_text(client, "503 Service Unavailable", "application/json; charset=utf-8",
                  "{\"error\":\"no snapshot yet\"}", "no-store");
        return;
    }
    const size_t length = webstatus_render_json(snapshotCopy, jsonBuffer, sizeof(jsonBuffer));
    if ( length == 0 ) {
        printf("[web] ERROR snapshot did not fit in %u bytes; raise WEB_JSON_BUFFER_BYTES\n",
               (unsigned int)sizeof(jsonBuffer));
        send_text(client, "500 Internal Server Error", "application/json; charset=utf-8",
                  "{\"error\":\"buffer too small\"}", "no-store");
        return;
    }
    send_headers(client, "200 OK", "application/json; charset=utf-8", length, "no-store");
    client.write((const uint8_t*)jsonBuffer, length);
}


/*==============================================================================
 * Request parsing
 *============================================================================*/

enum ReadLineResult {
    LINE_OK,
    LINE_TIMEOUT,      // client stopped sending, or went away
    LINE_TOO_LONG,     // more than we are willing to buffer
};

static ReadLineResult read_line(WiFiClient& client, char* out, size_t outLen, uint64_t deadline) {
    size_t length = 0;
    for ( ;; ) {
        if ( get_clock_ms() > deadline ) {
            return LINE_TIMEOUT;
        }
        if ( client.available() == 0 ) {
            if ( !client.connected() ) {
                return LINE_TIMEOUT;
            }
            /* Yield rather than spin. The web task sits one priority above the
             * idle task, so a busy-wait here starves the idle task on this core
             * and the task watchdog reboots the board. */
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        const int c = client.read();
        if ( c < 0 ) {
            continue;
        }
        if ( c == '\n' ) {
            if ( length > 0 && out[length - 1] == '\r' ) {
                length--;
            }
            out[length] = '\0';
            return LINE_OK;
        }
        if ( length + 1 >= outLen ) {
            return LINE_TOO_LONG;
        }
        out[length++] = (char)c;
    }
}

/*
 * Pull the path out of a request line, rejecting anything we are not prepared
 * to serve.
 *
 * Path rules, deliberately strict: a single leading '/', then only letters,
 * digits, '.', '_' and '-'. No directory separators and therefore no way to
 * express "..", so a request cannot walk out of the filesystem root. Everything
 * this UI needs sits in one flat directory.
 */
static bool parse_request(const char* line, char* pathOut, size_t pathLen) {
    if ( strncmp(line, "GET ", 4) != 0 ) {
        return false;
    }
    const char* start = line + 4;
    const char* end = strchr(start, ' ');
    if ( end == nullptr ) {
        end = start + strlen(start);
    }
    // Drop any query string; nothing here takes parameters.
    const char* query = (const char*)memchr(start, '?', (size_t)(end - start));
    if ( query != nullptr ) {
        end = query;
    }
    const size_t length = (size_t)(end - start);
    if ( length == 0 || length + 1 > pathLen || start[0] != '/' ) {
        return false;
    }
    for ( size_t i = 1; i < length; i++ ) {
        const char c = start[i];
        const bool allowed = ( c >= 'a' && c <= 'z' ) || ( c >= 'A' && c <= 'Z' )
                          || ( c >= '0' && c <= '9' )
                          || c == '.' || c == '_' || c == '-' || c == '/';
        if ( !allowed ) {
            return false;
        }
    }
    memcpy(pathOut, start, length);
    pathOut[length] = '\0';
    /* '/' is allowed by the character test above only so that /api/status
     * parses. Checked on the copy, not on `start`, which still runs on to the
     * HTTP version and everything after it. */
    if ( strstr(pathOut, "..") != nullptr ) {
        return false;
    }
    return true;
}

static void handle_client(WiFiClient& client) {
    const uint64_t deadline = get_clock_ms() + WEB_CLIENT_TIMEOUT_MS;
    char line[192];

    const ReadLineResult result = read_line(client, line, sizeof(line), deadline);
    if ( result == LINE_TIMEOUT ) {
        return;  // nothing useful arrived; just close
    }

    char path[128];
    const bool understood = ( result == LINE_OK ) && parse_request(line, path, sizeof(path));

    /* Drain the rest of the request before answering. Closing a socket with
     * unread data still queued makes some TCP stacks send a reset, and the
     * browser then discards the response we just wrote. Bounded so a client
     * that sends headers forever cannot hold the task. */
    for ( int header = 0; header < 40; header++ ) {
        char discard[192];
        const ReadLineResult headerResult = read_line(client, discard, sizeof(discard), deadline);
        if ( headerResult == LINE_TIMEOUT || ( headerResult == LINE_OK && discard[0] == '\0' ) ) {
            break;
        }
    }

    if ( result == LINE_TOO_LONG ) {
        send_text(client, "414 URI Too Long", "text/plain; charset=utf-8",
                  "URI too long\n", "no-store");
        return;
    }
    if ( !understood ) {
        send_text(client, "400 Bad Request", "text/plain; charset=utf-8",
                  "This server answers GET only\n", "no-store");
        return;
    }

    if ( strcmp(path, "/api/status") == 0 ) {
        send_status_json(client);
        return;
    }
    /* Any remaining path with a '/' beyond the leading one is not a file we
     * serve -- the UI is one flat directory. */
    if ( strchr(path + 1, '/') != nullptr ) {
        send_text(client, "404 Not Found", "text/plain; charset=utf-8", "Not found\n", "no-store");
        return;
    }
    if ( strcmp(path, "/") == 0 ) {
        send_file(client, "/index.html");
        return;
    }
    send_file(client, path);
}


/*==============================================================================
 * Task and startup
 *============================================================================*/

static void web_task(void* /*pvParameters*/) {
    for ( ;; ) {
        WiFiClient client = server.available();
        if ( client ) {
            handle_client(client);
            /* flush() before stop() so the response is actually on the wire.
             * stop() discards whatever is still buffered. */
            client.flush();
            client.stop();
        }
        /* Always yield, client or not. server.available() does not block, so
         * without this the task spins at priority 1, starves the idle task on
         * this core and the watchdog resets the board. */
        vTaskDelay(pdMS_TO_TICKS(WEB_ACCEPT_POLL_MS));
    }
}

static bool start_wifi() {
#if WEB_WIFI_STATION_MODE
    if ( strlen(WEB_WIFI_STA_SSID) == 0 ) {
        printf("[web] ERROR WEB_WIFI_STATION_MODE is set but WEB_WIFI_STA_SSID is empty\n");
        return false;
    }
    printf("[web] joining WiFi network '%s' ...\n", WEB_WIFI_STA_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WEB_WIFI_STA_SSID, WEB_WIFI_STA_PASSWORD);
    const uint64_t deadline = get_clock_ms() + WEB_WIFI_STA_TIMEOUT_MS;
    while ( WiFi.status() != WL_CONNECTED && get_clock_ms() < deadline ) {
        delay(250);
    }
    if ( WiFi.status() != WL_CONNECTED ) {
        printf("[web] ERROR could not join '%s'; carrying on without a web interface\n", WEB_WIFI_STA_SSID);
        return false;
    }
    printf("[web] joined '%s', address http://%s/\n", WEB_WIFI_STA_SSID, WiFi.localIP().toString().c_str());
    return true;
#else
    /* softAP() rejects a password shorter than 8 characters and returns false,
     * which used to leave no AP and no explanation. Say which it is. */
    const bool open = ( strlen(WEB_WIFI_AP_PASSWORD) == 0 );
    if ( !open && strlen(WEB_WIFI_AP_PASSWORD) < 8 ) {
        printf("[web] ERROR WEB_WIFI_AP_PASSWORD must be empty or at least 8 characters\n");
        return false;
    }
    WiFi.mode(WIFI_AP);
    if ( !WiFi.softAP(WEB_WIFI_AP_SSID, open ? nullptr : WEB_WIFI_AP_PASSWORD) ) {
        printf("[web] ERROR could not start the access point\n");
        return false;
    }
    printf("[web] access point '%s' up (%s), address http://%s/\n",
           WEB_WIFI_AP_SSID, open ? "open" : "WPA2",
           WiFi.softAPIP().toString().c_str());
    return true;
#endif
}

void webserver_start() {
    printf("[web] starting web interface\n");

    filesystemMounted = LittleFS.begin();
    if ( !filesystemMounted ) {
        /* Format on a first boot with a blank partition, then try once more.
         * A failure here is not fatal: /api/status still works, and the
         * built-in page explains how to upload the UI. */
        printf("[web] LittleFS not mounted, formatting ...\n");
        filesystemMounted = LittleFS.begin(true);
    }
    if ( !filesystemMounted ) {
        printf("[web] WARNING no filesystem; only /api/status will be served\n");
    }

    if ( !start_wifi() ) {
        return;
    }

    server.begin();
    /* Nagle would sit on the last, short packet of every response waiting for
     * more; there is never any more, so each poll paid an extra round trip. */
    server.setNoDelay(true);

    if ( strlen(WEB_MDNS_HOSTNAME) > 0 ) {
        if ( MDNS.begin(WEB_MDNS_HOSTNAME) ) {
            MDNS.addService("http", "tcp", WEB_SERVER_PORT);
            printf("[web] also reachable at http://%s.local/\n", WEB_MDNS_HOSTNAME);
        } else {
            printf("[web] WARNING mDNS did not start; use the IP address\n");
        }
    }

    const BaseType_t created = xTaskCreate(
        web_task,
        "webServer",
        WEB_TASK_STACK_BYTES,
        NULL,
        WEB_TASK_PRIORITY,
        NULL);
    if ( created != pdPASS ) {
        printf("[web] ERROR could not create the web server task\n");
    }
}

#else  // !ESP32 || !WEB_INTERFACE_ENABLED

/* The native test build has no WiFi stack and no filesystem, and the sim's
 * xTaskCreate() only remembers one task. main.cpp calls this unconditionally,
 * so it has to link -- the logic worth testing is in webstatus.cpp. */
void webserver_start() {}

#endif

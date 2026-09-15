/*
 * wheelie-bms web interface.
 *
 * Polls /api/status and paints it. There is nothing else to it: the server is
 * read only, so this file never sends anything the BMS acts on.
 *
 * No framework and no build step. The whole page is three files served off the
 * ESP32's own flash over its own access point, and it has to work on a phone
 * with no internet connection standing next to the car.
 */
'use strict';

var POLL_MS = 1000;
var FETCH_TIMEOUT_MS = 2500;

/* --------------------------------------------------------------------------
 * Decoding tables.
 *
 * These mirror enums in the firmware. The numeric limits do NOT live here --
 * they arrive in the snapshot's "cfg" object so the page always colours
 * against the thresholds the running firmware was built with.
 * ------------------------------------------------------------------------ */

// InhibitReason, from include/bms.h. Index is the enum value; the bit in the
// mask is 1 << value. R_NONE (0) never appears in a mask.
var INHIBIT_REASONS = [
    null,
    'Battery too hot',
    'Battery too cold',
    'Battery full',
    'Battery empty',
    'Charging',
    'Illegal state transition',
    'Module unresponsive',
    'Shunt unresponsive',
    'Critical fault',
    'Dead cell',
    'Starting up'
];

// InternalErrorSource, from include/bms.h
var INTERNAL_ERRORS = [
    'Lowest cell voltage out of range',
    'Highest cell voltage out of range',
    'Lowest temperature out of range',
    'Highest temperature out of range',
    'Heater ineffective',
    'Temperature data stale',
    'Shunt readings implausible'           // IE_SHUNT_IMPLAUSIBLE, bit 6
];

// Bit layout of Bms::get_error_byte() / get_status_byte() / get_welding_byte()
var ERROR_BITS   = ['Internal error', 'Packs imbalanced', 'Shunt dead',
                    'Illegal transition', 'Battery not alive', 'Dead cell'];
var STATUS_BITS  = ['Charge inhibited', 'Drive inhibited', 'Heater on',
                    'Ignition on', 'Charge enabled', 'Regen not allowed'];
var WELDING_BITS = ['Positive contactor', 'Negative contactor',
                    'Pack 1 contactor', 'Pack 2 contactor'];

// State name from get_state_name() -> how the banner reads
var STATES = {
    standby:                     { text: 'STANDBY',           css: 'grey'  },
    drive:                       { text: 'DRIVE',             css: 'green' },
    batteryHeating:              { text: 'HEATING',           css: 'amber' },
    charging:                    { text: 'CHARGING',          css: 'blue'  },
    batteryEmpty:                { text: 'BATTERY EMPTY',     css: 'red'   },
    overTempFault:               { text: 'OVER TEMPERATURE',  css: 'red'   },
    illegalStateTransitionFault: { text: 'STATE FAULT',       css: 'red'   },
    criticalFault:               { text: 'CRITICAL FAULT',    css: 'red'   }
};

/* -------------------------------- helpers ------------------------------- */

function $(id) { return document.getElementById(id); }

function text(id, value) {
    var el = $(id);
    if (el && el.textContent !== value) { el.textContent = value; }
}

function classed(el, name, on) {
    if (el) { el.classList.toggle(name, !!on); }
}

/*
 * NO_CELL_VOLTAGE_READING is 10000 and the "highest cell" search starts at 0,
 * so a pack that has not reported yet reads back as min 10000 / max 0. Showing
 * those as though they were millivolts is worse than showing nothing.
 */
function hasReading(mV) { return mV > 0 && mV < 10000; }
function mv(value) { return hasReading(value) ? value + ' mV' : '\u2013'; }
function mvPair(lo, hi) {
    return (hasReading(lo) && hasReading(hi)) ? lo + ' / ' + hi : '\u2013';
}

/*
 * Temperature sentinels, all of which would otherwise print as a plausible-
 * looking reading: -127 is an unfitted sensor slot (NO_TEMPERATURE_READING),
 * and the pack min/max searches start at 126 and -126 and return those
 * untouched when no module has reported.
 */
function hasTemp(degC) { return degC > -126 && degC < 126; }
function tempPair(lo, hi) {
    return (hasTemp(lo) && hasTemp(hi)) ? lo + ' / ' + hi + '\u00b0' : '\u2013';
}

function volts(mV, digits) { return (mV / 1000).toFixed(digits === undefined ? 1 : digits) + ' V'; }
function amps(mA)          { return (mA / 1000).toFixed(1) + ' A'; }
function kilowatts(w)      { return (w / 1000).toFixed(2) + ' kW'; }

function duration(ms) {
    var s = Math.floor(ms / 1000);
    var h = Math.floor(s / 3600);
    var m = Math.floor((s % 3600) / 60);
    s = s % 60;
    function pad(n) { return (n < 10 ? '0' : '') + n; }
    return h > 0 ? h + ':' + pad(m) + ':' + pad(s) : m + ':' + pad(s);
}

function hex(value, width) {
    var s = value.toString(16).toUpperCase();
    while (s.length < width) { s = '0' + s; }
    return '0x' + s;
}

function yesNo(on, goodWhenTrue) {
    return { label: on ? 'Yes' : 'No', css: (on === !!goodWhenTrue) ? 'ok-text' : 'bad-text' };
}

// Turn a bitmask into the list of names whose bit is set.
function decodeBits(mask, names, shift) {
    var out = [];
    for (var i = 0; i < names.length; i++) {
        if (names[i] && (mask & (1 << (i + (shift || 0))))) { out.push(names[i]); }
    }
    return out;
}

function fillList(el, items, emptyText) {
    if (!el) { return; }
    var wanted = items.length ? items : (emptyText ? [emptyText] : []);
    // Rebuild only when the content actually changed, so the list is not
    // thrown away and recreated every single poll.
    if (el.dataset.key === wanted.join('|')) { return; }
    el.dataset.key = wanted.join('|');
    el.textContent = '';
    for (var i = 0; i < wanted.length; i++) {
        var li = document.createElement('li');
        li.textContent = wanted[i];
        el.appendChild(li);
    }
}

function fillTable(el, rows) {
    if (!el) { return; }
    var key = rows.map(function (r) { return r[0] + '=' + r[1]; }).join('|');
    if (el.dataset.key === key) { return; }
    el.dataset.key = key;
    el.textContent = '';
    rows.forEach(function (row) {
        var tr = document.createElement('tr');
        var td1 = document.createElement('td');
        td1.textContent = row[0];
        var td2 = document.createElement('td');
        td2.textContent = row[1];
        if (row[2]) { td2.className = row[2]; }
        tr.appendChild(td1);
        tr.appendChild(td2);
        el.appendChild(tr);
    });
}

/* ------------------------------- view state ----------------------------- */

var view = {
    tab: 'status',
    pack: 0,
    mode: 'volts',        // 'volts' or 'temps'
    numbers: false,
    layoutKey: '',        // what the modules pane is currently built for
    failures: 0,
    cfg: null
};

/* ---------------------------- tabs and controls ------------------------- */

var TABS = ['status', 'cells', 'packs', 'system'];

function showTab(name) {
    view.tab = name;
    /* Kept in the URL so reloading the page -- which a phone does every time it
     * decides to drop the WiFi and come back -- returns to the tab you were on
     * rather than to Status. replaceState, not pushState: the back button
     * should leave the page, not walk back through tabs. */
    if (window.history && window.history.replaceState && location.hash !== '#' + name) {
        window.history.replaceState(null, '', '#' + name);
    }
    TABS.forEach(function (t) {
        classed($('tab-' + t), 'hidden', t !== name);
    });
    document.querySelectorAll('#nav .nav-button').forEach(function (b) {
        classed(b, 'active', b.dataset.tab === name);
    });
    window.scrollTo(0, 0);
}

document.querySelectorAll('#nav .nav-button').forEach(function (b) {
    b.addEventListener('click', function () { showTab(b.dataset.tab); });
});

document.querySelectorAll('.seg[data-view]').forEach(function (b) {
    b.addEventListener('click', function () {
        view.mode = b.dataset.view;
        document.querySelectorAll('.seg[data-view]').forEach(function (o) {
            classed(o, 'active', o === b);
        });
    });
});

$('show-numbers').addEventListener('change', function (e) {
    view.numbers = e.target.checked;
});

function buildPackPicker(packCount) {
    var picker = $('pack-picker');
    if (picker.childElementCount === packCount) { return; }
    picker.textContent = '';
    // One pack needs no picker at all.
    if (packCount < 2) { return; }
    for (var p = 0; p < packCount; p++) {
        var b = document.createElement('button');
        b.className = 'seg' + (p === view.pack ? ' active' : '');
        b.textContent = 'Pack ' + (p + 1);
        b.dataset.pack = String(p);
        b.addEventListener('click', function (e) {
            view.pack = Number(e.target.dataset.pack);
            picker.querySelectorAll('.seg').forEach(function (o) {
                classed(o, 'active', o === e.target);
            });
        });
        picker.appendChild(b);
    }
}

/* ------------------------------ cell colours ---------------------------- */

/*
 * Which colour class a cell voltage earns.
 *
 * `extreme` marks the pack's own lowest/highest cell, and is only passed as
 * true once the pack delta is past the warning threshold -- so in a healthy
 * pack nothing is highlighted, and in a drifting one the offenders are.
 */
function voltageClass(mV, cfg, extreme) {
    if (!mV || mV >= 10000) { return 'none'; }        // NO_CELL_VOLTAGE_READING
    if (mV <= cfg.cellDead) { return 'dead'; }
    if (mV <= cfg.cellEmpty || mV >= cfg.cellFull) { return 'bad'; }
    return extreme ? 'warn' : '';
}

function temperatureClass(degC, cfg) {
    if (!hasTemp(degC)) { return 'none'; }
    if (degC >= cfg.tempMaxLimit) { return 'bad'; }
    if (degC >= cfg.tempWarn) { return 'warn'; }
    return '';
}

/* ---------------------------- the modules pane -------------------------- */

/*
 * Rebuilt only when the shape changes (pack, voltages/temperatures, numbers on
 * or off). Values are written into the existing nodes on every poll, so a 1 Hz
 * refresh does not churn 200 elements a second on a phone.
 */
function buildModules(cfg) {
    var host = $('modules');
    host.textContent = '';
    var count = view.mode === 'volts' ? cfg.cells : cfg.temps;

    for (var m = 0; m < cfg.modules; m++) {
        var wrap = document.createElement('div');
        wrap.className = 'module';

        var head = document.createElement('div');
        head.className = 'module-head';
        var name = document.createElement('b');
        name.textContent = 'Module ' + (m + 1);
        var range = document.createElement('span');
        head.appendChild(name);
        head.appendChild(range);
        wrap.appendChild(head);

        var body = document.createElement('div');
        if (view.numbers || view.mode === 'temps') {
            body.className = 'grid' + (view.mode === 'temps' ? ' temps' : '');
            for (var c = 0; c < count; c++) {
                var chip = document.createElement('div');
                chip.className = 'chip';
                var value = document.createElement('span');
                var label = document.createElement('i');
                label.textContent = (view.mode === 'temps' ? 'T' : '') + (c + 1);
                chip.appendChild(value);
                chip.appendChild(label);
                body.appendChild(chip);
            }
        } else {
            body.className = 'bars';
            for (var b = 0; b < count; b++) {
                var bar = document.createElement('div');
                bar.className = 'cell';
                body.appendChild(bar);
            }
        }
        wrap.appendChild(body);
        host.appendChild(wrap);
    }
}

function paintModules(pack, cfg) {
    var host = $('modules');
    var wraps = host.children;
    var showNumbers = view.numbers || view.mode === 'temps';

    for (var m = 0; m < cfg.modules && m < wraps.length; m++) {
        var module = pack.modules[m];
        var wrap = wraps[m];
        var range = wrap.firstChild.lastChild;
        var body = wrap.lastChild;

        // A module we have not heard from, or one that has not sent every
        // voltage group yet, is faded rather than hidden: the numbers are real,
        // they are just not current.
        classed(wrap, 'stale', !module.alive || !module.pop);

        if (view.mode === 'volts') {
            /* Bars are scaled across the whole pack, not each module's own
             * range, so a module sitting 30 mV below the others looks 30 mV
             * below the others instead of being renormalised back to healthy.
             * A pack that has not reported has min 10000 / max 0, which would
             * otherwise scale every bar to nonsense. */
            var lo = pack.cellMin;
            var hi = pack.cellMax;
            var scaled = hasReading(lo) && hasReading(hi);
            var span = Math.max(1, hi - lo);
            var flagExtremes = (pack.cellDelta > cfg.deltaWarn);

            // The label, though, is this module's own spread.
            var mLo = 10000;
            var mHi = 0;
            for (var i = 0; i < cfg.cells; i++) {
                if (!hasReading(module.v[i])) { continue; }
                if (module.v[i] < mLo) { mLo = module.v[i]; }
                if (module.v[i] > mHi) { mHi = module.v[i]; }
            }
            range.textContent = hasReading(mLo)
                ? mLo + '\u2013' + mHi + ' mV  \u0394' + (mHi - mLo)
                : 'no data';

            for (var c = 0; c < cfg.cells; c++) {
                var mV = module.v[c];
                var extreme = flagExtremes && (mV === lo || mV === hi);
                var cls = voltageClass(mV, cfg, extreme);
                var node = body.children[c];
                if (!node) { continue; }
                if (showNumbers) {
                    node.className = 'chip' + (cls ? ' ' + cls : '');
                    node.firstChild.textContent = hasReading(mV) ? String(mV) : '\u2013';
                } else {
                    node.className = 'cell' + (cls ? ' ' + cls : '');
                    /* 15% floor so a cell sitting at the pack minimum is still a
                     * visible bar rather than a blank column. */
                    var height = (cls === 'none' || !scaled) ? 0 : 15 + 85 * ((mV - lo) / span);
                    node.style.height = Math.max(0, Math.min(100, height)) + '%';
                }
            }
        } else {
            range.textContent = module.alive
                ? module.t.filter(hasTemp).length + ' sensors fitted'
                : 'no data';
            for (var t = 0; t < cfg.temps; t++) {
                var degC = module.t[t];
                var chip = body.children[t];
                if (!chip) { continue; }
                var tcls = temperatureClass(degC, cfg);
                chip.className = 'chip' + (tcls ? ' ' + tcls : '');
                chip.firstChild.textContent = hasTemp(degC) ? degC + '\u00b0' : '\u2013';
            }
        }
    }
}

function renderCells(data) {
    var cfg = data.cfg;
    buildPackPicker(cfg.packs);
    if (view.pack >= cfg.packs) { view.pack = 0; }
    var pack = data.packs[view.pack];

    // The summary row follows whichever view is showing, so it never reads out
    // millivolts above a screenful of temperatures.
    if (view.mode === 'temps') {
        var known = hasTemp(pack.tempMin) && hasTemp(pack.tempMax);
        text('c-min', known ? pack.tempMin + '\u00b0C' : '\u2013');
        text('c-delta', known ? (pack.tempMax - pack.tempMin) + '\u00b0C' : '\u2013');
        text('c-max', known ? pack.tempMax + '\u00b0C' : '\u2013');
        classed($('c-max').parentNode, 'warn', known && pack.tempMax >= cfg.tempWarn);
        classed($('c-max').parentNode, 'bad', known && pack.tempMax >= cfg.tempMaxLimit);
        classed($('c-delta').parentNode, 'warn', false);
        classed($('c-delta').parentNode, 'bad', false);
    } else {
        text('c-min', mv(pack.cellMin));
        text('c-delta', hasReading(pack.cellMin) ? pack.cellDelta + ' mV' : '\u2013');
        text('c-max', mv(pack.cellMax));
        classed($('c-max').parentNode, 'warn', false);
        classed($('c-max').parentNode, 'bad', false);
        classed($('c-delta').parentNode, 'warn', pack.cellDelta > cfg.deltaWarn);
        classed($('c-delta').parentNode, 'bad', pack.cellDelta > cfg.deltaAlarm);
    }

    // Temperatures are always shown as numbers, so the toggle has no meaning.
    classed($('show-numbers').parentNode, 'hidden', view.mode === 'temps');

    var key = [view.pack, view.mode, view.numbers, cfg.modules, cfg.cells, cfg.temps].join(':');
    if (key !== view.layoutKey) {
        view.layoutKey = key;
        buildModules(cfg);
    }
    paintModules(pack, cfg);
}

/* ------------------------------- packs pane ----------------------------- */

function renderPacks(data) {
    var host = $('packs');
    var cfg = data.cfg;
    if (host.childElementCount !== cfg.packs) {
        host.textContent = '';
        for (var i = 0; i < cfg.packs; i++) {
            var card = document.createElement('div');
            card.className = 'card';
            var title = document.createElement('h2');
            title.textContent = 'Pack ' + (i + 1);
            var table = document.createElement('table');
            table.className = 'kv';
            var liveTitle = document.createElement('h2');
            liveTitle.textContent = 'Modules';
            liveTitle.style.marginTop = '14px';
            var live = document.createElement('div');
            live.className = 'liveness';
            card.appendChild(title);
            card.appendChild(table);
            card.appendChild(liveTitle);
            card.appendChild(live);
            host.appendChild(card);
        }
    }

    for (var p = 0; p < cfg.packs; p++) {
        var pack = data.packs[p];
        var card = host.children[p];
        fillTable(card.children[1], [
            ['Voltage', volts(pack.voltage)],
            ['Cell min / max', mvPair(pack.cellMin, pack.cellMax)],
            ['Cell delta', hasReading(pack.cellMin) ? pack.cellDelta + ' mV' : '\u2013',
                pack.cellDelta > cfg.deltaAlarm ? 'bad-text'
                    : (pack.cellDelta > cfg.deltaWarn ? 'warn-text' : '')],
            ['Temp min / max', tempPair(pack.tempMin, pack.tempMax)],
            ['Responding', yesNo(pack.alive, true).label, yesNo(pack.alive, true).css],
            ['Contactors', pack.inhibited ? 'Inhibited' : 'Permitted',
                pack.inhibited ? 'warn-text' : 'ok-text'],
            ['Contactor welded', yesNo(pack.welded, false).label, yesNo(pack.welded, false).css],
            ['Dead cell', yesNo(pack.deadCell, false).label, yesNo(pack.deadCell, false).css],
            ['Balancing', pack.balancing ? 'Yes, to ' + pack.balanceTarget + ' mV' : 'No'],
            ['CAN errors tx / rx', pack.canTx + ' / ' + pack.canRx,
                (pack.canTx || pack.canRx) ? 'warn-text' : '']
        ]);

        var live = card.children[3];
        if (live.childElementCount !== cfg.modules) {
            live.textContent = '';
            for (var m = 0; m < cfg.modules; m++) {
                var dot = document.createElement('div');
                dot.className = 'live-dot';
                dot.textContent = String(m + 1);
                live.appendChild(dot);
            }
        }
        for (var n = 0; n < cfg.modules; n++) {
            var module = pack.modules[n];
            // Amber means "talking but has not sent a full set of readings yet",
            // which is the normal state for the first sweep after a boot.
            var cls = !module.alive ? 'bad' : (module.pop ? 'ok' : 'warn');
            if (module.err) { cls = 'bad'; }
            live.children[n].className = 'live-dot ' + cls;
        }
    }
}

/* ------------------------------ system pane ----------------------------- */

function renderSystem(data) {
    text('y-state', data.state);
    text('y-timeinstate', duration(data.timeInState));
    text('y-uptime', duration(data.uptime));
    text('y-invalid', String(data.fault.invalidEvents));

    var illegal = yesNo(data.fault.illegalTransition, false);
    text('y-illegal', illegal.label);
    $('y-illegal').className = illegal.css;

    text('y-boot', data.fault.watchdogReboot ? 'Watchdog reset' : 'Clean');
    $('y-boot').className = data.fault.watchdogReboot ? 'bad-text' : 'ok-text';

    fillList($('y-internal'), decodeBits(data.fault.internal, INTERNAL_ERRORS), 'No internal errors');

    text('y-errorbyte', hex(data.fault.errorByte, 2) +
        (data.fault.errorByte ? ' \u2013 ' + decodeBits(data.fault.errorByte, ERROR_BITS).join(', ') : ''));
    text('y-statusbyte', hex(data.fault.statusByte, 2) +
        (data.fault.statusByte ? ' \u2013 ' + decodeBits(data.fault.statusByte, STATUS_BITS).join(', ') : ''));
    text('y-weldingbyte', hex(data.fault.weldingByte, 2) +
        (data.fault.weldingByte ? ' \u2013 ' + decodeBits(data.fault.weldingByte, WELDING_BITS).join(', ') : ''));
    $('y-weldingbyte').className = data.fault.weldingByte ? 'bad-text' : '';

    var shunt = data.shunt;
    var alive = yesNo(shunt.alive, true);
    text('y-shuntalive', alive.label);
    $('y-shuntalive').className = alive.css;
    text('y-shuntamps', amps(shunt.amps));
    text('y-shuntv1', volts(shunt.v1, 2));
    text('y-shuntv2', volts(shunt.v2, 2));
    text('y-shuntv3', volts(shunt.v3, 2));
    text('y-shuntw', kilowatts(shunt.watts));
    text('y-shuntas', (shunt.as / 3600).toFixed(2) + ' Ah');
    text('y-shuntwh', (shunt.wh / 1000).toFixed(2) + ' kWh');
    text('y-shunttemp', shunt.alive ? shunt.temp + ' \u00b0C' : '\u2013');

    var canRows = [['Main bus tx / rx', data.can.tx + ' / ' + data.can.rx,
                    (data.can.tx || data.can.rx) ? 'warn-text' : '']];
    for (var p = 0; p < data.cfg.packs; p++) {
        canRows.push(['Pack ' + (p + 1) + ' tx / rx',
                      data.packs[p].canTx + ' / ' + data.packs[p].canRx,
                      (data.packs[p].canTx || data.packs[p].canRx) ? 'warn-text' : '']);
    }
    fillTable($('y-can'), canRows);

    var cfg = data.cfg;
    fillTable($('y-cfg'), [
        ['Packs / modules / cells', cfg.packs + ' / ' + cfg.modules + ' / ' + cfg.cells],
        ['Temperature sensors', String(cfg.temps) + ' per module'],
        ['Cell empty / full', cfg.cellEmpty + ' / ' + cfg.cellFull + ' mV'],
        ['Dead cell below', cfg.cellDead + ' mV'],
        ['Delta warn / alarm', cfg.deltaWarn + ' / ' + cfg.deltaAlarm + ' mV'],
        ['Temp warn / max', cfg.tempWarn + ' / ' + cfg.tempMaxLimit + ' \u00b0C'],
        ['Min charge temp', cfg.tempChargeMin + ' \u00b0C'],
        ['Capacity', (cfg.capacityWh / 1000).toFixed(1) + ' kWh'],
        ['Firmware', cfg.version]
    ]);
}

/* ------------------------------ status pane ----------------------------- */

function renderStatus(data) {
    var cfg = data.cfg;

    // Banner
    var state = STATES[data.state] || { text: data.state.toUpperCase(), css: 'grey' };
    text('banner-state', state.text);
    text('banner-sub', 'for ' + duration(data.timeInState));
    $('banner').className = 'banner ' + state.css;

    // State of charge
    var soc = Math.max(0, Math.min(100, data.soc));
    var fill = $('soc-fill');
    fill.style.width = soc + '%';
    fill.className = 'bar-fill' + (soc <= 10 ? ' low' : (soc <= 25 ? ' mid' : ''));
    text('soc-text', soc + '%');
    text('soc-energy', ((cfg.capacityWh * soc / 100) / 1000).toFixed(1) + ' kWh of ' +
                       (cfg.capacityWh / 1000).toFixed(1) + ' kWh');

    // Headline numbers. The shunt is the authority on current and power; the
    // pack voltage comes from the modules.
    text('s-voltage', volts(data.voltage));
    text('s-current', data.shunt.alive ? amps(data.shunt.amps) : '\u2013');
    text('s-power', data.shunt.alive ? kilowatts(data.shunt.watts) : '\u2013');
    text('s-temp', tempPair(data.tempMin, data.tempMax));
    text('s-cells', mvPair(data.cellMin, data.cellMax));
    text('s-delta', hasReading(data.cellMin) ? data.cellDelta + ' mV' : '\u2013');

    classed($('s-temp').parentNode, 'warn', data.tempMax >= cfg.tempWarn);
    classed($('s-temp').parentNode, 'bad', data.tempMax >= cfg.tempMaxLimit);
    classed($('s-delta').parentNode, 'warn', data.cellDelta > cfg.deltaWarn);
    classed($('s-delta').parentNode, 'bad', data.cellDelta > cfg.deltaAlarm);

    // Alerts: only the things that need a human.
    var alerts = [];
    if (data.fault.weldingByte) {
        alerts.push(['Contactor welded: ' +
            decodeBits(data.fault.weldingByte, WELDING_BITS).join(', '), false]);
    }
    if (!data.batteryAlive)     { alerts.push(['Battery is not responding', false]); }
    if (data.flags.deadCell)    { alerts.push(['A cell is below the dead-cell threshold', false]); }
    if (data.flags.tooHot)      { alerts.push(['Battery too hot', false]); }
    if (data.fault.internal)    {
        decodeBits(data.fault.internal, INTERNAL_ERRORS).forEach(function (e) { alerts.push([e, true]); });
    }
    if (data.flags.imbalanced)  { alerts.push(['Packs are imbalanced', true]); }
    if (!data.shunt.alive)      { alerts.push(['Shunt is not responding', true]); }
    if (data.flags.tooCold)     { alerts.push(['Too cold to charge', true]); }
    if (data.cellDelta > cfg.deltaAlarm) { alerts.push(['Cell delta above alarm threshold', true]); }

    var host = $('alerts');
    var key = alerts.map(function (a) { return a[0]; }).join('|');
    if (host.dataset.key !== key) {
        host.dataset.key = key;
        host.textContent = '';
        alerts.forEach(function (a) {
            var div = document.createElement('div');
            div.className = 'alert' + (a[1] ? ' warn' : '');
            div.textContent = a[0];
            host.appendChild(div);
        });
    }
    classed(host, 'hidden', alerts.length === 0);

    // Drive and charge permission
    $('drive-head').textContent = data.drive.inhibited ? 'Drive blocked' : 'Drive allowed';
    classed($('drive-head'), 'blocked', data.drive.inhibited);
    fillList($('drive-reasons'), decodeBits(data.drive.reasons, INHIBIT_REASONS), 'No restrictions');

    $('charge-head').textContent = data.charge.inhibited ? 'Charge blocked' : 'Charge allowed';
    classed($('charge-head'), 'blocked', data.charge.inhibited);
    fillList($('charge-reasons'), decodeBits(data.charge.reasons, INHIBIT_REASONS), 'No restrictions');

    text('s-ignition', data.ignition ? 'On' : 'Off');
    $('s-ignition').className = data.ignition ? 'ok-text' : '';
    text('s-chargeenable', data.chargeEnable ? 'On' : 'Off');
    text('s-heater', data.heater ? 'On' : 'Off');
    $('s-heater').className = data.heater ? 'warn-text' : '';
    text('s-chargelimit', data.maxChargeCurrent + ' A');
    text('s-dischargelimit', data.maxDischargeCurrent + ' A');
}

/* --------------------------------- polling ------------------------------ */

function setLink(ok, message) {
    var dot = $('link-dot');
    dot.className = 'dot ' + (ok ? 'ok' : 'bad');
    text('link-text', message);
}

function render(data) {
    view.cfg = data.cfg;
    renderStatus(data);
    // Only the visible tab is worth painting; the others are repainted the
    // moment they are shown, on the next poll.
    if (view.tab === 'cells')  { renderCells(data); }
    if (view.tab === 'packs')  { renderPacks(data); }
    if (view.tab === 'system') { renderSystem(data); }
}

var pollTimer = null;
var inFlight = false;

function schedule() {
    clearTimeout(pollTimer);
    /* Nothing to draw for a screen that is off, and a phone in a pocket should
     * not keep the ESP32's radio busy once a second. visibilitychange restarts
     * the loop the moment the page comes back. */
    if (document.hidden) { return; }
    pollTimer = setTimeout(poll, POLL_MS);
}

function poll() {
    // Guard against a second loop being started by visibilitychange while a
    // request is already outstanding.
    if (inFlight) { return; }
    inFlight = true;

    var controller = new AbortController();
    var timer = setTimeout(function () { controller.abort(); }, FETCH_TIMEOUT_MS);

    fetch('/api/status', { signal: controller.signal, cache: 'no-store' })
        .then(function (response) {
            clearTimeout(timer);
            if (response.status === 503) { throw new Error('BMS is still starting up'); }
            if (!response.ok) { throw new Error('HTTP ' + response.status); }
            return response.json();
        })
        .then(function (data) {
            view.failures = 0;
            setLink(true, 'live \u00b7 up ' + duration(data.uptime));
            render(data);
        })
        .catch(function (error) {
            clearTimeout(timer);
            view.failures++;
            setLink(false, error.message === 'The user aborted a request.'
                ? 'no response' : error.message);
            /* Two misses is a blip -- WiFi on a phone does that. Three means
             * the link or the board is gone, and showing the last readings as
             * though they were current would be worse than saying so. */
            if (view.failures >= 3) {
                text('banner-state', 'NO CONNECTION');
                text('banner-sub', 'last seen ' + view.failures + ' polls ago');
                $('banner').className = 'banner grey';
            }
        })
        .then(function () {
            inFlight = false;
            schedule();
        });
}

document.addEventListener('visibilitychange', function () {
    if (document.hidden) { clearTimeout(pollTimer); } else { poll(); }
});

// Reopen on whichever tab the URL names, so a reload does not lose your place.
if (TABS.indexOf(location.hash.slice(1)) >= 0) {
    showTab(location.hash.slice(1));
}

poll();

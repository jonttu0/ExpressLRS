
const WSMSGID_ERROR_IND             = 0xCAFE;

const WSMSGID_ESPNOW_ADDRS          = 0x1100;

const WSMSGID_STM32_RESET           = 0x1200;

const WSMSGID_ELRS_SETTINGS         = 0x2200;
const WSMSGID_ELRS_DOMAIN           = 0x2201;
const WSMSGID_ELRS_RATE             = 0x2202;
const WSMSGID_ELRS_POWER            = 0x2203;
const WSMSGID_ELRS_TLM              = 0x2204;
const WSMSGID_ELRS_RF_POWER_TEST    = 0x2205;

const WSMSGID_HANDSET_MIXER         = 0x2300;
const WSMSGID_HANDSET_CALIBRATE     = 0x2301;
const WSMSGID_HANDSET_ADJUST        = 0x2302;
const WSMSGID_HANDSET_RELOAD        = 0x2303;
const WSMSGID_HANDSET_SAVE          = 0x2304;
const WSMSGID_HANDSET_BATT_INFO     = 0x2305;
const WSMSGID_HANDSET_BATT_CONFIG   = 0x2306;

const WSMSGID_HANDSET_TLM_LINK_STATS = 0x2380;
const WSMSGID_HANDSET_TLM_BATTERY    = 0x2381; // not used
const WSMSGID_HANDSET_TLM_GPS        = 0x2382;

const WSMSGID_VIDEO_FREQ         = 0x2400;


const ws_msgid_lookup = {
    // STM32 slave control
    "stm32_reset":       WSMSGID_STM32_RESET,

    "refresh":           WSMSGID_ELRS_SETTINGS,
    "rf_module":         WSMSGID_ELRS_DOMAIN,
    "rate":              WSMSGID_ELRS_RATE,
    "power":             WSMSGID_ELRS_POWER,
    "telemetry":         WSMSGID_ELRS_TLM,
    "rf_pwr":            WSMSGID_ELRS_RF_POWER_TEST,

    "handset_reload":    WSMSGID_HANDSET_RELOAD,
    "handset_save":      WSMSGID_HANDSET_SAVE,

    "vtx_freq":          WSMSGID_VIDEO_FREQ,
};

const elrs_settings_lookup = {
    0: {"rfidx": 1, "rates": ['200Hz', '100Hz', '50Hz'], "info": "915MHz"},
    1: {"rfidx": 1, "rates": ['200Hz', '100Hz', '50Hz'], "info": "868MHz"},
    2: {"rfidx": 1, "rates": ['200Hz', '100Hz', '50Hz'], "info": "433MHz"},
    3: {"rfidx": 2, "rates": ['500Hz', '250Hz', '125Hz', '50Hz'], "info": "2400MHz ISM (LoRa)"},
    4: {"rfidx": 2, "rates": ['500Hz', '250Hz', '125Hz', '50Hz'], "info": "2400MHz ISM (LoRa)"},
    5: {"rfidx": 3, "rates": ['1000Hz', '500Hz', '250Hz'], "info": "2400MHz ISM (FLRC)"},
    6: {"rfidx": 4, "rates": ['DVDA500Hz', 'DVDA250Hz', 'LORA500Hz', 'LORA250Hz'], "info": "2400MHz ISM (VANILLA 3.x)"},
};

function $id(id) {
    return document.getElementById(id);
}
function $class(id) {
    return document.getElementsByClassName(id);
}
function $name(name) {
    return document.getElementsByName(name);
}
function time_current() {
    const dateobj = new Date()
    return dateobj.toLocaleTimeString([], {hour12: false});
}
function datetime_current() {
    const dateobj = new Date()
    return dateobj.toLocaleTimeString([], {hour12: false}) + '.' +
            `00${dateobj.getMilliseconds()}`.slice(-3);
}
function int2str_pad(num, size=2, base=10) {
    return num.toString(base).padStart(size,"0");
}
DataView.prototype.nextUint8 = function () {
    if (this.offset_next == undefined) this.offset_next = 0;
    const idx = this.offset_next; this.offset_next += 1;
    try {
        return this.getUint8(idx);
    } catch(e) {
        console.error("error! nextUint8: idx: %o, buffer: %o", idx, this);
        return undefined;
    }
};
DataView.prototype.nextInt8 = function () {
    if (this.offset_next == undefined) this.offset_next = 0;
    const idx = this.offset_next; this.offset_next += 1;
    try {
        return this.getInt8(idx);
    } catch(e) {
        console.error("error! getInt8: idx: %o, buffer: %o", idx, this);
        return undefined;
    }
};
DataView.prototype.nextUint16 = function (little_endian=false) {
    if (this.offset_next == undefined) this.offset_next = 0;
    const idx = this.offset_next; this.offset_next += 2;
    try {
        return this.getUint16(idx, little_endian);
    } catch(e) {
        console.error("error! nextUint16: idx: %o, buffer: %o", idx, this);
        return undefined;
    }
};
DataView.prototype.nextInt16 = function (little_endian=false) {
    if (this.offset_next == undefined) this.offset_next = 0;
    const idx = this.offset_next; this.offset_next += 2;
    try {
        return this.getInt16(idx, little_endian);
    } catch(e) {
        console.error("error! nextUint16: idx: %o, buffer: %o", idx, this);
        return undefined;
    }
};
DataView.prototype.nextInt32 = function (little_endian=false) {
    if (this.offset_next == undefined) this.offset_next = 0;
    const idx = this.offset_next; this.offset_next += 4;
    try {
        return this.getInt32(idx, little_endian);
    } catch(e) {
        console.error("error! nextInt32: idx: %o, buffer: %o", idx, this);
        return undefined;
    }
};
function bufferToHex (buffer) {
    return [...new Uint8Array (buffer)]
        .map (b => b.toString (16).padStart (2, "0"))
        .join ("");
}

var websock = null;
function start() {
    const logger = $id("logField");
    var _bands = $id("vtx_band");
    while (_bands.length > 1) {
        _bands.remove(_bands.length - 1);
    }
    for (const band in channelFreqTable) {
      var option = document.createElement("option");
      option.text = option.value = band;
      _bands.add(option);
    }

    // configure events
    if (!!window.EventSource) {
        var source = new EventSource('/events');
        source.addEventListener('open', function(e) {/*console.log("Events Connected");*/}, false);
        source.addEventListener('error', function(e) {
            if (e.target.readyState != EventSource.OPEN) {/*console.log("Events Disconnected");*/}}, false);
        source.addEventListener('message', function(e) {console.log("message", e.data);}, false);
        source.addEventListener('elrs_version', function(e) {$id("firmware_version_elrs").innerHTML = e.data;}, false);
        source.addEventListener('elrs_settings', function(e) {console.log("elrs settings:", e);
            const settings = JSON.parse(e.data);
            handle_settings(settings);
            handset_mixer(settings['mixer']);
            handset_calibrate_adjust(settings['gimbals']);
        }, false);
    }

    logger.scrollTop = logger.scrollHeight;
    if (false) {
        websock = new WebSocket('ws://elrs_handset.local:/ws');
    } else {
        if (!window.location.hostname)
          return;
        websock = new WebSocket('ws://' + window.location.hostname + '/ws');
    }
    websock.binaryType = "arraybuffer";
    websock.onopen = function (evt) {console.log('websock open');};
    websock.onclose = function(e) {
        console.log('websock closed. Reconnect in 1 second.', e.reason);
        setTimeout(function() {start();}, 1000);
    };
    websock.onerror = function (evt) {console.log("websock error: ", evt);};
    websock.onmessage = function (evt) {
        if (evt.data instanceof ArrayBuffer) {
            const message = new DataView(evt.data);
            const msgid = message.getUint16();
            const payload = new DataView(evt.data, 2);
            switch(msgid) {
                case WSMSGID_ESPNOW_ADDRS: espnowclients_parse(payload);break;
                case WSMSGID_ELRS_SETTINGS: handle_settings(settings_parse(payload)); break;
                case WSMSGID_VIDEO_FREQ: msp_vtx_freq(payload.getUint16()); break;
                case WSMSGID_HANDSET_CALIBRATE:
                    if (calibrate_btn != null) {
                        calibrate_btn.disabled = false;
                        calibrate_btn = null;
                        $id("handset_calibrate_stat").innerHTML = 'Calibration failed!';
                    } break;
                case WSMSGID_HANDSET_BATT_INFO: handset_battery_value(payload); break;
                case WSMSGID_HANDSET_ADJUST: handset_calibrate_adjust(handset_calibrate_adjust_to_dict(payload)); break;
                case WSMSGID_HANDSET_MIXER: handset_mixer(mixer_list_to_dict(payload)); break;
                case WSMSGID_HANDSET_TLM_LINK_STATS: handset_telemetry_link_stats(payload); break;
                case WSMSGID_HANDSET_TLM_BATTERY: handset_telemetry_battery(payload); break;
                case WSMSGID_HANDSET_TLM_GPS: handset_telemetry_gps(payload); break;
                default: console.error("Invalid message received: " + msgid);
            }
            return;
        }
        const text = evt.data;
        if (!text) return; // ignore empty messages

        if (text.startsWith("CMD_WIFINETS")) {
            wifinetworks_parse(text);
            return;
        } else if (text.startsWith("CMD_LAPTIMER=")) {
            laptimernet_parse(text.replace("CMD_LAPTIMER=", ""));
            return;
        }

        const scrollsize = parseInt($id("scrollsize").value, 10);
        var log_history = logger.value.split("\n");
        while (scrollsize < log_history.length) {log_history.shift();}
        log_history.push(datetime_current() + ' ' + text);
        logger.value = log_history.join('\n');
        if ($id("autoscroll").checked)
            logger.scrollTop = logger.scrollHeight;
    };
}

function saveTextAsFile() {
    var textToWrite = $id('logField').value;
    var textFileAsBlob = new Blob([textToWrite], { type: 'text/plain' });
    var downloadLink = document.createElement("a");
    downloadLink.download = "tx_log.txt";
    downloadLink.innerHTML = "Download File";
    if (window.webkitURL != null) {
        // Chrome allows the link to be clicked without actually adding it to the DOM.
        downloadLink.href = window.webkitURL.createObjectURL(textFileAsBlob);
    } else {
        // Firefox requires the link to be added to the DOM before it can be clicked.
        downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
        downloadLink.onclick = function(event) {document.body.removeChild(event.target);};
        downloadLink.style.display = "none";
        document.body.appendChild(downloadLink);
    }
    downloadLink.click();
}

function message_send(elem=null, bytesize=1)
{
    var sendarray = new ArrayBuffer(2 + ((elem == null) ? 0 : bytesize));  // MSGID + value
    var view = new Uint8Array(sendarray);

    if (elem) {
        const msg_id = ws_msgid_lookup[elem.name];
        view[0] = msg_id & 0xFF;
        view[1] = (msg_id >> 8) & 0xFF;
        for (var iter = 0; iter < bytesize; iter++)
            view[2+iter] = (elem.value >> (8*iter)) & 0xFF;
    } else {
        view[0] = WSMSGID_ELRS_SETTINGS & 0xFF;
        view[1] = (WSMSGID_ELRS_SETTINGS >> 8) & 0xFF;
    }
    if (websock_validate()) websock.send(sendarray, { binary: true });
}

function rf_module_changed(elem)
{
    $id("rf_module").disabled = true;
    $id("rates_input").disabled = true;
    $id("power_input").disabled = true;
    $id("tlm_input").disabled = true;
    message_send(elem);
}

function handle_setting_region(domain)
{
    if (domain == undefined) return;
    const FLAGS = {
        dual: 0x80,
        handset: 0x40,
        mask: 0x3F,
    };

    const rf_module = $id("rf_module");
    if (!(domain & FLAGS.dual)) {
        /* Disable RF selection if not dual module */
        rf_module.disabled = true;
    } else {
        rf_module.disabled = false;
    }

    if (!(domain & FLAGS.handset)) {
        /* Disable tabs if not handset */
        var tabs = $name('handset');
        for (var tab of tabs) {
            tab.className += " disabled";
        }
    }
    domain = domain & FLAGS.mask;

    const domain_params = elrs_settings_lookup[domain];
    if (domain_params == undefined) {
        rf_module.disabled = true;
        $id("rates_input").disabled = true;
        $id("power_input").disabled = true;
        $id("tlm_input").disabled = true;
        $id("region_domain").innerHTML = "Invalid domain: " + domain;
        return false;
    }

    // Store domain index for later use
    rf_module.domain_in_use = domain;
    rf_module.selectedIndex = domain_params.rfidx;
    $id("region_domain").innerHTML = "Domain " + domain_params.info;

    // update rate options
    const rates = $id("rates_input");
    while (rates.length > 0) {
        rates.remove(rates.length-1);
    }
    const options = domain_params.rates;
    for (i = 0; i < options.length; i++) {
        var option = document.createElement("option");
        option.text = options[i];
        option.value = i;
        rates.add(option);
    }
    return true;
}

function handle_setting_generic(elem, value, max_value=null)
{
    if (value == undefined) return;
    if (max_value != null) {
        max_value = max_value + 2; // include reset and dummy
        // enable all
        for (var i = 0; i < elem.length; i++)
            elem.options[i].disabled = false;
        // disable unavailable values
        for (var i = (elem.length-1); max_value < i; i--)
            elem.options[i].disabled = true;
    }
    value = value.toString();
    elem.selectedIndex = [...elem.options].findIndex (option => option.value === value);
    elem.disabled = false;
}

function handle_settings(settings)
{
    if (handle_setting_region(settings.region)) {
        handle_setting_generic($id("rates_input"), settings.rate);
        handle_setting_generic($id("power_input"), settings.power, settings.power_max);
        handle_setting_generic($id("tlm_input"), settings.telemetry);
        // disable telemetry options if vanilla mode
        $id("tlm_input").disabled = ($id("rf_module").domain_in_use == 6);
    }
    msp_vtx_freq(settings.vtxfreq);
}

function settings_parse(payload)
{
    const settings = {
        "region": payload.nextUint8(),
        "rate": payload.nextUint8(),
        "power": payload.nextUint8(),
        "power_max": payload.nextUint8(),
        "telemetry": payload.nextUint8(),
        "vtxfreq": (payload.nextUint8() << 8) + payload.nextUint8(),
    };
    return settings;
}

/********************* VTX *******************************/

// Supported bands and channels
const channelFreqTable = {
    "A": [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725], // A
    "B": [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866], // B
    "E": [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945], // E
    "F": [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880], // F / Airwave
    "R": [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], // R / Immersion Raceband
    "L": [5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621], // L
    "U": [5325, 5348, 5366, 5384, 5402, 5420, 5438, 5456], // U
    "O": [5474, 5492, 5510, 5528, 5546, 5564, 5582, 5600], // O
    "H": [5653, 5693, 5733, 5773, 5813, 5853, 5893, 5933]  // H
};

function vtx_show_freq()
{
    const band = $id("vtx_band").value;
    const ch = $id("vtx_channel").value;
    if (band == "" || ch == "") {
        $id("vtx_send_btn").disabled = true;
        $id("vtx_freq").innerHTML = "";
        return;
    }
    const freq = channelFreqTable[band][parseInt(ch, 10)];
    $id("vtx_freq").innerHTML = "" + freq + " MHz";
    $id("vtx_send_btn").disabled = !(0 < freq);
}

var vtx_last_band = "-";
function vtx_band_changed(band)
{
    if (band != vtx_last_band) {
        vtx_last_band = band;
        $id("vtx_band").value = band;
        var channels = $id("vtx_channel");
        while (channels.length > 1) {
            channels.remove(channels.length - 1);
        }
        for (const ch_idx in channelFreqTable[band]) {
            var option = document.createElement("option");
            option.text = parseInt(ch_idx, 10) + 1;
            option.value = ch_idx;
            channels.add(option);
        }
        $id("vtx_channel").value = "";
    }
    vtx_show_freq();
}

function msp_vtx_freq(freq)
{
    $id("vtx_send_btn").disabled = true;
    if (freq == 0 || freq == undefined) {
        // Clear selections
        $id("vtx_band").value = "";
        $id("vtx_channel").value = "";
        $id("vtx_freq").innerHTML = "";
        return;
    }
    for (const band in channelFreqTable) {
        const channels = channelFreqTable[band];
        for (const ch in channels) {
            if (freq == channels[ch]) {
                vtx_band_changed(band);
                $id("vtx_channel").value = ch;
                $id("vtx_freq").innerHTML = "" + freq + " MHz";
                return;
            }
        }
    }
}

function msp_vtx_freq_send()
{
    var band = $id("vtx_band").value;
    var ch = $id("vtx_channel").value;
    if (band == "" || ch == "")
        return;
    var elem = Object();
    elem.name = "vtx_freq";
    elem.value = channelFreqTable[band][parseInt(ch, 10)];
    message_send(elem, 2);
}


/********************* STM32 *******************************/

function command_stm32(type)
{
    var elem = Object();
    elem.name = "stm32_" + type;
    message_send(elem, 0);
}


/********************* HANDSET MIXER *********************/

function mixer_list_to_dict(payload)
{
    var mixer_config = {};
    mixer_config['aux'] = payload.nextUint8();
    mixer_config['total'] = mixer_config['aux'] + 4;
    mixer_config['switch'] = payload.nextUint8();
    const config = [];
    for (var i = 0; i < 16; i++) {
        config.push({'index': -1, 'inv': false, 'scale': 1.0});
    }
    for (var i = 0; i < 16 && i < ((payload.byteLength - 2) / 4); i++) {
        const idx = payload.nextUint8();
        config[idx].index = payload.nextUint8();
        config[idx].inv = payload.nextUint8();
        config[idx].scale = payload.nextUint8();
    }
    mixer_config['config'] = config;
    return mixer_config;
}

function mixer_add_selection_lst(cell, input=-1, values={})
{
    var option;
    var sel = document.createElement("select");
    sel.style.width = "100px";

    option = document.createElement("option");
    option.value = -1;
    option.text = "";
    sel.add(option, option.value);

    /* add options */
    for (const opt in values) {
        option = document.createElement("option");
        option.value = values[opt];
        option.text = opt;
        sel.add(option, values[opt]);
    }
    /* set selected */
    sel.selectedIndex = input;
    cell.appendChild(sel);
}

function handset_mixer(mixes)
{
    if (mixes == undefined) return;
    var iter;
    var table = $id("handset_mixer");

    var gimbals = {
        'Gimbal L1': 0, 'Gimbal L2': 1,
        'Gimbal R1': 2, 'Gimbal R2': 3};
    var switches = {};
    for (iter = 0; iter < mixes['switch']; iter++) {
        switches['Switch ' + (iter+1)] = iter;
    }

    /* clean table */
    while (table.rows.length) {
        table.deleteRow(table.rows.length-1);
    }
    /* write values */
    for (iter = 0; iter < mixes['total']; iter++) {
        const mix_cfg = mixes['config'][iter];
        var row = table.insertRow();
        var cell = row.insertCell(0);
        if (iter < 4)
            cell.innerHTML = "Analog " + (iter + 1);
        else
            cell.innerHTML = "AUX " + (iter - 3);
        cell = row.insertCell(1);
        cell.style.width = "auto";
        mixer_add_selection_lst(cell, mix_cfg.index,
            ((iter < 4) ? gimbals : switches));

        cell = row.insertCell(2);
        cell.style.width = "100px";
        // creating checkbox element
        var checkbox = document.createElement('input');
        checkbox.type = "checkbox";
        checkbox.name = "invert";
        checkbox.value = iter;
        checkbox.id = "inverted" + iter;
        checkbox.checked = mix_cfg.inv ? true : false;
        var label = document.createElement('label');
        label.htmlFor = checkbox.id;
        label.appendChild(document.createTextNode('Inverted:'));
        // creating label for checkbox
        cell.appendChild(label);
        cell.appendChild(checkbox);

        cell = row.insertCell(3);
        cell.style.width = "110px";
        if (iter < 4) {
            var scale = document.createElement("input");
            scale.type = "number";
            scale.name = "scale";
            scale.min = 0.1;
            scale.max = 1.0;
            scale.step = 0.05;
            scale.value = (mix_cfg.scale) ? (mix_cfg.scale.toFixed(4) / 100.) : 1.0;
            scale.id = "scale" + iter;
            scale.style.width = "50px";
            // creating label for checkbox
            label = document.createElement('label');
            label.htmlFor = scale.id;
            label.style.width = "40px";
            label.appendChild(document.createTextNode('Scale:'));
            cell.appendChild(label);
            cell.appendChild(scale);
        }
    }
}

function handset_mixer_send()
{
    var table = $id("handset_mixer");
    var output = [];
    for (var index = 0; index < table.rows.length; index++) {
        var rows = table.rows[index];
        var selected = rows.cells[1].getElementsByTagName("select")[0];
        if (-1 < selected.selectedIndex) {
            selected = selected.options[selected.selectedIndex].value;
            if (selected < 0) continue;
            /* Channel index */
            output.push(index);
            /* Output channel */
            output.push(selected);
            /* Inverted */
            var _in = rows.cells[2].getElementsByTagName("input")[0];
            output.push(_in.checked ? 1 : 0);

            if (index < 4) {
                /* add scale */
                _in = rows.cells[3].getElementsByTagName("input")[0];
                _in = parseFloat(_in.value) * 100;
                if (100 <= _in)
                    _in = 0;
                else if (_in <= 10)
                    _in = 10;
                output.push(_in);
            }
        }
    }
    var sendarray = new ArrayBuffer(2 + output.length);
    var view = new Uint8Array(sendarray);
    view[0] = WSMSGID_HANDSET_MIXER & 0xFF;
    view[1] = (WSMSGID_HANDSET_MIXER >> 8) & 0xFF;
    for (var i = 0; i < output.length; i++) view[2+i] = output[i];
    if (websock_validate()) websock.send(sendarray, { binary: true });
    console.log(view);
}

/********************* CALIBRATE *****************************/
const mapping_calibrate = {
    "L1_min": 0x01, "L1_mid": 0x02, "L1_max": 0x03,
    "L2_min": 0x11, "L2_mid": 0x12, "L2_max": 0x13,
    "R1_min": 0x21, "R1_mid": 0x22, "R1_max": 0x23,
    "R2_min": 0x31, "R2_mid": 0x32, "R2_max": 0x33,
};
var calibrate_btn = null;
function handset_calibrate_auto_send(btn, type)
{
    if (calibrate_btn != null) {
        return;
    }
    btn.disabled = true;
    calibrate_btn = btn;

    var sendarray = new ArrayBuffer(3);
    var view = new Uint8Array(sendarray);
    view[0] = WSMSGID_HANDSET_CALIBRATE & 0xFF;
    view[1] = (WSMSGID_HANDSET_CALIBRATE >> 8) & 0xFF;
    view[2] = mapping_calibrate[type];
    if (websock_validate()) websock.send(sendarray, { binary: true });
}

function handset_calibrate_adjust_send(event)
{
    var value = parseInt(event.target.value, 10);
    if (value < 0) {
        event.target.value = value = 0;
    } else if (value > 4095) {
        event.target.value = value = 4095;
    }

    var sendarray = new ArrayBuffer(5);
    var view = new Uint8Array(sendarray);
    view[0] = WSMSGID_HANDSET_ADJUST & 0xFF;
    view[1] = (WSMSGID_HANDSET_ADJUST >> 8) & 0xFF;
    view[2] = mapping_calibrate[event.target.id];
    view[3] = event.target.value & 0xFF;
    view[4] = (event.target.value >> 8) & 0xFF;
    if (websock_validate()) websock.send(sendarray, { binary: true });
}

function handset_calibrate_adjust_to_dict(payload)
{
    const config = [];
    for (var iter = 0; iter < 4; iter++) {
        const offset = iter * 6; // 3 * 2B
        config[iter] = {
            'min': payload.getUint16(offset+0, true),
            'mid': payload.getUint16(offset+2, true),
            'max': payload.getUint16(offset+4, true),
        };
    }
}

function handset_calibrate_adjust(calibration)
{
    if (calibration == undefined) return;
    //const calib_table = $id("calibrate");
    //for (var row of calib_table.rows)
    //    console.log(row);
    const mapping = {
        0 : 'L1_', 1 : 'L2_',
        2 : 'R1_', 3 : 'R2_',
    };
    for (const iter in calibration) {
        const type = mapping[iter];
        const values = calibration[iter];
        $id(type + 'min').value = values['min'];
        $id(type + 'mid').value = values['mid'];
        $id(type + 'max').value = values['max'];
    }
    if (calibrate_btn != null) {
        calibrate_btn.disabled = false;
        calibrate_btn = null;
        $id("handset_calibrate_stat").innerHTML = 'Calibration ready!';
    }
}

/******************** BATTERY MONITORING *********************/
function handset_battery_value(payload)
{
    $id("battery_voltage").innerHTML =
        (payload.getUint32(0) / 1000.).toFixed(1) + "V";
    $id("battery_scale").value = payload.getUint8(4);
    $id("battery_warning").value = payload.getUint8(5);
}

function handset_battery_adjust() {
    var sendarray = new ArrayBuffer(4);
    var view = new Uint8Array(sendarray);
    view[0] = WSMSGID_HANDSET_BATT_CONFIG & 0xFF;
    view[1] = (WSMSGID_HANDSET_BATT_CONFIG >> 8) & 0xFF;
    view[2] = parseInt($id("battery_scale").value, 10);
    view[3] = parseInt($id("battery_warning").value, 10);
    if (websock_validate()) websock.send(sendarray, { binary: true });
}

/********************* TELEMETRY *****************************/
var gps_mode = "kmh";
function convert_gps_speed(speed)
{
    speed = parseInt(speed, 10);
    if (gps_mode == "kmh") {
        speed = ((speed) * 36 / 1000);
        speed = speed.toString() + " km/h";
    } else {
        speed = ((speed) * 10000 / 5080 / 88);
        speed = speed.toString() + " mp/h";
    }
    return speed;
}

function convert_gps_value(val)
{
    val = parseInt(val, 10).toFixed(2);
    const GPS_DEGREES_DIVIDER = 10000000.;
    const result = (val / GPS_DEGREES_DIVIDER).toString();
    return result;
}

function handset_telemetry_link_stats(payload)
{
    $id("tlm_ul_RSSI1").innerHTML = payload.nextInt8().toString() + " dBm";
    $id("tlm_ul_RSSI2").innerHTML = payload.nextInt8().toString() + " dBm";
    $id("tlm_ul_LQ").innerHTML = payload.nextUint8();
    $id("tlm_ul_SNR").innerHTML = (payload.nextInt8().toFixed(1) / 10.).toString() + " dB";
    //payload.nextUint8(); // active_antenna
    //payload.nextUint8(); // rf_Mode
    //payload.nextUint8(); // uplink_TX_Power
    payload.offset_next += 3; // skip unused values
    $id("tlm_dl_RSSI1").innerHTML = payload.nextInt8().toString() + " dBm";
    $id("tlm_dl_LQ").innerHTML = payload.nextUint8();
    $id("tlm_dl_SNR").innerHTML = (payload.nextInt8().toFixed(1) / 10.).toString() + " dB";
    const now = time_current();
    $id("tlm_ul_updated").innerHTML = now;
    $id("tlm_dl_updated").innerHTML = now;

    handset_telemetry_battery(payload, now);
}

function handset_telemetry_battery(payload, now=null)
{
    $id("tlm_batt_V").innerHTML = (payload.nextUint16().toFixed(1) / 10.).toString() + " V";
    $id("tlm_batt_A").innerHTML = (payload.nextUint16().toFixed(1) / 10.).toString() + " A";
    const capacity = (payload.nextUint8() << 16) + (payload.nextUint8() << 8) + payload.nextUint8();
    $id("tlm_batt_C").innerHTML = capacity.toString() + " mAh";
    $id("tlm_batt_R").innerHTML = payload.nextUint8().toString() + " %";
    if (now == null) now = time_current();
    $id("tlm_batt_updated").innerHTML = now;
}

function handset_telemetry_gps(payload)
{
    $id("tlm_gps_lat").innerHTML = convert_gps_value(payload.nextInt32(true));
    $id("tlm_gps_lon").innerHTML = convert_gps_value(payload.nextInt32(true));
    $id("tlm_gps_speed").innerHTML = convert_gps_speed(payload.nextUint16(true));
    $id("tlm_gps_head").innerHTML = (payload.nextUint16(true).toFixed(1) / 10.).toString() + " deg";
    $id("tlm_gps_alt").innerHTML = (payload.nextInt16(true) - 1000).toString() + " m";
    $id("tlm_gps_sat").innerHTML = payload.nextUint8();
    $id("tlm_gps_updated").innerHTML = time_current();
}

/********************* ESP-NOW *****************************/
function espnowclients_send() {
    const clients = $id("espnowclients").value.split("\n");
    var bytes = [];
    for (const client of clients) {
        if (!client || client.length == 0) continue;
        const mac = client.split(":").map(function (val) { return parseInt(val, 16) });
        if (mac.length != 6) { console.error("Invalid MAC address: %s", client); return; }
        mac.forEach(element => { bytes.push(element); });
    }
    var sendarray = new ArrayBuffer(bytes.length + 2);
    var view = new Uint8Array(sendarray);
    view[0] = WSMSGID_ESPNOW_ADDRS & 0xFF; // Little endian 0x1100
    view[1] = (WSMSGID_ESPNOW_ADDRS >> 8) & 0xFF;
    for (var i = 0; i < bytes.length; i++) view[2+i] = bytes[i];
    if (websock_validate()) websock.send(sendarray, { binary: true });
}

function espnowclients_parse(value) {
    var target = $id("espnowclients");
    target.value = "";
    if (typeof (value) == "string") target.value = value.split(",").join("\n");
    else if (value instanceof DataView) { // binary data
        for (var mter = 0; mter < Math.floor(value.byteLength / 6); mter++) {
            var mac = "";
            for (var iter = 0; iter < 6; iter++) {
                if (0 < iter) mac += ":";
                mac += value.getUint8(mter * 6 + iter).toString(16).padStart(2, "0");
            }
            target.value += mac + "\n";
        }
    } else {
        target.value = "data error! " + typeof (value) + "\n";
    }
    // Resize the text field
    target.rows = target.value.split('\n').length;
}

function espnowclients_autosize(el){
    el.rows = el.value.split('\n').length;
}

/********************* WiFi Nets *****************************/
function wifinetworks_parse(value) {
    var table = $id("wifinetworks");
    /* Parse input message */
    var networks = [];
    /* clean table */
    while (table.rows.length) {
        table.deleteRow(table.rows.length-1);
    }
    if (value == "CMD_WIFINETS")
        return; // empty list
    const parts = value.replace("CMD_WIFINETS/\\/\\", "").split('/\\/\\');
    for (const network_str of parts) {
        networks.push({
            "idx": network_str.substring(0, 2),
            "mac": network_str.substring(2, 19),
            "ssid": network_str.substring(19),
        });
    }
    for (const network of networks) {
        const row = table.insertRow();
        row.esp_index = network["idx"];
        // Show SSID
        var cell = row.insertCell(0);
        cell.style.width = "auto";
        if (network["ssid"]) cell.innerHTML = network["ssid"];
        else if (network["mac"] != "00:00:00:00:00:00") cell.innerHTML = network["mac"];
        else cell.innerHTML = "ERROR - No SSID or MAC defined!";
        // Add remove button
        cell = row.insertCell(1);
        cell.style = "width:50px;";
        var btn = document.createElement('button');
        btn.style = cell.style;
        btn.innerHTML = "DEL";
        btn.onclick = wifinetworks_del;
        cell.appendChild(btn);
    }
}
function wifinetworks_add(event) {
    const ssid = $id("wifi_new_ssid").value;
    const mac = $id("wifi_new_mac").value;
    if (!ssid && !mac) {
        show_error("SSID or MAC is mandatory!");
        return;
    }
    if (websock_validate()) {
        const psk = $id("wifi_new_psk").value;
        websock.send(
            "WIFIADD/" +
            int2str_pad(ssid.length, 2) + "/" + ssid + "/" +
            int2str_pad(psk.length, 2) + "/" + psk +
            (mac ? "/" + mac.split(":").join("") : "")
        );
    }
}
function wifinetworks_del(event) {
    if (websock_validate()) {
        websock.send("WIFIDEL/" + int2str_pad(event.target.parentElement.parentElement.esp_index, 2));
        event.target.disabled = true;
    }
}
function laptimernet_parse(network_str) {
    if (network_str == "") return;
    const network = {"mac": network_str.substring(0, 17), "ssid": network_str.substring(17)}
    // Not used, ignore for now
}

function websock_validate() {
    if (!websock || websock.readyState !== WebSocket.OPEN) {
        show_error("Failed to communicate with handset, websocket connection is closed!");
        return false;
    }
    return true;
}

function show_error(msg) {
    console.log(msg);
    alert(msg);
}

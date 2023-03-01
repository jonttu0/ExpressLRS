
export const WSMSGID_ERROR_IND             = 0xCAFE;

export const WSMSGID_ESPNOW_ADDRS          = 0x1100;

export const WSMSGID_STM32_RESET           = 0x1200;

export const WSMSGID_ELRS_SETTINGS         = 0x2200;
export const WSMSGID_ELRS_DOMAIN           = 0x2201;
export const WSMSGID_ELRS_RATE             = 0x2202;
export const WSMSGID_ELRS_POWER            = 0x2203;
export const WSMSGID_ELRS_TLM              = 0x2204;
export const WSMSGID_ELRS_RF_POWER_TEST    = 0x2205;

export const WSMSGID_HANDSET_MIXER         = 0x2300;
export const WSMSGID_HANDSET_CALIBRATE     = 0x2301;
export const WSMSGID_HANDSET_ADJUST        = 0x2302;
export const WSMSGID_HANDSET_RELOAD        = 0x2303;
export const WSMSGID_HANDSET_SAVE          = 0x2304;
export const WSMSGID_HANDSET_BATT_INFO     = 0x2305;
export const WSMSGID_HANDSET_BATT_CONFIG   = 0x2306;

export const WSMSGID_HANDSET_TLM_LINK_STATS = 0x2380;
export const WSMSGID_HANDSET_TLM_BATTERY    = 0x2381; // not used
export const WSMSGID_HANDSET_TLM_GPS        = 0x2382;

export const WSMSGID_VIDEO_FREQ            = 0x2400;
export const WSMSGID_RECORDING_CTRL        = 0x2401;

export const WSMSGID_LAPTIMER_START_STOP   = 0x2500;
export const WSMSGID_LAPTIMER_LAPTIME      = 0x2501;

const message_id_map = {
    "ERROR_IND": WSMSGID_ERROR_IND,
    "ESPNOW_ADDR": WSMSGID_ESPNOW_ADDRS,
    "STM32_RESET": WSMSGID_STM32_RESET,
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
function $class_add(obj, type) {
    if(obj) obj.classList.add(type);
}
function $class_del(obj, type) {
    if(obj) obj.classList.remove(type);
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
function show_error(msg) {
    console.log(msg);
    alert(msg);
}
function bufferToHex(buffer) {
    return [...new Uint8Array (buffer)]
        .map (b => b.toString (16).padStart (2, "0"))
        .join ("");
}
function int2str_pad(num, size=2, base=10) {
    return num.toString(base).padStart(size,"0");
}
function msToTime(ms, hours=false) {
    // Pad to 2 or 3 digits, default is 2
    var pad = (n, z = 2) => ('00' + n).slice(-z);
    var resp = "";
    if (hours) resp = pad(ms/3.6e6|0) + ':';
    return resp + pad((ms%3.6e6)/6e4 | 0) + ':' + pad((ms%6e4)/1000|0) + '.' + pad(ms%1000, 3);
}

export const generics = {
    "$id": $id, "$class": $class, "$name": $name,
    "$class_add": $class_add, "$class_del": $class_del,
    "time_current": time_current, "datetime_current": datetime_current,
    "show_error": show_error, "bufferToHex": bufferToHex, "int2str_pad": int2str_pad,
};
for (const [key, value] of Object.entries(generics)) { window[key] = value; }

/******************* PROTOTYPES ***********************/

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
DataView.prototype.nextUint32 = function (little_endian=false) {
    if (this.offset_next == undefined) this.offset_next = 0;
    const idx = this.offset_next; this.offset_next += 4;
    try {
        return this.getUint32(idx, little_endian);
    } catch(e) {
        console.error("error! nextUint32: idx: %o, buffer: %o", idx, this);
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

/********************* CMN INIT *************************/
export function common_init() {
    const _bands = $id("vtx_band")
    while (_bands.length > 1) { _bands.remove(_bands.length - 1); }
    for (const band in vtx_table_get()) {
        const option = document.createElement("option");
        option.text = option.value = band;
        _bands.add(option);
    }

    const logger = $id("logField");
    logger.scrollTop = logger.scrollHeight;
}

/********************* WEB EVENTS *************************/
export function events_init(events) {
    if (!!window.EventSource) {
        var source = new EventSource('/events');
        source.addEventListener('open', function(e) {/*console.log("Events Connected");*/}, false);
        source.addEventListener('error', function(e) {
            if (e.target.readyState != EventSource.OPEN) {/*console.log("Events Disconnected");*/}}, false);
        source.addEventListener('message', function(e) {console.log("message", e.data);}, false);
        source.addEventListener('laptimer', function(e) {
            const config = JSON.parse(e.data);
            laptimer_info_handle(config.laptimer);}, false);
        for (var e in events) {
            source.addEventListener(e, events[e], false);
        }
    }
}

/********************* WEB SOCKET *************************/
var _websock = null;
var _callback = null;
export function websocket_init(callback) {
    _callback = callback;

    if (false) {
        _websock = new WebSocket('ws://elrs_handset.local:/ws');
    } else {
        if (!window.location.hostname)
            return;
        _websock = new WebSocket('ws://' + window.location.hostname + '/ws');
    }
    _websock.binaryType = "arraybuffer";
    _websock.onopen = function (evt) {/*console.log('websock open');*/};
    _websock.onclose = function(e) {
        /*console.log('websock closed. Reconnect in 1 second.', e.reason);*/
        setTimeout(function() {websocket_init(_callback);}, 1000);
    };
    _websock.onerror = function (evt) {/*console.log("websock error: ", evt);*/};
    _websock.onmessage = function (evt) {
        if (evt.data instanceof ArrayBuffer) {
            const message = new DataView(evt.data);
            const msgid = message.getUint16();
            const payload = new DataView(evt.data, 2);
            switch(msgid) {
                case WSMSGID_ESPNOW_ADDRS:
                    espnowclients_parse(payload);
                    break;
                case WSMSGID_VIDEO_FREQ:
                    msp_vtx_freq(payload.nextUint16());
                    break;
                case WSMSGID_LAPTIMER_START_STOP:
                    laptimer_ctrl_state(payload.nextUint16(), payload.nextUint8());
                    break;
                case WSMSGID_LAPTIMER_LAPTIME:
                    laptimer_laptime_parse(payload);
                    break;
                default:
                    if (_callback) {
                        if (!_callback(msgid, payload))
                            console.error("Invalid message received: " + msgid);
                    }
                    break;
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
        appendToLog(text);
    };
}

export function websock_validate_and_send(command) {
    if (!_websock || _websock.readyState !== WebSocket.OPEN) {
        show_error("Failed to communicate with handset, websocket connection is closed!");
        return false;
    }
    _websock.send(command);
    return true;
}

export function message_send_str(type, value) {
    return websock_validate_and_send(type + "=" + value);
}

export function message_send_binary(msg_id, bytes=[]) {
    if (typeof msg_id === 'string') {
        msg_id = message_id_map[msg_id];
    }
    var sendarray = new ArrayBuffer(2 + bytes.length);
    var view = new Uint8Array(sendarray);
    view[0] = msg_id & 0xFF;
    view[1] = (msg_id >> 8) & 0xFF;
    for (var iter = 0; iter < bytes.length; iter++)
        view[2+iter] = bytes[iter];
    return websock_validate_and_send(sendarray);
}
// Publish
window.websock_validate_and_send = websock_validate_and_send;
window.message_send_str = message_send_str;
window.message_send_binary = message_send_binary;

/********************* LOGGING *************************/
function appendToLog(text) {
    const logger = $id("logField");
    const scrollsize = parseInt($id("scrollsize").value, 10) - 1;
    const log_history = logger.value.split("\n");
    const numignore = log_history.length - scrollsize;
    if (0 < numignore) log_history.splice(0, numignore);
    log_history.push(datetime_current() + ' ' + text);
    logger.value = log_history.join('\n');
    if ($id("autoscroll").checked)
        logger.scrollTop = logger.scrollHeight;
}

export function saveTextAsFile() {
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

/********************* VTX FREQS *************************/
var channelFreqTable = {
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

export function vtx_table_set(table) {
    channelFreqTable = table;
}
export function vtx_table_get() {
    return channelFreqTable;
}

export function vtx_show_freq() {
    const band = $id("vtx_band").value;
    const ch = $id("vtx_channel").value;
    if (band == "" || ch == "") {
      $id("vtx_send_btn").disabled = true;
      $id("vtx_freq").innerHTML = "";
      return;
    }
    var freq = channelFreqTable[band][parseInt(ch, 10)];
    $id("vtx_freq").innerHTML = "" + freq + " MHz";
    $id("vtx_send_btn").disabled = !(0 < freq);
}

var vtx_last_band = "-";
export function vtx_band_changed(band) {
    if (band != vtx_last_band) {
        vtx_last_band = band;
        $id("vtx_band").value = band;
        var channels = $id("vtx_channel");
        while (channels.length > 1) {
            channels.remove(channels.length - 1);
        }
        var available = [];
        channelFreqTable[band].filter(function(elem, index, array){if(0 <= elem) available.push(index);});
        for (const chval of available) {
            var option = document.createElement("option");
            option.text = chval + 1;
            option.value = chval;
            channels.add(option);
        }
        $id("vtx_channel").value = "";
    }
    vtx_show_freq();
}

export function msp_vtx_freq(freq) {
    console.log("VTX freq: %o MHz", freq);
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

export function msp_vtx_freq_send() {
    const band = $id("vtx_band").value;
    const ch = $id("vtx_channel").value;
    if (band == "" || ch == "")
        return;
    const freq = channelFreqTable[band][parseInt(ch, 10)];
    const payload = [freq & 0xff, (freq >> 8) & 0xff];
    if (freq && 0 < freq)
        message_send_binary(WSMSGID_VIDEO_FREQ, payload);
}

/********************* ESP-NOW *****************************/
export function espnowclients_send() {
    const clients = $id("espnowclients").value.split("\n");
    var bytes = [];
    for (const client of clients) {
        if (!client || client.length == 0) continue;
        const mac = client.split(":").map(function (val) { return parseInt(val, 16) });
        if (mac.length != 6) { console.error("Invalid MAC address: %s", client); return; }
        mac.forEach(element => { bytes.push(element); });
    }
    message_send_binary(WSMSGID_ESPNOW_ADDRS, bytes);
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
export function wifinetworks_add(event) {
    const ssid = $id("wifi_new_ssid").value;
    const mac = $id("wifi_new_mac").value;
    if (!ssid && !mac) {
        show_error("SSID or MAC is mandatory!");
        return;
    }
    const psk = $id("wifi_new_psk").value;
    websock_validate_and_send(
        "WIFIADD/" +
        int2str_pad(ssid.length, 2) + "/" + ssid + "/" +
        int2str_pad(psk.length, 2) + "/" + psk +
        (mac ? "/" + mac.split(":").join("") : "")
    );
}
function wifinetworks_del(event) {
    if (websock_validate_and_send("WIFIDEL/" + int2str_pad(event.target.parentElement.parentElement.esp_index, 2))) {
        event.target.disabled = true;
    }
}
/********************* Lap Timer *****************************/
function laptimernet_parse(network_str) {
    if (network_str == "") return;
    const network = {"mac": network_str.substring(0, 17), "ssid": network_str.substring(17), "pilot": ""};
    //laptimer_info_handle(network);
}
function laptimer_info_handle(laptimer) {
    $id("laptimer_ssid").value = !!laptimer.ssid ? laptimer.ssid : "";
    $id("laptimer_mac").value = !!laptimer.mac ? laptimer.mac : "";
    $id("laptimer_pilot").value = !!laptimer.pilot ? laptimer.pilot : "";
}

export function laptimer_ctrl_send(btn) {
    const start = btn.innerHTML == "START";
    message_send_binary(WSMSGID_LAPTIMER_START_STOP, [start]);
}
function laptimer_ctrl_state(race_id, state) {
    const button = $id("laptimer_ctrl_btn");
    $class_del(button, "green"); $class_del(button, "red");
    $class_add(button, state ? "red" : "green");
    button.innerHTML = state ? "STOP" : "START";
    if (state) {
        const table = $id("laptimer_laps");
        while (1 < table.rows.length) {
            table.deleteRow(table.rows.length-1);
        }
        $id("laptimer_race_id").innerHTML = race_id.toString(10);
    }
}
function laptimer_laptime_parse(payload) {
    const time = msToTime(payload.nextUint32());
    const race_id = payload.nextUint16();
    const lap_index = payload.nextUint8();
    console.log(payload, " => ", time, race_id, lap_index);

    $id("laptimer_race_id").innerHTML = race_id.toString(10);

    const table = $id("laptimer_laps");
    const newrow = table.insertRow();
    var cell = newrow.insertCell(0);
    cell.innerHTML = lap_index.toString(10);
    var cell = newrow.insertCell(1);
    cell.innerHTML = time.toString(10);
}

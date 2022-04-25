
function $id(id) {
    return document.getElementById(id);
}
function $class(id) {
    return document.getElementsByClassName(id);
}
function $name(name) {
    return document.getElementsByName(name);
}
function safelyParseJson(json) {
    // This function cannot be optimised, it's best to
    // keep it small!
    var parsed;
    try {
        parsed = JSON.parse(json);
    } catch (e) {
        parsed = JSON.parse(JSON.stringify(json));
    }
    return parsed // Could be undefined!
}

var websock;
function start() {
    var test = "";
    //test = "0:1:1,1:0:1,2:3:0,3:2:0,4:0:0,5:2:0";
    //test = "5;3;0:2:1:95,1:1:1,2:0:0,3:3:0,4:0:0,5:16:0,6:16:0,7:16:0,8:16:0,9:16:0,10:16:0,11:16:0,12:16:0,13:16:0,14:16:0,15:16:0";
    //test= "4;2;0:2:1:1,1:1:2:0,2:3:3:0,3:0:0:5,4:0:6:2,5:7:3:0,6:16:0:9,7:0:0:0,8:11:16:0,9:16:0:13,10:0:0:0,11:15:16:0,12:5:85:0,13:0:0:0,14:0:0:0,15:0:0:0";
    handset_mix_reset(test);
    //test = "900:2196:3536;194:2023:3796;183:1860:3628;490:2094:3738;"
    //handle_calibrate_adjust(test);
    //test = "0,100,100";
    //handset_battery_value(test);
    //test = "ULQ:-1,UR1:-3,UR2:-2,USN:45,PWR:1,MO:7,DLQ:1,DR1:2,DSN:3";
    telmetry_set("tlm_uldl", test);
    //test = "V:168,A:105,C:1200,R:70";
    telmetry_set("tlm_batt", test);
    //test = "lon:1239248,lat:39879284,spe:23543,hea:234,alt:234,sat:10";
    telmetry_set("tlm_gps", test);

    $id("logField").scrollTop = $id("logField").scrollHeight;
    websock = new WebSocket('ws://' + window.location.hostname + ':81/');
    websock.onopen = function (evt) { console.log('websock open'); };
    websock.onclose = function(e) {
        console.log('Socket is closed. Reconnect will be attempted in 1 second.', e.reason);
        setTimeout(function() {
        start();
        }, 1000);
    };
    websock.onerror = function (evt) { console.log(evt); };
    websock.onmessage = function (evt) {
        //console.log(evt);
        var text = evt.data;
        if (text.startsWith("ELRS_setting_")) {
            var res = text.replace("ELRS_setting_", "");
            res = res.split("=");
            setting_set(res[0], res[1]);
        } else if (text.startsWith("ELRS_tlm_")) {
            var res = text.replace("ELRS_", "");
            res = res.split("=");
            telmetry_set(res[0], res[1]);
        } else if (text.startsWith("ELRS_handset_")) {
            var res = text.replace("ELRS_", "");
            res = res.split("=");
            handset_parse(res[0], res[1]);
        } else {
            var logger = $id("logField");
            var autoscroll = $id("autoscroll").checked;
            var scrollsize = parseInt($id("scrollsize").value, 10);
            var log_history = logger.value.split("\n");
            while (scrollsize < log_history.length) {
                log_history.shift();
            }
            var date = new Date();
            var n=new Date(date.getTime() - (date.getTimezoneOffset() * 60000)).toISOString();
            log_history.push(n + ' ' + text);
            logger.value = log_history.join('\n');
            if (autoscroll)
                logger.scrollTop = logger.scrollHeight;
        }
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
        downloadLink.onclick = destroyClickedElement;
        downloadLink.style.display = "none";
        document.body.appendChild(downloadLink);
    }

    downloadLink.click();
}

function destroyClickedElement(event) {
    // remove the link from the DOM
    document.body.removeChild(event.target);
}

function setting_set(type, value) {
    var elem = $id(type);
    if (elem) {
        if (type == "vtx_freq") {
            vtx_parse_freq(value);
        } else if (type == "region_domain") {
            var rf_module = $id("rf_module");
            value = parseInt(value);

            if (!(value & 0x80)) {
                /* Disable RF selection if not dual module */
                rf_module.disabled = true;
            } else {
                rf_module.disabled = false;
            }

            if (!(value & 0x40)) {
                /* Disable tabs if not handset */
                var tabs = $name('handset');
                for (tab in tabs) {
                    tabs[tab].className += " disabled";
                }
            }
            value = value & 0x3F;

            var domain_info = "Domain ";
            if (value == 0)
                domain_info += "915MHz";
            else if (value == 1)
                domain_info += "868MHz";
            else if (value == 2)
                domain_info += "433MHz";
            else if (value == 3 || value == 4)
                domain_info += "2400MHz ISM (LoRa)";
            else if (value == 5)
                domain_info += "2400MHz ISM (FLRC)";
            else if (value == 6)
                domain_info += "2400MHz ISM (LoRa, VANILLA)";
            else
                domain_info += "UNKNOWN";
            elem.innerHTML = domain_info;

            // disable telemetry options if vanilla mode
            $id("tlm_input").disabled = (value == 6);

            // update rate options
            var rates = $id("rates_input");
            while (rates.length > 0) {
                rates.remove(rates.length-1);
            }
            var options = [];
            if (6 == value) {
                options = ['500Hz', '250Hz', '150Hz', '50Hz'];
                rf_module.selectedIndex = 4;
            } else if (5 == value) {
                options = ['1000Hz', '500Hz', '250Hz'];
                rf_module.selectedIndex = 3;
            } else if (3 <= value && value <= 4) {
                options = ['500Hz', '250Hz', '125Hz', '50Hz'];
                rf_module.selectedIndex = 2;
            } else if (value <= 2) {
                options = ['200Hz', '100Hz', '50Hz'];
                rf_module.selectedIndex = 1;
            } else {
                rf_module.selectedIndex = 0;
            }
            for (i = 0; i < options.length; i++) {
                var option = document.createElement("option");
                option.text = options[i];
                option.value = i;
                rates.add(option);
            }
        } else {
            value = value.split(",");
            if (1 < value.length) {
                var max_value = parseInt(value[1], 10);
                max_value = max_value + 2; // include reset and dummy
                var i;
                // enable all
                for (i = 0; i < elem.length; i++) {
                    elem.options[i].disabled = false;
                }
                // disable unavailable values
                for (i = (elem.length-1); max_value < i; i--) {
                    //elem.remove(i);
                    elem.options[i].disabled = true;
                }
            }
            elem.selectedIndex = [...elem.options].findIndex (option => option.value === value[0]);
        }
    }
}

function setting_send(type, elem=null)
{
    if (elem) {
        websock.send(type + "=" + elem.value);
    } else {
        websock.send(type + "?");
    }
}

// Channels with their Mhz Values
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

function vtx_show_freq()
{
    var band = $id("vtx_band").value;
    var ch = $id("vtx_channel").value;
    if (band == "" || ch == "")
        return;
    var freq = "";
    freq += channelFreqTable[band][parseInt(ch, 10)];
    freq += " MHz";
    $id("vtx_freq").innerHTML = freq;
}

function vtx_parse_freq(freq)
{
    freq = parseInt(freq, 10);

    if (freq == 0) {
        // Clear selections
        $id("vtx_band").value = "";
        $id("vtx_channel").value = "";
        $id("vtx_freq").innerHTML = "";
        return;
    }

    for (var band in channelFreqTable) {
        var channels = channelFreqTable[band];
        for (var ch in channels) {
            if (freq == channels[ch]) {
                $id("vtx_band").value = band;
                $id("vtx_channel").value = ch;
                var _freq = "";
                _freq += freq;
                _freq += " MHz";
                $id("vtx_freq").innerHTML = _freq;
                return;
            }
        }
    }
}

function setting_send_vtx()
{
    var band = $id("vtx_band").value;
    var ch = $id("vtx_channel").value;
    if (band == "" || ch == "")
        return;
    var freq = channelFreqTable[band][parseInt(ch, 10)];
    if (websock)
        websock.send("S_vtx_freq=" + freq);
}

function command_stm32(type)
{
    websock.send("stm32_cmd=" + type);
}


/********************* HANDSET *****************************/

function mixer_list_to_dict(value)
{
    var dict = {};
    var i;
    var parts = value.split(";");
    var num_switches = 12, num_aux = 12;
    if (3 == parts.length) {
        num_aux = parseInt(parts[0]);
        num_switches = parseInt(parts[1]);
        value = parts[2];
    }
    dict['aux'] = num_aux;
    dict['total'] = num_aux + 4;
    dict['switch'] = num_switches;
    for (i = 0; i < 16; i++) {
        dict[i] = {'index': '-', 'inv': false, 'scale': 1.0};
    }
    value = value.split(",");
    for (var item in value) {
        if (!value[item].length)
            continue
        var mix = value[item].split(":");
        if (3 <= mix.length) {
            var idx = parseInt(mix[0]);
            dict[idx].index = mix[1];
            dict[idx].inv = mix[2] == '1' ? true : false;
            if (4 <= mix.length) {
                var scale = parseFloat(mix[3]);
                dict[idx].scale = (scale) ? (scale / 100.) : 1.0;
            }
        }
    }
    return dict;
}

function mixer_add_selection_lst(cell, input="-", values={})
{
    var option;
    var sel = document.createElement("select");
    sel.style.width = "100px";

    /* add options */
    for (var opt in values) {
        option = document.createElement("option");
        option.value = values[opt];
        option.text = opt;
        sel.add(option, values[opt]);
    }
    /* set selected */
    var input_num = parseInt(input);
    if (input != '-' && input_num != undefined) {
        sel.selectedIndex = input_num;
    } else {
        sel.selectedIndex = -1;
    }
    cell.appendChild(sel);
}

function handset_mix_reset(value="")
{
    var iter;
    var table = $id("handset_mixer");
    /* Parse input */
    var mixes = mixer_list_to_dict(value);

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
        var row = table.insertRow();
        var cell = row.insertCell(0);
        if (iter < 4)
            cell.innerHTML = "Analog " + (iter + 1);
        else
            cell.innerHTML = "AUX " + (iter - 3);
        cell = row.insertCell(1);
        cell.style.width = "auto";
        mixer_add_selection_lst(cell, mixes[iter].index,
            ((iter < 4) ? gimbals : switches));

        cell = row.insertCell(2);
        cell.style.width = "100px";
        // creating checkbox element
        var checkbox = document.createElement('input');
        checkbox.type = "checkbox";
        checkbox.name = "invert";
        checkbox.value = iter;
        checkbox.id = "inverted" + iter;
        checkbox.checked = mixes[iter].inv;
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
            scale.value = mixes[iter].scale;
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

function mixer_send()
{
    var table = $id("handset_mixer");
    var output = "handset_mixer=";
    var index, value;
    for (index = 0; index < table.rows.length; index++) {
        var rows = table.rows[index];
        var selected = rows.cells[1].getElementsByTagName("select")[0];
        if (selected.selectedIndex > -1) {
            /* Channel index */
            output += index.toString(16);
            /* Output channel */
            selected = selected.options[selected.selectedIndex].value;
            output += selected.toString(16);
            /* Inverted */
            var _in = rows.cells[2].getElementsByTagName("input")[0];
            output += _in.checked ? '1' : '0';

            if (index < 4) {
                /* add scale */
                _in = rows.cells[3].getElementsByTagName("input")[0];
                _in = parseFloat(_in.value) * 100;
                if (_in >= 100)
                    output += "00";
                else if (_in <= 10)
                    output += "10";
                else
                    output += _in;
            }
        }
    }
    //console.log("output: %s", output);
    websock.send(output);
}

/********************* CALIBRATE *****************************/
var calibrate_btn = null;
function handset_calibrate(btn, type)
{
    if (calibrate_btn != null) {
        return;
    }
    btn.disabled = true;
    calibrate_btn = btn;
    websock.send("handset_calibrate=" + type);
}

function handset_calibrate_adjust(event)
{
    var msg = "handset_adjust_" + event.target.id + "=";
    var value = parseInt(event.target.value, 10);
    if (value < 0) {
        event.target.value = value = 0;
    } else if (value > 4095) {
        event.target.value = value = 4095;
    }
    if (value < 0x10)
        msg += '00';
    else if (value < 0x100)
        msg += '0'
    msg += value.toString(16);
    websock.send(msg);
}

function handle_calibrate_adjust(value)
{
    var iter, type, limits;
    var map = {
        0 : 'L1_', 1 : 'L2_',
        2 : 'R1_', 3 : 'R2_',
    };
    value = value.split(";");
    for (iter = 0; iter < value.length; iter++) {
        if (value[iter] == "") {
            continue;
        }
        limits = value[iter].split(":");
        type = map[iter];
        $id(type + 'min').value = limits[0];
        $id(type + 'mid').value = limits[1];
        $id(type + 'max').value = limits[2];
    }

    if (calibrate_btn != null) {
        calibrate_btn.disabled = false;
        calibrate_btn = null;
        $id("handset_calibrate_stat").innerHTML = 'Calibration ready!';
    }
}

/******************** MONITORING ****************************/
function handset_battery_value(value)
{
    var batt = $id("battery_voltage");
    value = value.split(",");
    if (0 < value.length) {
        batt.innerHTML = (parseInt(value[0], 10) / 1000.).toFixed(1) + "V";
        if (1 < value.length) {
            $id("battery_scale").value = parseInt(value[1], 10);
            if (2 < value.length) {
                $id("battery_warning").value = parseInt(value[2], 10);
            }
        }
    }
}

function handset_battery_adjust()
{
    var msg = "handset_battery_config=";
    var scale = parseInt($id("battery_scale").value, 10);
    var warn = parseInt($id("battery_warning").value, 10);

    msg += scale.toString(16);
    msg += ",";
    msg += warn.toString(16);
    websock.send(msg);
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
    val = parseInt(val, 10);
    var GPS_DEGREES_DIVIDER = 10000000;
    var result = (val / GPS_DEGREES_DIVIDER).toString();
    return result;
}

function telmetry_set(type, value)
{
    var name, updated_value, temp;
    var date = new Date();
    var now = new Date(date.getTime()).toLocaleTimeString([], {hour12: false});
    if (type.includes("_uldl")) {
        $id("tlm_ul_updated").innerHTML = now;
        $id("tlm_dl_updated").innerHTML = now;
    } else if (type.includes("_batt")) {
        $id("tlm_batt_updated").innerHTML = now;
    } else if (type.includes("_gps")) {
        $id("tlm_gps_updated").innerHTML = now;
    }

    /* Find correct collection */
    //var table = $id(type);
    value = value.split(",");
    for (var item in value) {
        var data = value[item].split(":");
        /* Search row */
        //var row = table.rows.namedItem(data[0]);
        name = data[0];
        updated_value = data[1];
        if (type.includes("_uldl")) {
            if (name.includes("R")) {
                updated_value += " dBm";
            } else if (name.includes("SN")) {
                temp = parseFloat(updated_value) / 10;
                updated_value = temp.toString() + " dB";
            }
        } else if (type.includes("_batt")) {
            if (name == "V") {
                temp = parseFloat(updated_value) / 10;
                updated_value = temp.toString() + " V";
            } else if (name == "A") {
                temp = parseFloat(updated_value) / 10;
                updated_value = temp.toString() + " A";
            } else if (name == "C") {
                temp = parseInt(updated_value);
                updated_value = temp.toString() + " mAh";
            } else if (name == "R") {
                updated_value += " %";
            }
        } else if (type.includes("_gps")) {
            if (name == "spe")
                updated_value = convert_gps_speed(updated_value);
            else if (name == "lat")
                updated_value = convert_gps_value(updated_value);
            else if (name == "lon")
                updated_value = convert_gps_value(updated_value);
            else if (name == "hea")
                updated_value += " deg";
            else if (name == "alt")
                updated_value += " m";
        }

        var row = $id(data[0]);
        if (row) {
            /* update value */
            row.cells[1].innerHTML = updated_value;
        }
    }
}

/********************* HANDSET *****************************/
function refresh_values()
{
    websock.send("handset_refresh");
}

function save_values()
{
    websock.send("handset_save");
}

function handset_parse(type, value)
{
    console.log("HANDSET: %o = (%o)", type, value);
    /* Find correct element */
    if (type.includes("_calibrate")) {
        if (calibrate_btn != null) {
            calibrate_btn.disabled = false;
            calibrate_btn = null;
            $id("handset_calibrate_stat").innerHTML = 'Calibration status: "' + value + '"';
        }
    } else if (type.includes("_adjust")) {
        handle_calibrate_adjust(value);
    } else if (type.includes("_mixer")) {
        handset_mix_reset(value);
    } else if (type.includes("_battery")) {
        handset_battery_value(value);
    }
}

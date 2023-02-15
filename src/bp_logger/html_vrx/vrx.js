const WSMSGID_ESPNOW_ADDRS = 0x1100;
const WSMSGID_VIDEO_FREQ = 0x2400;
const WSMSGID_RECORDING_CTRL = 0x2401;
function $id(id) {return document.getElementById(id);}
function $class(id) {return document.getElementsByClassName(id);}
function $name(name) {return document.getElementsByName(name);}
function $class_add(obj, type) {if(obj) obj.classList.add(type);}
function $class_del(obj, type) {if(obj) obj.classList.remove(type);}
function int2str_pad(num, size=2, base=10) {return num.toString(base).padStart(size,"0");}
var websock = null;
function start() {
  var _bands = $id("vtx_band")
  while (_bands.length > 1) { _bands.remove(_bands.length - 1); }
  for (const band in channelFreqTable) {
    var option = document.createElement("option");
    option.text = option.value = band;
    _bands.add(option);
  }
  $id("logField").scrollTop = $id("logField").scrollHeight;
  if (!window.location.hostname) { console.log("No hostname!"); return; }
  websock = new WebSocket('ws://' + window.location.hostname + '/ws');
  websock.binaryType = "arraybuffer";
  websock.onopen = function (evt) {console.log('websock open');};
  websock.onclose = function (e) {
    console.log('websock closed. Reconnect in 1 second.', e.reason);
    setTimeout(function () {start();}, 1000);
  };
  websock.onerror = function (evt) { console.log("websock error: ", evt); };
  websock.onmessage = function (evt) {
    console.log(evt);
    if (evt.data instanceof ArrayBuffer) { // handle binary message
      const message = new DataView(evt.data);
      const msgid = message.getUint16();
      const payload = new DataView(evt.data, 2);
      if (msgid == WSMSGID_ESPNOW_ADDRS) espnowclients_parse(payload);
      else if (msgid == WSMSGID_VIDEO_FREQ) msp_vtx_freq(payload.getUint16());
      else if (msgid == WSMSGID_RECORDING_CTRL) recording_control_received(payload);
      else console.error("Invalid message received: " + msgid);
      return;
    }
    const text = evt.data; // handle text message
    if (!text) return;
    if (text.startsWith("CMD_WIFINETS")) {wifinetworks_parse(text); return;}
    const logger = $id("logField");
    const scrollsize = parseInt($id("scrollsize").value, 10);
    var log_history = logger.value.split("\n");
    while (scrollsize < log_history.length) { log_history.shift(); }
    const date = new Date();
    const logtime = new Date(date.getTime() - (date.getTimezoneOffset() * 60000)).toISOString();
    log_history.push(logtime + ' ' + text);
    logger.value = log_history.join('\n');
    if ($id("autoscroll").checked) logger.scrollTop = logger.scrollHeight;
  };
}
function message_send_str(type, value) {
  if (!websock) return;
  websock.send(type + "=" + value);
}
function message_send_binary(msg_id, bytes=[]) {
  var sendarray = new ArrayBuffer(2 + bytes.length);
  var view = new Uint8Array(sendarray);
  view[0] = msg_id & 0xFF;
  view[1] = (msg_id >> 8) & 0xFF;
  for (var iter = 0; iter < bytes.length; iter++)
    view[2+iter] = bytes[iter];
  if (websock) websock.send(sendarray, { binary: true });
}
/********************* RECORDING *************************/
function recording_control_send(btn) {
  var start;
  if (btn.innerHTML == "START") {
    btn.innerHTML = "STOP";
    start = true;
  } else {
    btn.innerHTML = "START";
    start = false;
  }
  message_send_binary(WSMSGID_RECORDING_CTRL, [start]);
}
function recording_control_received(payload) {
  const state = payload.getUint8();
  const button = $id("recording_state");
  $class_del(button, "green"); $class_del(button, "red");
  $class_add(button, state ? "red" : "green");
  button.innerHTML = state ? "STOP" : "START";
}
/********************* VTX *******************************/
// HDZero supported bands and channels
const channelFreqTable = {
  "F": [  -1, 5760,   -1, 5800,   -1,   -1,   -1,   -1], // F / Airwave
  "R": [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], // R / Immersion Raceband
};
function vtx_show_freq() {
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
function vtx_band_changed(band) {
  if (band != vtx_last_band) {
    vtx_last_band = band;
    $id("vtx_band").value = band;
    var channels = $id("vtx_channel");
    while (channels.length > 1) {channels.remove(channels.length - 1);}
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
function msp_vtx_freq(freq) {
  $id("vtx_send_btn").disabled = true;
  if (freq == 0) {
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
function msp_vtx_freq_send() {
  const band = $id("vtx_band").value;
  const ch = $id("vtx_channel").value;
  if (band == "" || ch == "") return;
  const freq = channelFreqTable[band][parseInt(ch, 10)];
  const payload = [freq & 0xff, (freq >> 8) & 0xff];
  if (websock && freq && 0 < freq) message_send_binary(WSMSGID_VIDEO_FREQ, payload);
}
/********************* ESP-NOW ***************************/
function espnowclients_send() {
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
  } else target.value = "data error! " + typeof (value);
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
  const parts = value.replace("CMD_WIFINETS/\\/\\", "").split('/\\/\\');
  for (const network_str of parts) {
    networks.push({
      "idx": network_str.substring(0, 2),
      "mac": network_str.substring(2, 19),
      "ssid": network_str.substring(19),
    });
  }
  /* clean table */
  while (table.rows.length) {
    table.deleteRow(table.rows.length-1);
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
  var command = "WIFIADD/";
  const ssid = $id("wifi_new_ssid").value;
  command += int2str_pad(ssid.length, 2) + "/" + ssid + "/";
  const psk = $id("wifi_new_psk").value;
  command += int2str_pad(psk.length, 2) + "/" + psk;
  const mac = $id("wifi_new_mac").value;
  if (!ssid && !mac && !psk) return;
  if (!ssid && !mac) {console.error("SSID or MAC is mandatory!");}
  if (mac) {command += "/" + mac.split(":").join("");}
  if (websock) websock.send(command);
}
function wifinetworks_del(event) {
  var row = event.target.parentElement.parentElement;
  var command = "WIFIDEL/" + int2str_pad(row.esp_index, 2);
  if (websock) websock.send(command);
  event.target.disabled = true;
}
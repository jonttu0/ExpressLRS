#pragma once
#include <pgmspace.h>

static const char PROGMEM HDZ_INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en"><head>
  <meta charset="utf-8">
  <meta name="viewport" content="width = device-width, initial-scale = 1.0">
  <title>HDZero Backpack</title>
  <style>
body {
  width: 100%; height: 100%; margin: 0px; padding: 0px;
  background-color: #1E1E1E; Color: #69cbf7;
  font-family: Arial, Helvetica, Sans-Serif;
}
.content {max-width: 700px; margin: auto; text-align: center;}
.hide {display: none;}
.center {margin-left: auto; margin-right: auto;}
.valigntop {display: inline-block; vertical-align: top;}
#logField {
  font-size: 12px;
  background-color: #252525; Color: #C5C5C5;
  border-radius: 5px; border: none;
  width: 100%; min-width: 650px;
  height: auto; min-height: 621px;
  margin: 0px; overflow: auto;
}
#espnowclients {
  font-size: 12px;
  background-color: #252525; Color: #C5C5C5;
  border-radius: 5px; border: none;
  width: 100%; min-width: 160px; max-width: 160px;
  height: auto; min-height: 50px;
  margin: 0px; overflow: auto; resize: none;
}
#validationMessage {color: red;}
table {
  width: 500px; max-width: 100%;
  table-layout: auto;
  text-align: left;
}
td {padding: 1px 1px 1px 1px;}
fieldset {
  margin: 10px;
  border: 1px solid #666; border-radius: 8px;
  display: block;
}
legend {
  text-align: left;
  color: #6692a8;
  font-size: 18px; font-weight: bold;
  padding: 2px 4px;
}
  </style>
  <script>
const WSMSGID_ESPNOW_ADDRS = 0x1100;
const WSMSGID_VIDEO_FREQ = 0x2400;
const WSMSGID_RECORDING_CTRL = 0x2401;
function $id(id) {return document.getElementById(id);}
function $class(id) {return document.getElementsByClassName(id);}
function $name(name) {return document.getElementsByName(name);}
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
  websock = new WebSocket('ws://' + window.location.hostname + ':81/');
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
  console.log(btn.innerHTML);
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
  $id("recording_state").innerHTML = payload.getUint8() ? "STOP" : "START";
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
  </script>
</head>
<body onload="javascript:start();"><div class="content">
  <fieldset><legend>Log Messages</legend>
    <textarea id="logField" readonly></textarea><br>
    <input type="checkbox" id="autoscroll" checked><label for="autoscroll"> Auto scroll</label> |
    <input type='number' value='512' name='scrollsize' id='scrollsize' min="256" style="width: 50px;">
    <label for="scrollsize"> Scroll len</label>
  </fieldset>
  <fieldset><legend>Settings</legend>
    <table class="center">
      <tr>
        <td width="200">
          <label for="btx_band">Band:</label>
          <select id="vtx_band" name="btx_band" onchange="vtx_band_changed(this.value)">
            <option value="" selected disabled hidden>Select band</option>
          </select>
        </td>
        <td width="130">
          <label for="btx_ch">Channel:</label>
          <select id="vtx_channel" name="btx_ch" onchange="vtx_show_freq()">
            <option value="" selected disabled hidden>Select channel</option>
          </select>
        </td>
        <td width="120">
          <div id="vtx_freq" style="width: 100px;"></div>
        </td>
        <td><button onclick="msp_vtx_freq_send()" id="vtx_send_btn" disabled>SET</button></td>
      </tr>
    </table>
  </fieldset>
  <fieldset><legend>Recording</legend>
    <button id="recording_state" onclick="recording_control_send(this)">START</button>
  </fieldset>
  <fieldset><legend>Debug</legend>
    <label for="osdtext">OSD text[32]:</label>
    <input type="text" id="osd_text" name="osdtext" onchange="message_send_str('SET_text', this.value)" maxlength="32"
      style="width: 320px;"><br>
  </fieldset>
  <fieldset><legend>Firmware Upgrade</legend>
    <div><form method='POST' action='/update' enctype='multipart/form-data'>
        <input type='file' accept='.bin,.bin.gz' name='backpack_fw' id="esp_fw" style="width:350px;">
        <input type='submit' value='FLASH' id='esp_submit' disabled='disabled'>
    </form></div>
    <p><span id="validationMessage" class="hide">Please, select correct firmware file!</span></p>
  </fieldset>
  <fieldset><legend>ESP-NOW Clients</legend><div>
    <label for="espnowclients" class="valigntop">Add one per line</label>
    <textarea id="espnowclients" class="valigntop" onkeyup="espnowclients_autosize(this)"></textarea>
    <button onclick="espnowclients_send()" id="espnowclients_btn" class="valigntop">SAVE</button>
  </div></fieldset>
  <script>
    const message = $id('validationMessage');
    $id('esp_fw').onchange = function (ev) {
      const FIRMWARE_PATTERN = /backpack\.(bin|bin.gz)$/g;
      const uploadButton = $id('esp_submit');
      if (FIRMWARE_PATTERN.test(ev.target.value)) {
        uploadButton.removeAttribute('disabled');
        message.classList.add('hide');
      } else {
        uploadButton.setAttribute('disabled', 'disabled');
        message.classList.remove('hide');
      }
    };
  </script>
</div></body></html>
)rawliteral";

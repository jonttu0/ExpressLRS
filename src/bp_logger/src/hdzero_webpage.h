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
function $id(id) { return document.getElementById(id); }
function $class(id) { return document.getElementsByClassName(id); }
function $name(name) { return document.getElementsByName(name); }
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
      if (msgid == 0x1100) espnowclients_parse(payload);
      return;
    }
    var text = evt.data; // handle text message
    if (text.startsWith("HDZ_CRTL_")) {
      var res = text.replace("HDZ_CRTL_", "");
      res = res.split("=");
      setting_set(res[0], res[1]);
    } else {
      var logger = $id("logField");
      const autoscroll = $id("autoscroll").checked;
      const scrollsize = parseInt($id("scrollsize").value, 10);
      var log_history = logger.value.split("\n");
      while (scrollsize < log_history.length) { log_history.shift(); }
      const date = new Date();
      const logtime = new Date(date.getTime() - (date.getTimezoneOffset() * 60000)).toISOString();
      log_history.push(logtime + ' ' + text);
      logger.value = log_history.join('\n');
      if (autoscroll) logger.scrollTop = logger.scrollHeight;
    }
  };
}

function setting_set(type, value) {
  if (type == "vtx_freq") {
    vtx_parse_freq(value);
  } else if (type == "espnowclients") {
    espnowclients_parse(value);
  } else { console.log("Unknown command: %s = %s", type, value); }
}

function setting_send(type, elem = null) {
  if (!websock) return;
  if (elem) websock.send(type + "=" + elem.value);
  else websock.send(type + "?");
}

// Channels with their MHz Values
const channelFreqTable = {
  "F": [  -1, 5760,   -1, 5800,   -1,   -1,   -1,   -1], // F / Airwave
  "R": [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], // R / Immersion Raceband
};
var vtx_last_band = "-";
function vtx_band_changed(band) {
  if (band != vtx_last_band) {
    var channels = $id("vtx_channel");
    while (channels.length > 1) {channels.remove(channels.length - 1);}
    var options = [];
    if (band == "F") options = [2, 4];
    else if (band == "R") options = [1, 2, 3, 4, 5, 6, 7, 8];
    for (i = 0; i < options.length; i++) {
      var option = document.createElement("option");
      option.text = options[i];
      option.value = options[i] - 1;
      channels.add(option);
    }
    $id("vtx_channel").value = "";
  }
  vtx_show_freq();
}
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
function vtx_parse_freq(freq) {
  freq = parseInt(freq, 10);
  if (0 < freq) {
    for (const band in channelFreqTable) {
      const channels = channelFreqTable[band];
      for (const ch in channels) {
        if (freq == channels[ch]) {
          $id("vtx_band").value = band;
          vtx_band_changed(band);
          $id("vtx_channel").value = ch;
          var _freq = "";
          _freq += freq;
          _freq += " MHz";
          $id("vtx_freq").innerHTML = _freq;
          $id("vtx_send_btn").disabled = false;
          return;
        }
      }
    }
  }
  // Clear selections
  $id("vtx_band").value = "";
  $id("vtx_channel").value = "";
  $id("vtx_freq").innerHTML = "";
  $id("vtx_send_btn").disabled = true;
}
function setting_send_vtx() {
  var band = $id("vtx_band").value;
  var ch = $id("vtx_channel").value;
  if (band == "" || ch == "") return;
  var freq = channelFreqTable[band][parseInt(ch, 10)];
  if (websock && freq && 0 < freq) websock.send("SET_vtx_freq=" + freq);
}
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
  view[0] = 0x00; // Little endian 0x1100
  view[1] = 0x11;
  for (var i = 0; i < bytes.length; i++) view[2+i] = bytes[i];
  if (websock) websock.send(sendarray, { binary: true });
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
            <option value="" selected disabled hidden></option>
          </select>
        </td>
        <td width="120">
          <div id="vtx_freq" style="width: 100px;"></div>
        </td>
        <td><button onclick="setting_send_vtx()" id="vtx_send_btn" disabled>SET</button></td>
      </tr>
    </table>
  </fieldset>
  <fieldset><legend>Debug</legend>
    <label for="osdtext">OSD text[32]:</label>
    <input type="text" id="osd_text" name="osdtext" onchange="setting_send('SET_text', this)" maxlength="32"
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

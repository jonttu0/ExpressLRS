import * as common from "./common.js";
export let cmn = common;

// HDZero supported bands and channels
const channelFreqTable = {
  "F": [  -1, 5760,   -1, 5800,   -1,   -1,   -1,   -1], // F / Airwave
  "R": [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917], // R / Immersion Raceband
};
common.vtx_table_set(channelFreqTable);

export function start() {
  const fea_debug = {"recording":1, "osd_text":1, "laptimer":1, "espnow":1};
  feature_config(fea_debug);

  common.common_init();

  // configure events
  const events = {
    "fea_config": function(e) {
      const config = JSON.parse(e.data);
      feature_config(config);
      common.msp_vtx_freq(config.vtxfreq);},
    "vrx_version": function(e) {
      $id("firmware_version_vrx").innerHTML = e.data;},
    "vtxfreq": function(e) {
      const config = JSON.parse(e.data);
      common.msp_vtx_freq(config.vtxfreq);},
  };
  common.events_init(events);

  common.websocket_init(handle_input_msg);
}

function handle_input_msg(msgid, payload) {
  switch (msgid) {
    case common.WSMSGID_RECORDING_CTRL:
      recording_control_received(!!payload.getUint8());
      break;
    default:
        return false;
  }
  return true;
}

/********************* UI CONFIG *************************/
function feature_config(config) {
  $id("recording_control").style.display = !!config.recording ? "block" : "none";
  $id("osd_text_control").style.display = !!config.osd_text ? "block" : "none";
  $id("laptimer_control").style.display = !!config.laptimer ? "block" : "none";
  $id("espnow_control").style.display = !!config.espnow ? "block" : "none";
  $id("vrxversion_control").style.display = !!config.vrxversion ? "block" : "none";
}

/********************* RECORDING *************************/
export function recording_control_send(btn) {
  var start;
  if (btn.innerHTML == "START") {
    btn.innerHTML = "STOP";
    start = true;
  } else {
    btn.innerHTML = "START";
    start = false;
  }
  common.message_send_binary(common.WSMSGID_RECORDING_CTRL, [start]);
}

function recording_control_received(state) {
  const button = $id("recording_state");
  $class_del(button, "green"); $class_del(button, "red");
  $class_add(button, state ? "red" : "green");
  button.innerHTML = state ? "STOP" : "START";
}

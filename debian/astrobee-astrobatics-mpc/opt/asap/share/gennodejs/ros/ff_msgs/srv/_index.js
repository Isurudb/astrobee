
"use strict";

let GetPipelines = require('./GetPipelines.js')
let SetInertia = require('./SetInertia.js')
let GetMap = require('./GetMap.js')
let SetRate = require('./SetRate.js')
let SetDataToDisk = require('./SetDataToDisk.js')
let ConfigureCamera = require('./ConfigureCamera.js')
let GetOccupancyMap = require('./GetOccupancyMap.js')
let SetEkfInput = require('./SetEkfInput.js')
let RegisterPlanner = require('./RegisterPlanner.js')
let SetBool = require('./SetBool.js')
let SetStreamingLights = require('./SetStreamingLights.js')
let EnableCamera = require('./EnableCamera.js')
let Trigger = require('./Trigger.js')
let VisualeyezConfig = require('./VisualeyezConfig.js')
let EnableRecording = require('./EnableRecording.js')
let GetZones = require('./GetZones.js')
let SetFloat = require('./SetFloat.js')
let ResetMap = require('./ResetMap.js')
let UnloadLoadNodelet = require('./UnloadLoadNodelet.js')
let SetState = require('./SetState.js')
let GetFloat = require('./GetFloat.js')
let SetZones = require('./SetZones.js')

module.exports = {
  GetPipelines: GetPipelines,
  SetInertia: SetInertia,
  GetMap: GetMap,
  SetRate: SetRate,
  SetDataToDisk: SetDataToDisk,
  ConfigureCamera: ConfigureCamera,
  GetOccupancyMap: GetOccupancyMap,
  SetEkfInput: SetEkfInput,
  RegisterPlanner: RegisterPlanner,
  SetBool: SetBool,
  SetStreamingLights: SetStreamingLights,
  EnableCamera: EnableCamera,
  Trigger: Trigger,
  VisualeyezConfig: VisualeyezConfig,
  EnableRecording: EnableRecording,
  GetZones: GetZones,
  SetFloat: SetFloat,
  ResetMap: ResetMap,
  UnloadLoadNodelet: UnloadLoadNodelet,
  SetState: SetState,
  GetFloat: GetFloat,
  SetZones: SetZones,
};

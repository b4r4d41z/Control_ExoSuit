
"use strict";

let srvChangeJoller = require('./srvChangeJoller.js')
let srvManiInst = require('./srvManiInst.js')
let srvchangeCtlMode = require('./srvchangeCtlMode.js')
let srvClearPositionCMD = require('./srvClearPositionCMD.js')
let changeAMBACCtrlMode = require('./changeAMBACCtrlMode.js')
let changeArmCtrlMode = require('./changeArmCtrlMode.js')
let controlEndHand = require('./controlEndHand.js')
let srvChangePhases = require('./srvChangePhases.js')

module.exports = {
  srvChangeJoller: srvChangeJoller,
  srvManiInst: srvManiInst,
  srvchangeCtlMode: srvchangeCtlMode,
  srvClearPositionCMD: srvClearPositionCMD,
  changeAMBACCtrlMode: changeAMBACCtrlMode,
  changeArmCtrlMode: changeArmCtrlMode,
  controlEndHand: controlEndHand,
  srvChangePhases: srvChangePhases,
};

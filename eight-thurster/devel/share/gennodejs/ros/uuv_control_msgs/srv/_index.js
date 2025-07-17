
"use strict";

let InitWaypointSet = require('./InitWaypointSet.js')
let Hold = require('./Hold.js')
let GoTo = require('./GoTo.js')
let SwitchToAutomatic = require('./SwitchToAutomatic.js')
let InitCircularTrajectory = require('./InitCircularTrajectory.js')
let AddWaypoint = require('./AddWaypoint.js')
let GetPIDParams = require('./GetPIDParams.js')
let StartTrajectory = require('./StartTrajectory.js')
let InitHelicalTrajectory = require('./InitHelicalTrajectory.js')
let SetMBSMControllerParams = require('./SetMBSMControllerParams.js')
let GetMBSMControllerParams = require('./GetMBSMControllerParams.js')
let ResetController = require('./ResetController.js')
let SetPIDParams = require('./SetPIDParams.js')
let GoToIncremental = require('./GoToIncremental.js')
let SetSMControllerParams = require('./SetSMControllerParams.js')
let InitRectTrajectory = require('./InitRectTrajectory.js')
let SwitchToManual = require('./SwitchToManual.js')
let GetWaypoints = require('./GetWaypoints.js')
let ClearWaypoints = require('./ClearWaypoints.js')
let InitWaypointsFromFile = require('./InitWaypointsFromFile.js')
let GetSMControllerParams = require('./GetSMControllerParams.js')
let IsRunningTrajectory = require('./IsRunningTrajectory.js')

module.exports = {
  InitWaypointSet: InitWaypointSet,
  Hold: Hold,
  GoTo: GoTo,
  SwitchToAutomatic: SwitchToAutomatic,
  InitCircularTrajectory: InitCircularTrajectory,
  AddWaypoint: AddWaypoint,
  GetPIDParams: GetPIDParams,
  StartTrajectory: StartTrajectory,
  InitHelicalTrajectory: InitHelicalTrajectory,
  SetMBSMControllerParams: SetMBSMControllerParams,
  GetMBSMControllerParams: GetMBSMControllerParams,
  ResetController: ResetController,
  SetPIDParams: SetPIDParams,
  GoToIncremental: GoToIncremental,
  SetSMControllerParams: SetSMControllerParams,
  InitRectTrajectory: InitRectTrajectory,
  SwitchToManual: SwitchToManual,
  GetWaypoints: GetWaypoints,
  ClearWaypoints: ClearWaypoints,
  InitWaypointsFromFile: InitWaypointsFromFile,
  GetSMControllerParams: GetSMControllerParams,
  IsRunningTrajectory: IsRunningTrajectory,
};

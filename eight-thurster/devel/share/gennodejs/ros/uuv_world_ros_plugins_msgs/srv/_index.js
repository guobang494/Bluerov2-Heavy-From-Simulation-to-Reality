
"use strict";

let SetCurrentVelocity = require('./SetCurrentVelocity.js')
let TransformFromSphericalCoord = require('./TransformFromSphericalCoord.js')
let GetCurrentModel = require('./GetCurrentModel.js')
let SetOriginSphericalCoord = require('./SetOriginSphericalCoord.js')
let SetCurrentModel = require('./SetCurrentModel.js')
let SetCurrentDirection = require('./SetCurrentDirection.js')
let TransformToSphericalCoord = require('./TransformToSphericalCoord.js')
let GetOriginSphericalCoord = require('./GetOriginSphericalCoord.js')

module.exports = {
  SetCurrentVelocity: SetCurrentVelocity,
  TransformFromSphericalCoord: TransformFromSphericalCoord,
  GetCurrentModel: GetCurrentModel,
  SetOriginSphericalCoord: SetOriginSphericalCoord,
  SetCurrentModel: SetCurrentModel,
  SetCurrentDirection: SetCurrentDirection,
  TransformToSphericalCoord: TransformToSphericalCoord,
  GetOriginSphericalCoord: GetOriginSphericalCoord,
};

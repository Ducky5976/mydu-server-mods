# API available in Javascript for mods

## CPPMod.sendModAction(modName, actionId, [constructId, elementId, playerId], payload)

Send a modAction to the given loaded DLL mod

## CPPMod.raycast()

Raycast in front of the player view.

Returns a serialized JSON object string with the following fields:

playerId: hit playerId or 0

constructId: hit construct or 0

elementId: hit element or 0

impactPoint: [x,y,z] world coordinates of impact point

impactNormal: [x,y,z] normal vector of the impact surface

## CPPMOd.anyRay([startx, starty, startz], [endx, endy, endz], mask)

Do a raycast from start to end in world coordinates, using mask as an object kind filter (use 255 for all).

Returns the same JSON serialized string format as `raycast()`.

## CPPMod.playerInfo()

Returns various infos about current player. Serialized JSON with fields:

transform: 4x3 local transform matrix

worldTransform: 4x3 world transform matrix

parentId: construct player is parented to

velocity: 3d vector of current local velocity

centerOfMass: 3d world vector of center of mass coordinates

jetpack: is jetpack on

sprint: is sprinting

grounded: is grounded

torch: is flashlight on

gameTime: total game time in seconds

currentTool: current equiped tool Item Hieararchy name

playerId: current player id


## CPPMod.avatarInfo(playerId)

Returns info on given avatar, if loaded:

playerId: the player Id.

parentId: construct player is parented to

localPosition: 3d vector

localRotation: quaternion

worldPosition: 3d vector

worldRotation: quaternion

## CPPMod.soundElementPostEvent(constructId, elementId, eventId)

## CPPMod.soundElementSetValue(constructId, elementId, eventId, floatValue)

## CPPMod.soundElementSetSwitch(constructId, elementId, eventId, switchId)

## CPPMod.soundAvatarPostEvent(playerId, eventId)

## CPPMod.soundAvatarSetValue(playerId, eventId, floatValue)

## CPPMod.soundAvatarSetSwitch(playerId, eventId, switchId)

## CPPMod.luaElementSetSignalIn(constructId, elementId, plugName, doubleValue)

## CPPMod.luaElementGetSignalIn(constructId, elementId, plugName)

## CPPMod.luaElementGetSignalOut(constructId, elementId, plugName)

## CPPMod.luaElementEmitEvent(constructId, elementId, eventName, [argsDouble...])

## CPPMod.animationElementPlay(constructId, elementId, animName)

## CPPMod.animationElementReset(constructId, elementId, timeRatio)

## CPPMod.animationElementDuration(constructId, elementId) -> float

## CPPMod.animationElementTimeRatio(constructId, elementId) -> float

## CPPMod.animationElementPause(constructId, elementId)

## CPPMod.animationElementList(constructId, elementId) -> [string]

## CPPMod.elementGetTag(constructId, elementId) -> gamescriptTag

## CPPMod.pkfxElementStart(constructId, elementId, fxName, prewarmTimeFloat)

## CPPMod.pkfxElementStop(constructId, elementId, fxName)

## CPPMod.pkfxElementKill(constructId, elementId, fxName)

## CPPMod.pkfxElementAttributesList(constructId, elementId, fxName) -> [attributeName]

## CPPMod.pkfxSetSerializedAttribute(constructId, elementId, fxName, attrName, valueAsString)



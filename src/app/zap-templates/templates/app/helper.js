/*
 *
 *    Copyright (c) 2020 Project CHIP Authors
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

// Import helpers from zap core
const zapPath      = '../../../../../third_party/zap/repo/dist/src-electron/';
const templateUtil = require(zapPath + 'generator/template-util.js')
const zclHelper    = require(zapPath + 'generator/helper-zcl.js')
const queryCommand = require(zapPath + 'db/query-command.js')
const zclQuery     = require(zapPath + 'db/query-zcl.js')
const cHelper      = require(zapPath + 'generator/helper-c.js')
const string       = require(zapPath + 'util/string.js')

const StringHelper    = require('../../common/StringHelper.js');
const ChipTypesHelper = require('../../common/ChipTypesHelper.js');

// This list of attributes is taken from section '11.2. Global Attributes' of the
// Data Model specification.
const kGlobalAttributes = [
  0xfffb, // AttributeList
  0xfffc, // ClusterRevision
  0xfffd, // FeatureMap
];

// TODO Expose the readType as an additional member field of {{asUnderlyingZclType}} instead
//      of having to call this method separately.
function asReadType(type)
{
  if (StringHelper.isShortString(type)) {
    return 'String';
  }

  if (StringHelper.isLongString(type)) {
    return 'LongString';
  }

  function fn(pkgId)
  {
    const options = { 'hash' : {} };
    return zclHelper.asUnderlyingZclType.call(this, type, options).then(zclType => {
      const basicType = ChipTypesHelper.asBasicType(zclType);
      switch (basicType) {
      case 'bool':
        return 'Int8u';
      case 'int8_t':
        return 'Int8s';
      case 'uint8_t':
        return 'Int8u';
      case 'int16_t':
        return 'Int16s';
      case 'uint16_t':
        return 'Int16u';
      case 'int24_t':
        return 'Int24s';
      case 'uint24_t':
        return 'Int24u';
      case 'int32_t':
        return 'Int32s';
      case 'uint32_t':
        return 'Int32u';
      case 'int64_t':
        return 'Int64s';
      case 'uint64_t':
        return 'Int64u';
      default:
        error = 'Unhandled underlying type ' + zclType + ' for original type ' + type;
        throw error;
      }
    })
  }

  const promise = templateUtil.ensureZclPackageId(this).then(fn.bind(this)).catch(err => {
    console.log(err);
    throw err;
  });
  return templateUtil.templatePromise(this.global, promise)
}

//  Endpoint-config specific helpers
// these helpers are a Hot fix for the "GENERATED_FUNCTIONS" problem
// They should be removed or replace once issue #4369 is resolved
// These helpers only works within the endpoint_config iterator

// List of all cluster with generated functions
var endpointClusterWithInit = [
  'Basic',
  'Identify',
  'Groups',
  'Scenes',
  'Occupancy Sensing',
  'On/Off',
  'Level Control',
  'Color Control',
  'IAS Zone',
  'Pump Configuration and Control',
];
var endpointClusterWithAttributeChanged = [ 'Identify', 'Door Lock', 'Pump Configuration and Control' ];
var endpointClusterWithPreAttribute     = [ 'IAS Zone', 'Thermostat User Interface Configuration' ];
var endpointClusterWithMessageSent      = [ 'IAS Zone' ];

/**
 * Populate the GENERATED_FUNCTIONS field
 */
function chip_endpoint_generated_functions()
{
  let alreadySetCluster = [];
  let ret               = '\\\n';
  this.clusterList.forEach((c) => {
    let clusterName  = c.clusterName;
    let functionList = '';
    if (alreadySetCluster.includes(clusterName)) {
      // Only one array of Generated functions per cluster across all endpoints
      return
    }
    if (c.comment.includes('server')) {
      let hasFunctionArray = false
      if (endpointClusterWithInit.includes(clusterName))
      {
        hasFunctionArray = true
        functionList     = functionList.concat(
            `  (EmberAfGenericClusterFunction) emberAf${cHelper.asCamelCased(clusterName, false)}ClusterServerInitCallback,\\\n`)
      }

      if (endpointClusterWithAttributeChanged.includes(clusterName)) {
        functionList     = functionList.concat(`  (EmberAfGenericClusterFunction) Matter${
            cHelper.asCamelCased(clusterName, false)}ClusterServerAttributeChangedCallback,\\\n`)
        hasFunctionArray = true
      }

      if (endpointClusterWithMessageSent.includes(clusterName)) {
        functionList     = functionList.concat(`  (EmberAfGenericClusterFunction) emberAf${
            cHelper.asCamelCased(clusterName, false)}ClusterServerMessageSentCallback,\\\n`)
        hasFunctionArray = true
      }

      if (endpointClusterWithPreAttribute.includes(clusterName)) {
        functionList     = functionList.concat(`  (EmberAfGenericClusterFunction) Matter${
            cHelper.asCamelCased(clusterName, false)}ClusterServerPreAttributeChangedCallback,\\\n`)
        hasFunctionArray = true
      }

      if (hasFunctionArray) {
        ret = ret.concat(
            `const EmberAfGenericClusterFunction chipFuncArray${cHelper.asCamelCased(clusterName, false)}Server[] = {\\\n`)
        ret = ret.concat(functionList)
        ret = ret.concat(`};\\\n`)
        alreadySetCluster.push(clusterName)
      }
    }
  })
  return ret.concat('\n');
}

/**
 * Return endpoint config GENERATED_CLUSTER MACRO
 * To be used as a replacement of endpoint_cluster_list since this one
 * includes the GENERATED_FUNCTIONS array
 */
function chip_endpoint_cluster_list()
{
  let ret = '{ \\\n';
  this.clusterList.forEach((c) => {
    let mask          = '';
    let functionArray = c.functions;
    let clusterName   = c.clusterName;

    if (c.comment.includes('server')) {
      let hasFunctionArray = false;
      if (endpointClusterWithInit.includes(clusterName)) {
        c.mask.push('INIT_FUNCTION')
        hasFunctionArray = true
      }

      if (endpointClusterWithAttributeChanged.includes(clusterName)) {
        c.mask.push('ATTRIBUTE_CHANGED_FUNCTION')
        hasFunctionArray = true
      }

      if (endpointClusterWithPreAttribute.includes(clusterName)) {
        c.mask.push('PRE_ATTRIBUTE_CHANGED_FUNCTION')
        hasFunctionArray = true
      }

      if (endpointClusterWithMessageSent.includes(clusterName)) {
        c.mask.push('MESSAGE_SENT_FUNCTION')
        hasFunctionArray = true
      }

      if (hasFunctionArray) {
        functionArray = 'chipFuncArray' + cHelper.asCamelCased(clusterName, false) + 'Server'
      }
    }

    if (c.mask.length == 0) {
      mask = '0'
    } else {
      mask = c.mask.map((m) => `ZAP_CLUSTER_MASK(${m.toUpperCase()})`).join(' | ')
    }
    ret = ret.concat(`  { ${c.clusterId}, ZAP_ATTRIBUTE_INDEX(${c.attributeIndex}), ${c.attributeCount}, ${c.attributeSize}, ${
        mask}, ${functionArray} }, /* ${c.comment} */ \\\n`)
  })
  return ret.concat('}\n');
}

//  End of Endpoint-config specific helpers

function asPrintFormat(type)
{
  if (StringHelper.isString(type)) {
    return '%s';
  }

  function fn(pkgId)
  {
    const options = { 'hash' : {} };
    return zclHelper.asUnderlyingZclType.call(this, type, options).then(zclType => {
      const basicType = ChipTypesHelper.asBasicType(zclType);
      switch (basicType) {
      case 'bool':
        return '%d';
      case 'int8_t':
        return '%" PRId8 "';
      case 'uint8_t':
        return '%" PRIu8 "';
      case 'int16_t':
        return '%" PRId16 "';
      case 'uint16_t':
        return '%" PRIu16 "';
      case 'int24_t':
        return '%" PRId32 "';
      case 'uint24_t':
        return '%" PRIu32 "';
      case 'int32_t':
        return '%" PRId32 "';
      case 'uint32_t':
        return '%" PRIu32 "';
      case 'int64_t':
        return '%" PRId64 "';
      case 'uint64_t':
        return '%" PRIu64 "';
      default:
        return '%p';
      }
    })
  }

  const promise = templateUtil.ensureZclPackageId(this).then(fn.bind(this)).catch(err => {
    console.log(err);
    throw err;
  });
  return templateUtil.templatePromise(this.global, promise)
}

function asTypedLiteral(value, type)
{
  const valueIsANumber = !isNaN(value);
  function fn(pkgId)
  {
    const options = { 'hash' : {} };
    return zclHelper.asUnderlyingZclType.call(this, type, options).then(zclType => {
      const basicType = ChipTypesHelper.asBasicType(zclType);
      switch (basicType) {
      case 'int32_t':
        return value + (valueIsANumber ? 'L' : '');
      case 'int64_t':
        return value + (valueIsANumber ? 'LL' : '');
      case 'uint16_t':
        return value + (valueIsANumber ? 'U' : '');
      case 'uint32_t':
        return value + (valueIsANumber ? 'UL' : '');
      case 'uint64_t':
        return value + (valueIsANumber ? 'ULL' : '');
      case 'float':
        if (!valueIsANumber) {
          return value;
        }
        if (value == Infinity || value == -Infinity) {
          return `${(value < 0) ? '-' : ''}INFINITY`
        }
        // If the number looks like an integer, append ".0" to the end;
        // otherwise adding an "f" suffix makes compilers complain.
        value = value.toString();
        if (value.match(/^[0-9]+$/)) {
          value = value + ".0";
        }
        return value + 'f';
      default:
        if (!valueIsANumber) {
          return value;
        }
        if (value == Infinity || value == -Infinity) {
          return `${(value < 0) ? '-' : ''}INFINITY`
        }
        return value;
      }
    })
  }

  const promise = templateUtil.ensureZclPackageId(this).then(fn.bind(this)).catch(err => {
    console.log(err);
    throw err;
  });
  return templateUtil.templatePromise(this.global, promise)
}

function hasSpecificAttributes(options)
{
  return this.count > kGlobalAttributes.length;
}

function asLowerCamelCase(label)
{
  let str = string.toCamelCase(label, true);
  // Check for the case when were:
  // 1. A single word (that's the regexp at the beginning, which matches the
  //    word-splitting regexp in string.toCamelCase).
  // 2. Starting with multiple capital letters in a row.
  // 3. But not _all_ capital letters (which we purposefully
  //    convert to all-lowercase).
  //
  // and if all those conditions hold, preserve the leading capital letters by
  // uppercasing the first one, which got lowercased.
  if (!/ |_|-|\//.test(label) && label.length > 1 && label.substring(0, 2).toUpperCase() == label.substring(0, 2)
      && label.toUpperCase() != label) {
    str = str[0].toUpperCase() + str.substring(1);
  }
  return str.replace(/[\.:]/g, '');
}

function asUpperCamelCase(label)
{
  let str = string.toCamelCase(label, false);
  return str.replace(/[\.:]/g, '');
}

function asMEI(prefix, suffix)
{
  return cHelper.asHex((prefix << 16) + suffix, 8);
}

/*
 * @brief
 *
 * This function converts a given ZAP type to a Cluster Object
 * type used by the Matter SDK.
 *
 * Args:
 *
 * type:            ZAP type specified in the XML
 * isDecodable:     Whether to emit an Encodable or Decodable cluster
 *                  object type.
 *
 * These types can be found in src/app/data-model/.
 *
 */
async function zapTypeToClusterObjectType(type, isDecodable, options)
{
  let passByReference = false;
  async function fn(pkgId)
  {
    const ns          = options.hash.ns ? ('chip::app::Clusters::' + asUpperCamelCase(options.hash.ns) + '::') : '';
    const typeChecker = async (method) => zclHelper[method](this.global.db, type, pkgId).then(zclType => zclType != 'unknown');

    if (await typeChecker('isEnum')) {
      return ns + type;
    }

    if (await typeChecker('isBitmap')) {
      return 'chip::BitFlags<' + ns + type + '>';
    }

    if (await typeChecker('isStruct')) {
      passByReference = true;
      return ns + 'Structs::' + type + '::' + (isDecodable ? 'DecodableType' : 'Type');
    }

    return zclHelper.asUnderlyingZclType.call({ global : this.global }, type, options);
  }

  let typeStr = await templateUtil.ensureZclPackageId(this).then(fn.bind(this));
  if ((this.isList || this.isArray || this.entryType) && !options.hash.forceNotList) {
    passByReference = true;
    // If we did not have a namespace provided, we can assume we're inside
    // chip::app.
    let listNamespace = options.hash.ns ? "chip::app::" : ""
    if (isDecodable)
    {
      typeStr = `${listNamespace}DataModel::DecodableList<${typeStr}>`;
    }
    else
    {
      // Use const ${typeStr} so that consumers don't have to create non-const
      // data to encode.
      typeStr = `${listNamespace}DataModel::List<const ${typeStr}>`;
    }
  }
  if (this.isNullable && !options.hash.forceNotNullable) {
    passByReference = true;
    // If we did not have a namespace provided, we can assume we're inside
    // chip::app::.
    let ns  = options.hash.ns ? "chip::app::" : ""
    typeStr = `${ns}DataModel::Nullable<${typeStr}>`;
  }
  if (this.isOptional && !options.hash.forceNotOptional) {
    passByReference = true;
    // If we did not have a namespace provided, we can assume we're inside
    // chip::.
    let ns  = options.hash.ns ? "chip::" : ""
    typeStr = `${ns}Optional<${typeStr}>`;
  }
  if (options.hash.isArgument && passByReference) {
    typeStr = `const ${typeStr} &`;
  }
  return templateUtil.templatePromise(this.global, Promise.resolve(typeStr))
}

function zapTypeToEncodableClusterObjectType(type, options)
{
  return zapTypeToClusterObjectType.call(this, type, false, options)
}

function zapTypeToDecodableClusterObjectType(type, options)
{
  return zapTypeToClusterObjectType.call(this, type, true, options)
}

async function _zapTypeToPythonClusterObjectType(type, options)
{
  async function fn(pkgId)
  {
    const ns          = options.hash.ns;
    const typeChecker = async (method) => zclHelper[method](this.global.db, type, pkgId).then(zclType => zclType != 'unknown');

    if (await typeChecker('isEnum')) {
      return ns + '.Enums.' + type;
    }

    if (await typeChecker('isBitmap')) {
      return 'uint';
    }

    if (await typeChecker('isStruct')) {
      return ns + '.Structs.' + type;
    }

    if (StringHelper.isCharString(type)) {
      return 'str';
    }

    if (StringHelper.isOctetString(type)) {
      return 'bytes';
    }

    if ([ 'single', 'double' ].includes(type.toLowerCase())) {
      return 'float';
    }

    if (type.toLowerCase() == 'boolean') {
      return 'bool'
    }

    // #10748: asUnderlyingZclType will emit wrong types for int{48|56|64}(u), so we process all int values here.
    if (type.toLowerCase().match(/^int\d+$/)) {
      return 'int'
    }

    if (type.toLowerCase().match(/^int\d+u$/)) {
      return 'uint'
    }

    resolvedType = await zclHelper.asUnderlyingZclType.call({ global : this.global }, type, options);
    {
      basicType = ChipTypesHelper.asBasicType(resolvedType);
      if (basicType.match(/^int\d+_t$/)) {
        return 'int'
      }
      if (basicType.match(/^uint\d+_t$/)) {
        return 'uint'
      }
    }
  }

  let promise = templateUtil.ensureZclPackageId(this).then(fn.bind(this));
  if ((this.isList || this.isArray || this.entryType) && !options.hash.forceNotList) {
    promise = promise.then(typeStr => `typing.List[${typeStr}]`);
  }

  const isNull     = (this.isNullable && !options.hash.forceNotNullable);
  const isOptional = (this.isOptional && !options.hash.forceNotOptional);

  if (isNull && isOptional) {
    promise = promise.then(typeStr => `typing.Union[None, Nullable, ${typeStr}]`);
  } else if (isNull) {
    promise = promise.then(typeStr => `typing.Union[Nullable, ${typeStr}]`);
  } else if (isOptional) {
    promise = promise.then(typeStr => `typing.Optional[${typeStr}]`);
  }

  return templateUtil.templatePromise(this.global, promise)
}

function zapTypeToPythonClusterObjectType(type, options)
{
  return _zapTypeToPythonClusterObjectType.call(this, type, options)
}

async function _getPythonFieldDefault(type, options)
{
  async function fn(pkgId)
  {
    const ns          = options.hash.ns;
    const typeChecker = async (method) => zclHelper[method](this.global.db, type, pkgId).then(zclType => zclType != 'unknown');

    if (await typeChecker('isEnum')) {
      return '0';
    }

    if (await typeChecker('isBitmap')) {
      return '0';
    }

    if (await typeChecker('isStruct')) {
      return 'field(default_factory=lambda: ' + ns + '.Structs.' + type + '())';
    }

    if (StringHelper.isCharString(type)) {
      return '""';
    }

    if (StringHelper.isOctetString(type)) {
      return 'b""';
    }

    if ([ 'single', 'double' ].includes(type.toLowerCase())) {
      return '0.0';
    }

    if (type.toLowerCase() == 'boolean') {
      return 'False'
    }

    // #10748: asUnderlyingZclType will emit wrong types for int{48|56|64}(u), so we process all int values here.
    if (type.toLowerCase().match(/^int\d+$/)) {
      return '0'
    }

    if (type.toLowerCase().match(/^int\d+u$/)) {
      return '0'
    }

    resolvedType = await zclHelper.asUnderlyingZclType.call({ global : this.global }, type, options);
    {
      basicType = ChipTypesHelper.asBasicType(resolvedType);
      if (basicType.match(/^int\d+_t$/)) {
        return '0'
      }
      if (basicType.match(/^uint\d+_t$/)) {
        return '0'
      }
    }
  }

  let promise = templateUtil.ensureZclPackageId(this).then(fn.bind(this));
  if ((this.isList || this.isArray || this.entryType) && !options.hash.forceNotList) {
    promise = promise.then(typeStr => `field(default_factory=lambda: [])`);
  }

  const isNull     = (this.isNullable && !options.hash.forceNotNullable);
  const isOptional = (this.isOptional && !options.hash.forceNotOptional);

  if (isNull && isOptional) {
    promise = promise.then(typeStr => `None`);
  } else if (isNull) {
    promise = promise.then(typeStr => `NullValue`);
  } else if (isOptional) {
    promise = promise.then(typeStr => `None`);
  }

  return templateUtil.templatePromise(this.global, promise)
}

function getPythonFieldDefault(type, options)
{
  return _getPythonFieldDefault.call(this, type, options)
}

async function getResponseCommandName(responseRef, options)
{
  let pkgId = await templateUtil.ensureZclPackageId(this);

  const { db, sessionId } = this.global;
  return queryCommand.selectCommandById(db, responseRef, pkgId).then(response => asUpperCamelCase(response.name));
}

// Allow-list of enums that we generate as enums, not enum classes.  The goal is
// to drive this down to 0.
function isWeaklyTypedEnum(label)
{
  return [
    "ApplicationLauncherStatus",
    "AttributeWritePermission",
    "AudioOutputType",
    "BarrierControlBarrierPosition",
    "BarrierControlMovingState",
    "BootReasonType",
    "ChangeReasonEnum",
    "ColorControlOptions",
    "ColorLoopAction",
    "ColorLoopDirection",
    "ColorMode",
    "ContentLaunchStatus",
    "ContentLaunchStreamingType",
    "DoorLockEventSource",
    "DoorLockEventType",
    "DoorLockOperatingMode",
    "DoorLockOperationEventCode",
    "DoorLockProgrammingEventCode",
    "DoorLockState",
    "DoorLockUserStatus",
    "DoorLockUserType",
    "DoorState",
    "EnhancedColorMode",
    "HardwareFaultType",
    "HueDirection",
    "HueMoveMode",
    "HueStepMode",
    "IasEnrollResponseCode",
    "IasZoneState",
    "IasZoneType",
    "IdentifyEffectIdentifier",
    "IdentifyEffectVariant",
    "IdentifyIdentifyType",
    "InterfaceType",
    "KeypadInputCecKeyCode",
    "KeypadInputStatus",
    "KeypadLockout",
    "LevelControlOptions",
    "MediaInputType",
    "MediaPlaybackState",
    "MediaPlaybackStatus",
    "MoveMode",
    "NetworkCommissioningError",
    "NetworkFaultType",
    "NodeOperationalCertStatus",
    "OTAAnnouncementReason",
    "OTAApplyUpdateAction",
    "OTADownloadProtocol",
    "OnOffDelayedAllOffEffectVariant",
    "OnOffDyingLightEffectVariant",
    "OnOffEffectIdentifier",
    "PHYRateType",
    "RadioFaultType",
    "RoutingRole",
    "RegulatoryLocationType",
    "SaturationMoveMode",
    "SaturationStepMode",
    "SecurityType",
    "SetpointAdjustMode",
    "StartUpOnOffValue",
    "StatusCode",
    "StepMode",
    "TemperatureDisplayMode",
    "ThermostatControlSequence",
    "ThermostatRunningMode",
    "ThermostatSystemMode",
    "UpdateStateEnum",
    "WcEndProductType",
    "WcType",
    "WiFiVersionType",
  ].includes(label);
}

//
// Module exports
//
exports.asPrintFormat                       = asPrintFormat;
exports.asReadType                          = asReadType;
exports.chip_endpoint_generated_functions   = chip_endpoint_generated_functions
exports.chip_endpoint_cluster_list          = chip_endpoint_cluster_list
exports.asTypedLiteral                      = asTypedLiteral;
exports.asLowerCamelCase                    = asLowerCamelCase;
exports.asUpperCamelCase                    = asUpperCamelCase;
exports.hasSpecificAttributes               = hasSpecificAttributes;
exports.asMEI                               = asMEI;
exports.zapTypeToEncodableClusterObjectType = zapTypeToEncodableClusterObjectType;
exports.zapTypeToDecodableClusterObjectType = zapTypeToDecodableClusterObjectType;
exports.zapTypeToPythonClusterObjectType    = zapTypeToPythonClusterObjectType;
exports.getResponseCommandName              = getResponseCommandName;
exports.isWeaklyTypedEnum                   = isWeaklyTypedEnum;
exports.getPythonFieldDefault               = getPythonFieldDefault;

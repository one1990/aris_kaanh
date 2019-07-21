// Auto-generated. Do not edit!

// (in-package controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class motorpos {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angular = null;
    }
    else {
      if (initObj.hasOwnProperty('angular')) {
        this.angular = initObj.angular
      }
      else {
        this.angular = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motorpos
    // Serialize message field [angular]
    bufferOffset = _serializer.uint16(obj.angular, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motorpos
    let len;
    let data = new motorpos(null);
    // Deserialize message field [angular]
    data.angular = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'controller/motorpos';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '620daad8e7540d60806fb147811e141d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 angular
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motorpos(null);
    if (msg.angular !== undefined) {
      resolved.angular = msg.angular;
    }
    else {
      resolved.angular = 0
    }

    return resolved;
    }
};

module.exports = motorpos;

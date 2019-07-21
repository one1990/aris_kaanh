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

class pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pose
    // Serialize message field [pose]
    bufferOffset = _serializer.string(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pose
    let len;
    let data = new pose(null);
    // Deserialize message field [pose]
    data.pose = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.pose.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'controller/pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0cbc29f91adcb71214b885e1c0073aa7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string pose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pose(null);
    if (msg.pose !== undefined) {
      resolved.pose = msg.pose;
    }
    else {
      resolved.pose = ''
    }

    return resolved;
    }
};

module.exports = pose;

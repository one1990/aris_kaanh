// Auto-generated. Do not edit!

// (in-package controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class setposRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setposRequest
    // Serialize message field [cmd]
    bufferOffset = _serializer.uint16(obj.cmd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setposRequest
    let len;
    let data = new setposRequest(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller/setposRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2e923d8b5bede2be04b562e0f28dfb89';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 cmd
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setposRequest(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = 0
    }

    return resolved;
    }
};

class setposResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.return_code = null;
    }
    else {
      if (initObj.hasOwnProperty('return_code')) {
        this.return_code = initObj.return_code
      }
      else {
        this.return_code = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setposResponse
    // Serialize message field [return_code]
    bufferOffset = _serializer.int64(obj.return_code, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setposResponse
    let len;
    let data = new setposResponse(null);
    // Deserialize message field [return_code]
    data.return_code = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller/setposResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fef2d33174a4c686f62532cd40b1b7da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 return_code
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setposResponse(null);
    if (msg.return_code !== undefined) {
      resolved.return_code = msg.return_code;
    }
    else {
      resolved.return_code = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: setposRequest,
  Response: setposResponse,
  md5sum() { return 'b7d0a4a753fe0e11a1400076e4574484'; },
  datatype() { return 'controller/setpos'; }
};

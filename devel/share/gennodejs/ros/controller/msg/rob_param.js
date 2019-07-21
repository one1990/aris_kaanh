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

class rob_param {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.rx = null;
      this.ry = null;
      this.rz = null;
      this.j1 = null;
      this.j2 = null;
      this.j3 = null;
      this.j4 = null;
      this.j5 = null;
      this.j6 = null;
      this.select_mode = null;
      this.start = null;
      this.forward_back = null;
      this.gear = null;
      this.rs_button = null;
      this.md_ds_button = null;
      this.rc_en_button = null;
      this.start_stop = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0;
      }
      if (initObj.hasOwnProperty('rx')) {
        this.rx = initObj.rx
      }
      else {
        this.rx = 0;
      }
      if (initObj.hasOwnProperty('ry')) {
        this.ry = initObj.ry
      }
      else {
        this.ry = 0;
      }
      if (initObj.hasOwnProperty('rz')) {
        this.rz = initObj.rz
      }
      else {
        this.rz = 0.0;
      }
      if (initObj.hasOwnProperty('j1')) {
        this.j1 = initObj.j1
      }
      else {
        this.j1 = 0;
      }
      if (initObj.hasOwnProperty('j2')) {
        this.j2 = initObj.j2
      }
      else {
        this.j2 = 0;
      }
      if (initObj.hasOwnProperty('j3')) {
        this.j3 = initObj.j3
      }
      else {
        this.j3 = 0;
      }
      if (initObj.hasOwnProperty('j4')) {
        this.j4 = initObj.j4
      }
      else {
        this.j4 = 0;
      }
      if (initObj.hasOwnProperty('j5')) {
        this.j5 = initObj.j5
      }
      else {
        this.j5 = 0;
      }
      if (initObj.hasOwnProperty('j6')) {
        this.j6 = initObj.j6
      }
      else {
        this.j6 = 0.0;
      }
      if (initObj.hasOwnProperty('select_mode')) {
        this.select_mode = initObj.select_mode
      }
      else {
        this.select_mode = 0;
      }
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = 0;
      }
      if (initObj.hasOwnProperty('forward_back')) {
        this.forward_back = initObj.forward_back
      }
      else {
        this.forward_back = 0.0;
      }
      if (initObj.hasOwnProperty('gear')) {
        this.gear = initObj.gear
      }
      else {
        this.gear = 0;
      }
      if (initObj.hasOwnProperty('rs_button')) {
        this.rs_button = initObj.rs_button
      }
      else {
        this.rs_button = 0;
      }
      if (initObj.hasOwnProperty('md_ds_button')) {
        this.md_ds_button = initObj.md_ds_button
      }
      else {
        this.md_ds_button = 0;
      }
      if (initObj.hasOwnProperty('rc_en_button')) {
        this.rc_en_button = initObj.rc_en_button
      }
      else {
        this.rc_en_button = 0.0;
      }
      if (initObj.hasOwnProperty('start_stop')) {
        this.start_stop = initObj.start_stop
      }
      else {
        this.start_stop = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rob_param
    // Serialize message field [x]
    bufferOffset = _serializer.int64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.int64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.int64(obj.z, buffer, bufferOffset);
    // Serialize message field [rx]
    bufferOffset = _serializer.int64(obj.rx, buffer, bufferOffset);
    // Serialize message field [ry]
    bufferOffset = _serializer.int64(obj.ry, buffer, bufferOffset);
    // Serialize message field [rz]
    bufferOffset = _serializer.float64(obj.rz, buffer, bufferOffset);
    // Serialize message field [j1]
    bufferOffset = _serializer.int64(obj.j1, buffer, bufferOffset);
    // Serialize message field [j2]
    bufferOffset = _serializer.int64(obj.j2, buffer, bufferOffset);
    // Serialize message field [j3]
    bufferOffset = _serializer.int64(obj.j3, buffer, bufferOffset);
    // Serialize message field [j4]
    bufferOffset = _serializer.int64(obj.j4, buffer, bufferOffset);
    // Serialize message field [j5]
    bufferOffset = _serializer.int64(obj.j5, buffer, bufferOffset);
    // Serialize message field [j6]
    bufferOffset = _serializer.float64(obj.j6, buffer, bufferOffset);
    // Serialize message field [select_mode]
    bufferOffset = _serializer.int64(obj.select_mode, buffer, bufferOffset);
    // Serialize message field [start]
    bufferOffset = _serializer.int64(obj.start, buffer, bufferOffset);
    // Serialize message field [forward_back]
    bufferOffset = _serializer.float64(obj.forward_back, buffer, bufferOffset);
    // Serialize message field [gear]
    bufferOffset = _serializer.int64(obj.gear, buffer, bufferOffset);
    // Serialize message field [rs_button]
    bufferOffset = _serializer.int64(obj.rs_button, buffer, bufferOffset);
    // Serialize message field [md_ds_button]
    bufferOffset = _serializer.int64(obj.md_ds_button, buffer, bufferOffset);
    // Serialize message field [rc_en_button]
    bufferOffset = _serializer.float64(obj.rc_en_button, buffer, bufferOffset);
    // Serialize message field [start_stop]
    bufferOffset = _serializer.float64(obj.start_stop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rob_param
    let len;
    let data = new rob_param(null);
    // Deserialize message field [x]
    data.x = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [rx]
    data.rx = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [ry]
    data.ry = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [rz]
    data.rz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [j1]
    data.j1 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [j2]
    data.j2 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [j3]
    data.j3 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [j4]
    data.j4 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [j5]
    data.j5 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [j6]
    data.j6 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [select_mode]
    data.select_mode = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [start]
    data.start = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [forward_back]
    data.forward_back = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gear]
    data.gear = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [rs_button]
    data.rs_button = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [md_ds_button]
    data.md_ds_button = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [rc_en_button]
    data.rc_en_button = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [start_stop]
    data.start_stop = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 160;
  }

  static datatype() {
    // Returns string type for a message object
    return 'controller/rob_param';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0491c34928b4803b4c4b4d889e27e443';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 x
    int64 y 
    int64 z
    int64 rx
    int64 ry
    float64 rz
    int64 j1
    int64 j2
    int64 j3
    int64 j4
    int64 j5
    float64 j6
    int64 select_mode
    int64 start
    float64 forward_back
    int64 gear
    int64 rs_button
    int64 md_ds_button
    float64 rc_en_button
    float64 start_stop
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rob_param(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0
    }

    if (msg.rx !== undefined) {
      resolved.rx = msg.rx;
    }
    else {
      resolved.rx = 0
    }

    if (msg.ry !== undefined) {
      resolved.ry = msg.ry;
    }
    else {
      resolved.ry = 0
    }

    if (msg.rz !== undefined) {
      resolved.rz = msg.rz;
    }
    else {
      resolved.rz = 0.0
    }

    if (msg.j1 !== undefined) {
      resolved.j1 = msg.j1;
    }
    else {
      resolved.j1 = 0
    }

    if (msg.j2 !== undefined) {
      resolved.j2 = msg.j2;
    }
    else {
      resolved.j2 = 0
    }

    if (msg.j3 !== undefined) {
      resolved.j3 = msg.j3;
    }
    else {
      resolved.j3 = 0
    }

    if (msg.j4 !== undefined) {
      resolved.j4 = msg.j4;
    }
    else {
      resolved.j4 = 0
    }

    if (msg.j5 !== undefined) {
      resolved.j5 = msg.j5;
    }
    else {
      resolved.j5 = 0
    }

    if (msg.j6 !== undefined) {
      resolved.j6 = msg.j6;
    }
    else {
      resolved.j6 = 0.0
    }

    if (msg.select_mode !== undefined) {
      resolved.select_mode = msg.select_mode;
    }
    else {
      resolved.select_mode = 0
    }

    if (msg.start !== undefined) {
      resolved.start = msg.start;
    }
    else {
      resolved.start = 0
    }

    if (msg.forward_back !== undefined) {
      resolved.forward_back = msg.forward_back;
    }
    else {
      resolved.forward_back = 0.0
    }

    if (msg.gear !== undefined) {
      resolved.gear = msg.gear;
    }
    else {
      resolved.gear = 0
    }

    if (msg.rs_button !== undefined) {
      resolved.rs_button = msg.rs_button;
    }
    else {
      resolved.rs_button = 0
    }

    if (msg.md_ds_button !== undefined) {
      resolved.md_ds_button = msg.md_ds_button;
    }
    else {
      resolved.md_ds_button = 0
    }

    if (msg.rc_en_button !== undefined) {
      resolved.rc_en_button = msg.rc_en_button;
    }
    else {
      resolved.rc_en_button = 0.0
    }

    if (msg.start_stop !== undefined) {
      resolved.start_stop = msg.start_stop;
    }
    else {
      resolved.start_stop = 0.0
    }

    return resolved;
    }
};

module.exports = rob_param;

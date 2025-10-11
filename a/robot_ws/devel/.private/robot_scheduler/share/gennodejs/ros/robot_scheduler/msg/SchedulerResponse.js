// Auto-generated. Do not edit!

// (in-package robot_scheduler.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SchedulerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.uid = null;
      this.target = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('uid')) {
        this.uid = initObj.uid
      }
      else {
        this.uid = 0;
      }
      if (initObj.hasOwnProperty('target')) {
        this.target = initObj.target
      }
      else {
        this.target = '';
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SchedulerResponse
    // Serialize message field [uid]
    bufferOffset = _serializer.uint64(obj.uid, buffer, bufferOffset);
    // Serialize message field [target]
    bufferOffset = _serializer.string(obj.target, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SchedulerResponse
    let len;
    let data = new SchedulerResponse(null);
    // Deserialize message field [uid]
    data.uid = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [target]
    data.target = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.target);
    length += _getByteLength(object.status);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_scheduler/SchedulerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e762c6baa6b0e70c2b053bc0f5604848';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint64 uid
    string target
    string status      
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SchedulerResponse(null);
    if (msg.uid !== undefined) {
      resolved.uid = msg.uid;
    }
    else {
      resolved.uid = 0
    }

    if (msg.target !== undefined) {
      resolved.target = msg.target;
    }
    else {
      resolved.target = ''
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    return resolved;
    }
};

module.exports = SchedulerResponse;

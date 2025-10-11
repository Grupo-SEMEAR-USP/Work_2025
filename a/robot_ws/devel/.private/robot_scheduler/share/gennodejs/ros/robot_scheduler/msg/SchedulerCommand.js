// Auto-generated. Do not edit!

// (in-package robot_scheduler.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SchedulerCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.uid = null;
      this.target = null;
      this.payload = null;
      this.need_ack = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
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
      if (initObj.hasOwnProperty('payload')) {
        this.payload = initObj.payload
      }
      else {
        this.payload = '';
      }
      if (initObj.hasOwnProperty('need_ack')) {
        this.need_ack = initObj.need_ack
      }
      else {
        this.need_ack = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SchedulerCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [uid]
    bufferOffset = _serializer.uint64(obj.uid, buffer, bufferOffset);
    // Serialize message field [target]
    bufferOffset = _serializer.string(obj.target, buffer, bufferOffset);
    // Serialize message field [payload]
    bufferOffset = _serializer.string(obj.payload, buffer, bufferOffset);
    // Serialize message field [need_ack]
    bufferOffset = _serializer.bool(obj.need_ack, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SchedulerCommand
    let len;
    let data = new SchedulerCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [uid]
    data.uid = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [target]
    data.target = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [payload]
    data.payload = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [need_ack]
    data.need_ack = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.target);
    length += _getByteLength(object.payload);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_scheduler/SchedulerCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8c8e7e95e708dac0be8b710a396741c1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint64 uid
    string target
    string payload
    bool need_ack
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SchedulerCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

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

    if (msg.payload !== undefined) {
      resolved.payload = msg.payload;
    }
    else {
      resolved.payload = ''
    }

    if (msg.need_ack !== undefined) {
      resolved.need_ack = msg.need_ack;
    }
    else {
      resolved.need_ack = false
    }

    return resolved;
    }
};

module.exports = SchedulerCommand;

// Auto-generated. Do not edit!

// (in-package robot_base_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class encoder_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.front_right_encoder_data = null;
      this.front_left_encoder_data = null;
      this.rear_right_encoder_data = null;
      this.rear_left_encoder_data = null;
    }
    else {
      if (initObj.hasOwnProperty('front_right_encoder_data')) {
        this.front_right_encoder_data = initObj.front_right_encoder_data
      }
      else {
        this.front_right_encoder_data = 0.0;
      }
      if (initObj.hasOwnProperty('front_left_encoder_data')) {
        this.front_left_encoder_data = initObj.front_left_encoder_data
      }
      else {
        this.front_left_encoder_data = 0.0;
      }
      if (initObj.hasOwnProperty('rear_right_encoder_data')) {
        this.rear_right_encoder_data = initObj.rear_right_encoder_data
      }
      else {
        this.rear_right_encoder_data = 0.0;
      }
      if (initObj.hasOwnProperty('rear_left_encoder_data')) {
        this.rear_left_encoder_data = initObj.rear_left_encoder_data
      }
      else {
        this.rear_left_encoder_data = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type encoder_data
    // Serialize message field [front_right_encoder_data]
    bufferOffset = _serializer.float32(obj.front_right_encoder_data, buffer, bufferOffset);
    // Serialize message field [front_left_encoder_data]
    bufferOffset = _serializer.float32(obj.front_left_encoder_data, buffer, bufferOffset);
    // Serialize message field [rear_right_encoder_data]
    bufferOffset = _serializer.float32(obj.rear_right_encoder_data, buffer, bufferOffset);
    // Serialize message field [rear_left_encoder_data]
    bufferOffset = _serializer.float32(obj.rear_left_encoder_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type encoder_data
    let len;
    let data = new encoder_data(null);
    // Deserialize message field [front_right_encoder_data]
    data.front_right_encoder_data = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [front_left_encoder_data]
    data.front_left_encoder_data = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rear_right_encoder_data]
    data.rear_right_encoder_data = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rear_left_encoder_data]
    data.rear_left_encoder_data = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_base_controller/encoder_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c8bd3a66bc403e0d2c70d6f2f2b5db75';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # encoder_data.msg
    float32 front_right_encoder_data
    float32 front_left_encoder_data
    float32 rear_right_encoder_data
    float32 rear_left_encoder_data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new encoder_data(null);
    if (msg.front_right_encoder_data !== undefined) {
      resolved.front_right_encoder_data = msg.front_right_encoder_data;
    }
    else {
      resolved.front_right_encoder_data = 0.0
    }

    if (msg.front_left_encoder_data !== undefined) {
      resolved.front_left_encoder_data = msg.front_left_encoder_data;
    }
    else {
      resolved.front_left_encoder_data = 0.0
    }

    if (msg.rear_right_encoder_data !== undefined) {
      resolved.rear_right_encoder_data = msg.rear_right_encoder_data;
    }
    else {
      resolved.rear_right_encoder_data = 0.0
    }

    if (msg.rear_left_encoder_data !== undefined) {
      resolved.rear_left_encoder_data = msg.rear_left_encoder_data;
    }
    else {
      resolved.rear_left_encoder_data = 0.0
    }

    return resolved;
    }
};

module.exports = encoder_data;

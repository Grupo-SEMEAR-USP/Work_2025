// Auto-generated. Do not edit!

// (in-package robot_scheduler.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Object = require('./Object.js');

//-----------------------------------------------------------

class Subtask {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object = null;
      this.source = null;
      this.destination = null;
    }
    else {
      if (initObj.hasOwnProperty('object')) {
        this.object = initObj.object
      }
      else {
        this.object = new Object();
      }
      if (initObj.hasOwnProperty('source')) {
        this.source = initObj.source
      }
      else {
        this.source = '';
      }
      if (initObj.hasOwnProperty('destination')) {
        this.destination = initObj.destination
      }
      else {
        this.destination = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Subtask
    // Serialize message field [object]
    bufferOffset = Object.serialize(obj.object, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _serializer.string(obj.source, buffer, bufferOffset);
    // Serialize message field [destination]
    bufferOffset = _serializer.string(obj.destination, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Subtask
    let len;
    let data = new Subtask(null);
    // Deserialize message field [object]
    data.object = Object.deserialize(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [destination]
    data.destination = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Object.getMessageSize(object.object);
    length += _getByteLength(object.source);
    length += _getByteLength(object.destination);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_scheduler/Subtask';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6933e02dbcb5080f315b15eafcd4e885';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    robot_scheduler/Object object
    string source
    string destination
    
    ================================================================================
    MSG: robot_scheduler/Object
    uint16 object
    uint16 target
    bool decoy
    geometry_msgs/PoseStamped pose
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Subtask(null);
    if (msg.object !== undefined) {
      resolved.object = Object.Resolve(msg.object)
    }
    else {
      resolved.object = new Object()
    }

    if (msg.source !== undefined) {
      resolved.source = msg.source;
    }
    else {
      resolved.source = ''
    }

    if (msg.destination !== undefined) {
      resolved.destination = msg.destination;
    }
    else {
      resolved.destination = ''
    }

    return resolved;
    }
};

module.exports = Subtask;

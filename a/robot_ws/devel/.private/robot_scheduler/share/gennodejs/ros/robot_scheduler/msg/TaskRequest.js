// Auto-generated. Do not edit!

// (in-package robot_scheduler.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Subtask = require('./Subtask.js');

//-----------------------------------------------------------

class TaskRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.subtasks = null;
      this.execute_on = null;
    }
    else {
      if (initObj.hasOwnProperty('subtasks')) {
        this.subtasks = initObj.subtasks
      }
      else {
        this.subtasks = [];
      }
      if (initObj.hasOwnProperty('execute_on')) {
        this.execute_on = initObj.execute_on
      }
      else {
        this.execute_on = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TaskRequest
    // Serialize message field [subtasks]
    // Serialize the length for message field [subtasks]
    bufferOffset = _serializer.uint32(obj.subtasks.length, buffer, bufferOffset);
    obj.subtasks.forEach((val) => {
      bufferOffset = Subtask.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [execute_on]
    bufferOffset = _arraySerializer.string(obj.execute_on, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TaskRequest
    let len;
    let data = new TaskRequest(null);
    // Deserialize message field [subtasks]
    // Deserialize array length for message field [subtasks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.subtasks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.subtasks[i] = Subtask.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [execute_on]
    data.execute_on = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.subtasks.forEach((val) => {
      length += Subtask.getMessageSize(val);
    });
    object.execute_on.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_scheduler/TaskRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd4c07a22d46edc9f78c6294bee4a791a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    robot_scheduler/Subtask[] subtasks
    string[] execute_on
    
    ================================================================================
    MSG: robot_scheduler/Subtask
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
    const resolved = new TaskRequest(null);
    if (msg.subtasks !== undefined) {
      resolved.subtasks = new Array(msg.subtasks.length);
      for (let i = 0; i < resolved.subtasks.length; ++i) {
        resolved.subtasks[i] = Subtask.Resolve(msg.subtasks[i]);
      }
    }
    else {
      resolved.subtasks = []
    }

    if (msg.execute_on !== undefined) {
      resolved.execute_on = msg.execute_on;
    }
    else {
      resolved.execute_on = []
    }

    return resolved;
    }
};

module.exports = TaskRequest;

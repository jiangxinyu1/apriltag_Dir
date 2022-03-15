// Auto-generated. Do not edit!

// (in-package apriltags_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class AprilTag {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.corners_px = null;
      this.pose_3d = null;
      this.pose_valid = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('corners_px')) {
        this.corners_px = initObj.corners_px
      }
      else {
        this.corners_px = [];
      }
      if (initObj.hasOwnProperty('pose_3d')) {
        this.pose_3d = initObj.pose_3d
      }
      else {
        this.pose_3d = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('pose_valid')) {
        this.pose_valid = initObj.pose_valid
      }
      else {
        this.pose_valid = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AprilTag
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [corners_px]
    // Serialize the length for message field [corners_px]
    bufferOffset = _serializer.uint32(obj.corners_px.length, buffer, bufferOffset);
    obj.corners_px.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [pose_3d]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose_3d, buffer, bufferOffset);
    // Serialize message field [pose_valid]
    bufferOffset = _serializer.bool(obj.pose_valid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AprilTag
    let len;
    let data = new AprilTag(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [corners_px]
    // Deserialize array length for message field [corners_px]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.corners_px = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.corners_px[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [pose_3d]
    data.pose_3d = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose_valid]
    data.pose_valid = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    length += 24 * object.corners_px.length;
    return length + 65;
  }

  static datatype() {
    // Returns string type for a message object
    return 'apriltags_msgs/AprilTag';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '568095c4db6d177cf3bc296bc284d97b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string id
    geometry_msgs/Point[] corners_px
    geometry_msgs/Pose pose_3d
    bool pose_valid
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
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
    const resolved = new AprilTag(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.corners_px !== undefined) {
      resolved.corners_px = new Array(msg.corners_px.length);
      for (let i = 0; i < resolved.corners_px.length; ++i) {
        resolved.corners_px[i] = geometry_msgs.msg.Point.Resolve(msg.corners_px[i]);
      }
    }
    else {
      resolved.corners_px = []
    }

    if (msg.pose_3d !== undefined) {
      resolved.pose_3d = geometry_msgs.msg.Pose.Resolve(msg.pose_3d)
    }
    else {
      resolved.pose_3d = new geometry_msgs.msg.Pose()
    }

    if (msg.pose_valid !== undefined) {
      resolved.pose_valid = msg.pose_valid;
    }
    else {
      resolved.pose_valid = false
    }

    return resolved;
    }
};

module.exports = AprilTag;

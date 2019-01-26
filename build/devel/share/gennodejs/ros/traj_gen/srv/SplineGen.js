// Auto-generated. Do not edit!

// (in-package traj_gen.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

let PolySplineXYZ = require('../msg/PolySplineXYZ.js');

//-----------------------------------------------------------

class SplineGenRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.v0 = null;
      this.knot = null;
      this.knot_t = null;
    }
    else {
      if (initObj.hasOwnProperty('v0')) {
        this.v0 = initObj.v0
      }
      else {
        this.v0 = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('knot')) {
        this.knot = initObj.knot
      }
      else {
        this.knot = new nav_msgs.msg.Path();
      }
      if (initObj.hasOwnProperty('knot_t')) {
        this.knot_t = initObj.knot_t
      }
      else {
        this.knot_t = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SplineGenRequest
    // Serialize message field [v0]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.v0, buffer, bufferOffset);
    // Serialize message field [knot]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.knot, buffer, bufferOffset);
    // Serialize message field [knot_t]
    bufferOffset = _arraySerializer.float64(obj.knot_t, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SplineGenRequest
    let len;
    let data = new SplineGenRequest(null);
    // Deserialize message field [v0]
    data.v0 = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [knot]
    data.knot = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
    // Deserialize message field [knot_t]
    data.knot_t = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.Path.getMessageSize(object.knot);
    length += 8 * object.knot_t.length;
    return length + 52;
  }

  static datatype() {
    // Returns string type for a service object
    return 'traj_gen/SplineGenRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed5a8605137b03cd1ecf17eead696402';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Twist v0
    nav_msgs/Path knot
    float64[] knot_t
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: nav_msgs/Path
    #An array of poses that represents a Path for a robot to follow
    Header header
    geometry_msgs/PoseStamped[] poses
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    const resolved = new SplineGenRequest(null);
    if (msg.v0 !== undefined) {
      resolved.v0 = geometry_msgs.msg.Twist.Resolve(msg.v0)
    }
    else {
      resolved.v0 = new geometry_msgs.msg.Twist()
    }

    if (msg.knot !== undefined) {
      resolved.knot = nav_msgs.msg.Path.Resolve(msg.knot)
    }
    else {
      resolved.knot = new nav_msgs.msg.Path()
    }

    if (msg.knot_t !== undefined) {
      resolved.knot_t = msg.knot_t;
    }
    else {
      resolved.knot_t = []
    }

    return resolved;
    }
};

class SplineGenResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.spline_xyz = null;
    }
    else {
      if (initObj.hasOwnProperty('spline_xyz')) {
        this.spline_xyz = initObj.spline_xyz
      }
      else {
        this.spline_xyz = new PolySplineXYZ();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SplineGenResponse
    // Serialize message field [spline_xyz]
    bufferOffset = PolySplineXYZ.serialize(obj.spline_xyz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SplineGenResponse
    let len;
    let data = new SplineGenResponse(null);
    // Deserialize message field [spline_xyz]
    data.spline_xyz = PolySplineXYZ.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += PolySplineXYZ.getMessageSize(object.spline_xyz);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'traj_gen/SplineGenResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eeac386e24c9ae4f2b82978b3eb5d95e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    traj_gen/PolySplineXYZ spline_xyz
    
    
    ================================================================================
    MSG: traj_gen/PolySplineXYZ
    bool is_valid
    traj_gen/PolySpline spline_x
    traj_gen/PolySpline spline_y 
    traj_gen/PolySpline spline_z
    int8 n_seg
    int8 poly_order
    float64[] knot_time 
    
    ================================================================================
    MSG: traj_gen/PolySpline
    traj_gen/PolyCoeff[] poly_coeff
    float64[] knot_time 
    int8 n_seg
    bool is_valid
    
    
    
    ================================================================================
    MSG: traj_gen/PolyCoeff
    float64[] coeff
    int8 poly_order
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SplineGenResponse(null);
    if (msg.spline_xyz !== undefined) {
      resolved.spline_xyz = PolySplineXYZ.Resolve(msg.spline_xyz)
    }
    else {
      resolved.spline_xyz = new PolySplineXYZ()
    }

    return resolved;
    }
};

module.exports = {
  Request: SplineGenRequest,
  Response: SplineGenResponse,
  md5sum() { return '18f4637150c5208159e6299948387360'; },
  datatype() { return 'traj_gen/SplineGen'; }
};

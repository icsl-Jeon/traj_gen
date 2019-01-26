// Auto-generated. Do not edit!

// (in-package traj_gen.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PolySpline = require('./PolySpline.js');

//-----------------------------------------------------------

class PolySplineXYZ {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_valid = null;
      this.spline_x = null;
      this.spline_y = null;
      this.spline_z = null;
      this.n_seg = null;
      this.poly_order = null;
      this.knot_time = null;
    }
    else {
      if (initObj.hasOwnProperty('is_valid')) {
        this.is_valid = initObj.is_valid
      }
      else {
        this.is_valid = false;
      }
      if (initObj.hasOwnProperty('spline_x')) {
        this.spline_x = initObj.spline_x
      }
      else {
        this.spline_x = new PolySpline();
      }
      if (initObj.hasOwnProperty('spline_y')) {
        this.spline_y = initObj.spline_y
      }
      else {
        this.spline_y = new PolySpline();
      }
      if (initObj.hasOwnProperty('spline_z')) {
        this.spline_z = initObj.spline_z
      }
      else {
        this.spline_z = new PolySpline();
      }
      if (initObj.hasOwnProperty('n_seg')) {
        this.n_seg = initObj.n_seg
      }
      else {
        this.n_seg = 0;
      }
      if (initObj.hasOwnProperty('poly_order')) {
        this.poly_order = initObj.poly_order
      }
      else {
        this.poly_order = 0;
      }
      if (initObj.hasOwnProperty('knot_time')) {
        this.knot_time = initObj.knot_time
      }
      else {
        this.knot_time = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PolySplineXYZ
    // Serialize message field [is_valid]
    bufferOffset = _serializer.bool(obj.is_valid, buffer, bufferOffset);
    // Serialize message field [spline_x]
    bufferOffset = PolySpline.serialize(obj.spline_x, buffer, bufferOffset);
    // Serialize message field [spline_y]
    bufferOffset = PolySpline.serialize(obj.spline_y, buffer, bufferOffset);
    // Serialize message field [spline_z]
    bufferOffset = PolySpline.serialize(obj.spline_z, buffer, bufferOffset);
    // Serialize message field [n_seg]
    bufferOffset = _serializer.int8(obj.n_seg, buffer, bufferOffset);
    // Serialize message field [poly_order]
    bufferOffset = _serializer.int8(obj.poly_order, buffer, bufferOffset);
    // Serialize message field [knot_time]
    bufferOffset = _arraySerializer.float64(obj.knot_time, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PolySplineXYZ
    let len;
    let data = new PolySplineXYZ(null);
    // Deserialize message field [is_valid]
    data.is_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [spline_x]
    data.spline_x = PolySpline.deserialize(buffer, bufferOffset);
    // Deserialize message field [spline_y]
    data.spline_y = PolySpline.deserialize(buffer, bufferOffset);
    // Deserialize message field [spline_z]
    data.spline_z = PolySpline.deserialize(buffer, bufferOffset);
    // Deserialize message field [n_seg]
    data.n_seg = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [poly_order]
    data.poly_order = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [knot_time]
    data.knot_time = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += PolySpline.getMessageSize(object.spline_x);
    length += PolySpline.getMessageSize(object.spline_y);
    length += PolySpline.getMessageSize(object.spline_z);
    length += 8 * object.knot_time.length;
    return length + 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_gen/PolySplineXYZ';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cde8ea48996e4cbc3defb260ac40b943';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new PolySplineXYZ(null);
    if (msg.is_valid !== undefined) {
      resolved.is_valid = msg.is_valid;
    }
    else {
      resolved.is_valid = false
    }

    if (msg.spline_x !== undefined) {
      resolved.spline_x = PolySpline.Resolve(msg.spline_x)
    }
    else {
      resolved.spline_x = new PolySpline()
    }

    if (msg.spline_y !== undefined) {
      resolved.spline_y = PolySpline.Resolve(msg.spline_y)
    }
    else {
      resolved.spline_y = new PolySpline()
    }

    if (msg.spline_z !== undefined) {
      resolved.spline_z = PolySpline.Resolve(msg.spline_z)
    }
    else {
      resolved.spline_z = new PolySpline()
    }

    if (msg.n_seg !== undefined) {
      resolved.n_seg = msg.n_seg;
    }
    else {
      resolved.n_seg = 0
    }

    if (msg.poly_order !== undefined) {
      resolved.poly_order = msg.poly_order;
    }
    else {
      resolved.poly_order = 0
    }

    if (msg.knot_time !== undefined) {
      resolved.knot_time = msg.knot_time;
    }
    else {
      resolved.knot_time = []
    }

    return resolved;
    }
};

module.exports = PolySplineXYZ;

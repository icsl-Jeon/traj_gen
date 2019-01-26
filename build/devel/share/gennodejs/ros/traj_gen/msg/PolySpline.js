// Auto-generated. Do not edit!

// (in-package traj_gen.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PolyCoeff = require('./PolyCoeff.js');

//-----------------------------------------------------------

class PolySpline {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.poly_coeff = null;
      this.knot_time = null;
      this.n_seg = null;
      this.is_valid = null;
    }
    else {
      if (initObj.hasOwnProperty('poly_coeff')) {
        this.poly_coeff = initObj.poly_coeff
      }
      else {
        this.poly_coeff = [];
      }
      if (initObj.hasOwnProperty('knot_time')) {
        this.knot_time = initObj.knot_time
      }
      else {
        this.knot_time = [];
      }
      if (initObj.hasOwnProperty('n_seg')) {
        this.n_seg = initObj.n_seg
      }
      else {
        this.n_seg = 0;
      }
      if (initObj.hasOwnProperty('is_valid')) {
        this.is_valid = initObj.is_valid
      }
      else {
        this.is_valid = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PolySpline
    // Serialize message field [poly_coeff]
    // Serialize the length for message field [poly_coeff]
    bufferOffset = _serializer.uint32(obj.poly_coeff.length, buffer, bufferOffset);
    obj.poly_coeff.forEach((val) => {
      bufferOffset = PolyCoeff.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [knot_time]
    bufferOffset = _arraySerializer.float64(obj.knot_time, buffer, bufferOffset, null);
    // Serialize message field [n_seg]
    bufferOffset = _serializer.int8(obj.n_seg, buffer, bufferOffset);
    // Serialize message field [is_valid]
    bufferOffset = _serializer.bool(obj.is_valid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PolySpline
    let len;
    let data = new PolySpline(null);
    // Deserialize message field [poly_coeff]
    // Deserialize array length for message field [poly_coeff]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poly_coeff = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poly_coeff[i] = PolyCoeff.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [knot_time]
    data.knot_time = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [n_seg]
    data.n_seg = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [is_valid]
    data.is_valid = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.poly_coeff.forEach((val) => {
      length += PolyCoeff.getMessageSize(val);
    });
    length += 8 * object.knot_time.length;
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_gen/PolySpline';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '370db93bb3910f197622508a6a558d72';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new PolySpline(null);
    if (msg.poly_coeff !== undefined) {
      resolved.poly_coeff = new Array(msg.poly_coeff.length);
      for (let i = 0; i < resolved.poly_coeff.length; ++i) {
        resolved.poly_coeff[i] = PolyCoeff.Resolve(msg.poly_coeff[i]);
      }
    }
    else {
      resolved.poly_coeff = []
    }

    if (msg.knot_time !== undefined) {
      resolved.knot_time = msg.knot_time;
    }
    else {
      resolved.knot_time = []
    }

    if (msg.n_seg !== undefined) {
      resolved.n_seg = msg.n_seg;
    }
    else {
      resolved.n_seg = 0
    }

    if (msg.is_valid !== undefined) {
      resolved.is_valid = msg.is_valid;
    }
    else {
      resolved.is_valid = false
    }

    return resolved;
    }
};

module.exports = PolySpline;

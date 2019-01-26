// Auto-generated. Do not edit!

// (in-package traj_gen.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PolyCoeff {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.coeff = null;
      this.poly_order = null;
    }
    else {
      if (initObj.hasOwnProperty('coeff')) {
        this.coeff = initObj.coeff
      }
      else {
        this.coeff = [];
      }
      if (initObj.hasOwnProperty('poly_order')) {
        this.poly_order = initObj.poly_order
      }
      else {
        this.poly_order = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PolyCoeff
    // Serialize message field [coeff]
    bufferOffset = _arraySerializer.float64(obj.coeff, buffer, bufferOffset, null);
    // Serialize message field [poly_order]
    bufferOffset = _serializer.int8(obj.poly_order, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PolyCoeff
    let len;
    let data = new PolyCoeff(null);
    // Deserialize message field [coeff]
    data.coeff = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [poly_order]
    data.poly_order = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.coeff.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traj_gen/PolyCoeff';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eced8596b6a1e1e44efd431704fe5562';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] coeff
    int8 poly_order
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PolyCoeff(null);
    if (msg.coeff !== undefined) {
      resolved.coeff = msg.coeff;
    }
    else {
      resolved.coeff = []
    }

    if (msg.poly_order !== undefined) {
      resolved.poly_order = msg.poly_order;
    }
    else {
      resolved.poly_order = 0
    }

    return resolved;
    }
};

module.exports = PolyCoeff;

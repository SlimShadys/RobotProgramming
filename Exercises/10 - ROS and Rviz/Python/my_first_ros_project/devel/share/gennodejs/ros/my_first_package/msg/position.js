// Auto-generated. Do not edit!

// (in-package my_first_package.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class position {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.message = null;
      this.x = null;
      this.y = null;
      this.even = null;
      this.array = null;
    }
    else {
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('even')) {
        this.even = initObj.even
      }
      else {
        this.even = false;
      }
      if (initObj.hasOwnProperty('array')) {
        this.array = initObj.array
      }
      else {
        this.array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type position
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [even]
    bufferOffset = _serializer.bool(obj.even, buffer, bufferOffset);
    // Serialize message field [array]
    bufferOffset = _arraySerializer.string(obj.array, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type position
    let len;
    let data = new position(null);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [even]
    data.even = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [array]
    data.array = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    object.array.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_first_package/position';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9a284313f2e54143038565ca094ceb0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string message
    float32 x
    float32 y
    bool even
    string[] array
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new position(null);
    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.even !== undefined) {
      resolved.even = msg.even;
    }
    else {
      resolved.even = false
    }

    if (msg.array !== undefined) {
      resolved.array = msg.array;
    }
    else {
      resolved.array = []
    }

    return resolved;
    }
};

module.exports = position;

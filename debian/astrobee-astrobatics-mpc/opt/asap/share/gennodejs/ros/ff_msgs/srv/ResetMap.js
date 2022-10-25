// Auto-generated. Do not edit!

// (in-package ff_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ResetMapRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.map_file = null;
    }
    else {
      if (initObj.hasOwnProperty('map_file')) {
        this.map_file = initObj.map_file
      }
      else {
        this.map_file = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetMapRequest
    // Serialize message field [map_file]
    bufferOffset = _serializer.string(obj.map_file, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetMapRequest
    let len;
    let data = new ResetMapRequest(null);
    // Deserialize message field [map_file]
    data.map_file = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.map_file.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/ResetMapRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a377c8d7c4f71636969846ebf44e4df2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    string map_file
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetMapRequest(null);
    if (msg.map_file !== undefined) {
      resolved.map_file = msg.map_file;
    }
    else {
      resolved.map_file = ''
    }

    return resolved;
    }
};

class ResetMapResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetMapResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetMapResponse
    let len;
    let data = new ResetMapResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/ResetMapResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetMapResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: ResetMapRequest,
  Response: ResetMapResponse,
  md5sum() { return 'a377c8d7c4f71636969846ebf44e4df2'; },
  datatype() { return 'ff_msgs/ResetMap'; }
};

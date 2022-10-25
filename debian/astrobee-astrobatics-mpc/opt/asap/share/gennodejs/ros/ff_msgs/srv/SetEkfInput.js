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

class SetEkfInputRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetEkfInputRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetEkfInputRequest
    let len;
    let data = new SetEkfInputRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/SetEkfInputRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f21f65ebbf246d59f8a1b8f5fe25668b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    uint8 MODE_MAP_LANDMARKS = 0
    uint8 MODE_AR_TAGS       = 1
    uint8 MODE_HANDRAIL      = 2
    uint8 MODE_NONE          = 3
    uint8 MODE_PERCH         = 4
    uint8 MODE_TRUTH         = 5
    
    uint8 mode
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetEkfInputRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

// Constants for message
SetEkfInputRequest.Constants = {
  MODE_MAP_LANDMARKS: 0,
  MODE_AR_TAGS: 1,
  MODE_HANDRAIL: 2,
  MODE_NONE: 3,
  MODE_PERCH: 4,
  MODE_TRUTH: 5,
}

class SetEkfInputResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetEkfInputResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetEkfInputResponse
    let len;
    let data = new SetEkfInputResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/SetEkfInputResponse';
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
    const resolved = new SetEkfInputResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetEkfInputRequest,
  Response: SetEkfInputResponse,
  md5sum() { return 'f21f65ebbf246d59f8a1b8f5fe25668b'; },
  datatype() { return 'ff_msgs/SetEkfInput'; }
};

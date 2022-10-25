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

class SetRateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.which = null;
      this.rate = null;
    }
    else {
      if (initObj.hasOwnProperty('which')) {
        this.which = initObj.which
      }
      else {
        this.which = 0;
      }
      if (initObj.hasOwnProperty('rate')) {
        this.rate = initObj.rate
      }
      else {
        this.rate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRateRequest
    // Serialize message field [which]
    bufferOffset = _serializer.uint8(obj.which, buffer, bufferOffset);
    // Serialize message field [rate]
    bufferOffset = _serializer.float32(obj.rate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRateRequest
    let len;
    let data = new SetRateRequest(null);
    // Deserialize message field [which]
    data.which = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rate]
    data.rate = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/SetRateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '82ede382e05e19aa4b334143894e5c59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    uint8 COMM_STATUS         = 0
    uint8 CPU_STATE           = 1
    uint8 DISK_STATE          = 2
    uint8 EKF_STATE           = 3
    uint8 GNC_STATE           = 4
    uint8 PMC_CMD_STATE       = 5
    uint8 POSITION            = 6
    uint8 SPARSE_MAPPING_POSE = 7
    
    uint8 which
    float32 rate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRateRequest(null);
    if (msg.which !== undefined) {
      resolved.which = msg.which;
    }
    else {
      resolved.which = 0
    }

    if (msg.rate !== undefined) {
      resolved.rate = msg.rate;
    }
    else {
      resolved.rate = 0.0
    }

    return resolved;
    }
};

// Constants for message
SetRateRequest.Constants = {
  COMM_STATUS: 0,
  CPU_STATE: 1,
  DISK_STATE: 2,
  EKF_STATE: 3,
  GNC_STATE: 4,
  PMC_CMD_STATE: 5,
  POSITION: 6,
  SPARSE_MAPPING_POSE: 7,
}

class SetRateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetRateResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetRateResponse
    let len;
    let data = new SetRateResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.status.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/SetRateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38b8954d32a849f31d78416b12bff5d1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetRateResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetRateRequest,
  Response: SetRateResponse,
  md5sum() { return '64286d5b70f59b1a6bc4f3b0e85ea5c5'; },
  datatype() { return 'ff_msgs/SetRate'; }
};

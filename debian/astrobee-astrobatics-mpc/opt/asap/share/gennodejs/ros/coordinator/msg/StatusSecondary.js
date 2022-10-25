// Auto-generated. Do not edit!

// (in-package coordinator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class StatusSecondary {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.test_number = null;
      this.default_control = null;
      this.flight_mode = null;
      this.test_finished = null;
      this.coord_ok = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('test_number')) {
        this.test_number = initObj.test_number
      }
      else {
        this.test_number = 0;
      }
      if (initObj.hasOwnProperty('default_control')) {
        this.default_control = initObj.default_control
      }
      else {
        this.default_control = false;
      }
      if (initObj.hasOwnProperty('flight_mode')) {
        this.flight_mode = initObj.flight_mode
      }
      else {
        this.flight_mode = '';
      }
      if (initObj.hasOwnProperty('test_finished')) {
        this.test_finished = initObj.test_finished
      }
      else {
        this.test_finished = false;
      }
      if (initObj.hasOwnProperty('coord_ok')) {
        this.coord_ok = initObj.coord_ok
      }
      else {
        this.coord_ok = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StatusSecondary
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [test_number]
    bufferOffset = _serializer.int32(obj.test_number, buffer, bufferOffset);
    // Serialize message field [default_control]
    bufferOffset = _serializer.bool(obj.default_control, buffer, bufferOffset);
    // Serialize message field [flight_mode]
    bufferOffset = _serializer.string(obj.flight_mode, buffer, bufferOffset);
    // Serialize message field [test_finished]
    bufferOffset = _serializer.bool(obj.test_finished, buffer, bufferOffset);
    // Serialize message field [coord_ok]
    bufferOffset = _serializer.bool(obj.coord_ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StatusSecondary
    let len;
    let data = new StatusSecondary(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [test_number]
    data.test_number = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [default_control]
    data.default_control = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flight_mode]
    data.flight_mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [test_finished]
    data.test_finished = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [coord_ok]
    data.coord_ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.flight_mode.length;
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'coordinator/StatusSecondary';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '73edc0983b4cd83de63e2948253578db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    
    # base (shared) values
    int32 test_number
    bool default_control
    string flight_mode
    bool test_finished
    bool coord_ok
    
    # StatusSecondary
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StatusSecondary(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.test_number !== undefined) {
      resolved.test_number = msg.test_number;
    }
    else {
      resolved.test_number = 0
    }

    if (msg.default_control !== undefined) {
      resolved.default_control = msg.default_control;
    }
    else {
      resolved.default_control = false
    }

    if (msg.flight_mode !== undefined) {
      resolved.flight_mode = msg.flight_mode;
    }
    else {
      resolved.flight_mode = ''
    }

    if (msg.test_finished !== undefined) {
      resolved.test_finished = msg.test_finished;
    }
    else {
      resolved.test_finished = false
    }

    if (msg.coord_ok !== undefined) {
      resolved.coord_ok = msg.coord_ok;
    }
    else {
      resolved.coord_ok = false
    }

    return resolved;
    }
};

module.exports = StatusSecondary;

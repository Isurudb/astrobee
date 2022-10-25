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

class StatusPrimary {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.test_number = null;
      this.default_control = null;
      this.flight_mode = null;
      this.test_finished = null;
      this.coord_ok = null;
      this.control_mode = null;
      this.regulate_finished = null;
      this.description = null;
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
      if (initObj.hasOwnProperty('control_mode')) {
        this.control_mode = initObj.control_mode
      }
      else {
        this.control_mode = '';
      }
      if (initObj.hasOwnProperty('regulate_finished')) {
        this.regulate_finished = initObj.regulate_finished
      }
      else {
        this.regulate_finished = false;
      }
      if (initObj.hasOwnProperty('description')) {
        this.description = initObj.description
      }
      else {
        this.description = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StatusPrimary
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
    // Serialize message field [control_mode]
    bufferOffset = _serializer.string(obj.control_mode, buffer, bufferOffset);
    // Serialize message field [regulate_finished]
    bufferOffset = _serializer.bool(obj.regulate_finished, buffer, bufferOffset);
    // Serialize message field [description]
    bufferOffset = _serializer.string(obj.description, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StatusPrimary
    let len;
    let data = new StatusPrimary(null);
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
    // Deserialize message field [control_mode]
    data.control_mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [regulate_finished]
    data.regulate_finished = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [description]
    data.description = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.flight_mode.length;
    length += object.control_mode.length;
    length += object.description.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'coordinator/StatusPrimary';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7b1f7333b81ba92ccdc2c10a9400c075';
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
    
    # StatusPrimary
    string control_mode  # did something break?
    bool regulate_finished
    string description  # for telemetry description
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StatusPrimary(null);
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

    if (msg.control_mode !== undefined) {
      resolved.control_mode = msg.control_mode;
    }
    else {
      resolved.control_mode = ''
    }

    if (msg.regulate_finished !== undefined) {
      resolved.regulate_finished = msg.regulate_finished;
    }
    else {
      resolved.regulate_finished = false
    }

    if (msg.description !== undefined) {
      resolved.description = msg.description;
    }
    else {
      resolved.description = ''
    }

    return resolved;
    }
};

module.exports = StatusPrimary;

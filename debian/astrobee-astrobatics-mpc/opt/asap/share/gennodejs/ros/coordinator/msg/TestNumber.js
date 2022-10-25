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

class TestNumber {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.test_number = null;
      this.role = null;
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
      if (initObj.hasOwnProperty('role')) {
        this.role = initObj.role
      }
      else {
        this.role = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TestNumber
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [test_number]
    bufferOffset = _serializer.int32(obj.test_number, buffer, bufferOffset);
    // Serialize message field [role]
    bufferOffset = _serializer.string(obj.role, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TestNumber
    let len;
    let data = new TestNumber(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [test_number]
    data.test_number = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [role]
    data.role = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.role.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'coordinator/TestNumber';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a546cf58ee360e93604091100205de8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    int32 test_number
    string role
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TestNumber(null);
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

    if (msg.role !== undefined) {
      resolved.role = msg.role;
    }
    else {
      resolved.role = ''
    }

    return resolved;
    }
};

module.exports = TestNumber;

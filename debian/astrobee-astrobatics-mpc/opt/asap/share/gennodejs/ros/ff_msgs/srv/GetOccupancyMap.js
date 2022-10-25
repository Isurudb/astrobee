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

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GetOccupancyMapRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOccupancyMapRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOccupancyMapRequest
    let len;
    let data = new GetOccupancyMapRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/GetOccupancyMapRequest';
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
    const resolved = new GetOccupancyMapRequest(null);
    return resolved;
    }
};

class GetOccupancyMapResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.map = null;
      this.origin = null;
      this.dim = null;
      this.resolution = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('map')) {
        this.map = initObj.map
      }
      else {
        this.map = [];
      }
      if (initObj.hasOwnProperty('origin')) {
        this.origin = initObj.origin
      }
      else {
        this.origin = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('dim')) {
        this.dim = initObj.dim
      }
      else {
        this.dim = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOccupancyMapResponse
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [map]
    bufferOffset = _arraySerializer.int8(obj.map, buffer, bufferOffset, null);
    // Serialize message field [origin]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.origin, buffer, bufferOffset);
    // Serialize message field [dim]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.dim, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.float32(obj.resolution, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOccupancyMapResponse
    let len;
    let data = new GetOccupancyMapResponse(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [map]
    data.map = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [origin]
    data.origin = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [dim]
    data.dim = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.map.length;
    return length + 64;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/GetOccupancyMapResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7424633971d7f4a5e61061b696c2c185';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time timestamp
    
    int8[] map
    geometry_msgs/Vector3 origin
    geometry_msgs/Vector3 dim
    float32 resolution
    
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOccupancyMapResponse(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = {secs: 0, nsecs: 0}
    }

    if (msg.map !== undefined) {
      resolved.map = msg.map;
    }
    else {
      resolved.map = []
    }

    if (msg.origin !== undefined) {
      resolved.origin = geometry_msgs.msg.Vector3.Resolve(msg.origin)
    }
    else {
      resolved.origin = new geometry_msgs.msg.Vector3()
    }

    if (msg.dim !== undefined) {
      resolved.dim = geometry_msgs.msg.Vector3.Resolve(msg.dim)
    }
    else {
      resolved.dim = new geometry_msgs.msg.Vector3()
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: GetOccupancyMapRequest,
  Response: GetOccupancyMapResponse,
  md5sum() { return '7424633971d7f4a5e61061b696c2c185'; },
  datatype() { return 'ff_msgs/GetOccupancyMap'; }
};

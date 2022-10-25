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

class UnloadLoadNodeletRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.load = null;
      this.name = null;
      this.type = null;
      this.manager_name = null;
      this.remap_source_args = null;
      this.remap_target_args = null;
      this.my_argv = null;
      this.bond_id = null;
    }
    else {
      if (initObj.hasOwnProperty('load')) {
        this.load = initObj.load
      }
      else {
        this.load = false;
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('manager_name')) {
        this.manager_name = initObj.manager_name
      }
      else {
        this.manager_name = '';
      }
      if (initObj.hasOwnProperty('remap_source_args')) {
        this.remap_source_args = initObj.remap_source_args
      }
      else {
        this.remap_source_args = [];
      }
      if (initObj.hasOwnProperty('remap_target_args')) {
        this.remap_target_args = initObj.remap_target_args
      }
      else {
        this.remap_target_args = [];
      }
      if (initObj.hasOwnProperty('my_argv')) {
        this.my_argv = initObj.my_argv
      }
      else {
        this.my_argv = [];
      }
      if (initObj.hasOwnProperty('bond_id')) {
        this.bond_id = initObj.bond_id
      }
      else {
        this.bond_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UnloadLoadNodeletRequest
    // Serialize message field [load]
    bufferOffset = _serializer.bool(obj.load, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [manager_name]
    bufferOffset = _serializer.string(obj.manager_name, buffer, bufferOffset);
    // Serialize message field [remap_source_args]
    bufferOffset = _arraySerializer.string(obj.remap_source_args, buffer, bufferOffset, null);
    // Serialize message field [remap_target_args]
    bufferOffset = _arraySerializer.string(obj.remap_target_args, buffer, bufferOffset, null);
    // Serialize message field [my_argv]
    bufferOffset = _arraySerializer.string(obj.my_argv, buffer, bufferOffset, null);
    // Serialize message field [bond_id]
    bufferOffset = _serializer.string(obj.bond_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UnloadLoadNodeletRequest
    let len;
    let data = new UnloadLoadNodeletRequest(null);
    // Deserialize message field [load]
    data.load = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [manager_name]
    data.manager_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [remap_source_args]
    data.remap_source_args = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [remap_target_args]
    data.remap_target_args = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [my_argv]
    data.my_argv = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [bond_id]
    data.bond_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    length += object.type.length;
    length += object.manager_name.length;
    object.remap_source_args.forEach((val) => {
      length += 4 + val.length;
    });
    object.remap_target_args.forEach((val) => {
      length += 4 + val.length;
    });
    object.my_argv.forEach((val) => {
      length += 4 + val.length;
    });
    length += object.bond_id.length;
    return length + 29;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/UnloadLoadNodeletRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7b447d24c984d59cef0d4a86160190f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    bool load
    
    string name
    
    
    
    string type
    
    
    
    string manager_name
    
    string[] remap_source_args
    string[] remap_target_args
    string[] my_argv
    string bond_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UnloadLoadNodeletRequest(null);
    if (msg.load !== undefined) {
      resolved.load = msg.load;
    }
    else {
      resolved.load = false
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.manager_name !== undefined) {
      resolved.manager_name = msg.manager_name;
    }
    else {
      resolved.manager_name = ''
    }

    if (msg.remap_source_args !== undefined) {
      resolved.remap_source_args = msg.remap_source_args;
    }
    else {
      resolved.remap_source_args = []
    }

    if (msg.remap_target_args !== undefined) {
      resolved.remap_target_args = msg.remap_target_args;
    }
    else {
      resolved.remap_target_args = []
    }

    if (msg.my_argv !== undefined) {
      resolved.my_argv = msg.my_argv;
    }
    else {
      resolved.my_argv = []
    }

    if (msg.bond_id !== undefined) {
      resolved.bond_id = msg.bond_id;
    }
    else {
      resolved.bond_id = ''
    }

    return resolved;
    }
};

class UnloadLoadNodeletResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UnloadLoadNodeletResponse
    // Serialize message field [result]
    bufferOffset = _serializer.int32(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UnloadLoadNodeletResponse
    let len;
    let data = new UnloadLoadNodeletResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/UnloadLoadNodeletResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '595e167cf0aa390cd33b2486e284d56d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 result
    
    int32 SUCCESSFUL            = 1
    
    int32 ROS_SERVICE_FAILED    = 2
    
    
    
    int32 NODE_NOT_IN_MAP       = 3
    
    
    int32 MANAGER_NAME_MISSING  = 4
    
    
    int32 TYPE_MISSING          = 5
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UnloadLoadNodeletResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    return resolved;
    }
};

// Constants for message
UnloadLoadNodeletResponse.Constants = {
  SUCCESSFUL: 1,
  ROS_SERVICE_FAILED: 2,
  NODE_NOT_IN_MAP: 3,
  MANAGER_NAME_MISSING: 4,
  TYPE_MISSING: 5,
}

module.exports = {
  Request: UnloadLoadNodeletRequest,
  Response: UnloadLoadNodeletResponse,
  md5sum() { return '7f19eb1a2a34b5a95695a9d88b20e227'; },
  datatype() { return 'ff_msgs/UnloadLoadNodelet'; }
};

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

let Zone = require('../msg/Zone.js');

//-----------------------------------------------------------

class GetZonesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetZonesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetZonesRequest
    let len;
    let data = new GetZonesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/GetZonesRequest';
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
    const resolved = new GetZonesRequest(null);
    return resolved;
    }
};

class GetZonesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.zones = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('zones')) {
        this.zones = initObj.zones
      }
      else {
        this.zones = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetZonesResponse
    // Serialize message field [timestamp]
    bufferOffset = _serializer.time(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [zones]
    // Serialize the length for message field [zones]
    bufferOffset = _serializer.uint32(obj.zones.length, buffer, bufferOffset);
    obj.zones.forEach((val) => {
      bufferOffset = Zone.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetZonesResponse
    let len;
    let data = new GetZonesResponse(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [zones]
    // Deserialize array length for message field [zones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.zones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.zones[i] = Zone.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.zones.forEach((val) => {
      length += Zone.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ff_msgs/GetZonesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bcb94608c442ab09483e9fcbedd6ba7d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time timestamp
    ff_msgs/Zone[] zones
    
    ================================================================================
    MSG: ff_msgs/Zone
    # Copyright (c) 2017, United States Government, as represented by the
    # Administrator of the National Aeronautics and Space Administration.
    # 
    # All rights reserved.
    # 
    # The Astrobee platform is licensed under the Apache License, Version 2.0
    # (the "License"); you may not use this file except in compliance with the
    # License. You may obtain a copy of the License at
    # 
    #     http://www.apache.org/licenses/LICENSE-2.0
    # 
    # Unless required by applicable law or agreed to in writing, software
    # distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
    # WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
    # License for the specific language governing permissions and limitations
    # under the License.
    #
    # This message defines a zone, such as a Keepin or Keeoput.
    
    string name                   # Name of zone
    
    # A name can refer to multiple zones. This is the index of the zone with respect
    # to the zone name
    int32 index
    
    # Zone type
    uint8 KEEPOUT = 0       # An area the freeflyer should stay out of
    uint8 KEEPIN  = 1       # An area the freeflyer can fly freely in 
    uint8 CLUTTER = 2       # An area that the freeflyer should avoid due to clutter
    
    uint8 type              # Whether the zone is a keepin, keepout, or clutter
    
    geometry_msgs/Vector3 min   # One corner of the zone
    geometry_msgs/Vector3 max   # The opposite corner of the zone
    
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
    const resolved = new GetZonesResponse(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = {secs: 0, nsecs: 0}
    }

    if (msg.zones !== undefined) {
      resolved.zones = new Array(msg.zones.length);
      for (let i = 0; i < resolved.zones.length; ++i) {
        resolved.zones[i] = Zone.Resolve(msg.zones[i]);
      }
    }
    else {
      resolved.zones = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GetZonesRequest,
  Response: GetZonesResponse,
  md5sum() { return 'bcb94608c442ab09483e9fcbedd6ba7d'; },
  datatype() { return 'ff_msgs/GetZones'; }
};

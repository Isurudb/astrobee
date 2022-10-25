// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class VisualeyezData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tcmid = null;
      this.ledid = null;
      this.position = null;
    }
    else {
      if (initObj.hasOwnProperty('tcmid')) {
        this.tcmid = initObj.tcmid
      }
      else {
        this.tcmid = 0;
      }
      if (initObj.hasOwnProperty('ledid')) {
        this.ledid = initObj.ledid
      }
      else {
        this.ledid = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisualeyezData
    // Serialize message field [tcmid]
    bufferOffset = _serializer.uint8(obj.tcmid, buffer, bufferOffset);
    // Serialize message field [ledid]
    bufferOffset = _serializer.uint8(obj.ledid, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisualeyezData
    let len;
    let data = new VisualeyezData(null);
    // Deserialize message field [tcmid]
    data.tcmid = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ledid]
    data.ledid = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/VisualeyezData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0a4f041324891dc34a11ad8b15af9d60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    # Raw Visualeyez data.
    
    uint8 tcmid                         # Transmission control module ID
    uint8 ledid                         # Light emitting diode ID
    geometry_msgs/Vector3 position      # Coordinate 
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
    const resolved = new VisualeyezData(null);
    if (msg.tcmid !== undefined) {
      resolved.tcmid = msg.tcmid;
    }
    else {
      resolved.tcmid = 0
    }

    if (msg.ledid !== undefined) {
      resolved.ledid = msg.ledid;
    }
    else {
      resolved.ledid = 0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Vector3.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = VisualeyezData;

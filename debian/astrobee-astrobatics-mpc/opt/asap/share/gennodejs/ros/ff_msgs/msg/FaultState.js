// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Fault = require('./Fault.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FaultState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.hr_state = null;
      this.faults = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('hr_state')) {
        this.hr_state = initObj.hr_state
      }
      else {
        this.hr_state = '';
      }
      if (initObj.hasOwnProperty('faults')) {
        this.faults = initObj.faults
      }
      else {
        this.faults = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FaultState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [hr_state]
    bufferOffset = _serializer.string(obj.hr_state, buffer, bufferOffset);
    // Serialize message field [faults]
    // Serialize the length for message field [faults]
    bufferOffset = _serializer.uint32(obj.faults.length, buffer, bufferOffset);
    obj.faults.forEach((val) => {
      bufferOffset = Fault.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FaultState
    let len;
    let data = new FaultState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [hr_state]
    data.hr_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [faults]
    // Deserialize array length for message field [faults]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.faults = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.faults[i] = Fault.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.hr_state.length;
    object.faults.forEach((val) => {
      length += Fault.getMessageSize(val);
    });
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/FaultState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ded9e5628846b2af7eff4a5b8d34c68';
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
    # Fault state message used to alert the ground of the current faults. It is also
    # used to express to the executive that a fault has occurred that indirectly
    # affects the motion of the robot.
    
    std_msgs/Header header
    
    # Not sent to the ground, only used by the executive to determine what commands
    # to accept.
    uint8 state
    # System starting up
    uint8 STARTING_UP           = 0
    # No faults are occurring in system
    uint8 FUNCTIONAL            = 1
    # Faults are occurring in the system which may or may not leave the robot
    # functional
    uint8 FAULT                 = 2
    # A fault has occurred that indirectly affects the motion of the robot
    uint8 BLOCKED               = 3
    # Recovering from nodes dying on startup
    uint8 RELOADING_NODELETS    = 4
    
    # A human readable version of the state - only really used for when nodes die on
    # startup and need to be restarted.
    string hr_state
    
    # Faults occurring in the astrobee system, can only send 32 faults down
    ff_msgs/Fault[] faults
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: ff_msgs/Fault
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
    # Fault message is used to provide all the information about an occurring fault
    
    time time_of_fault        # Time when fault occurred
    
    uint32 id                 # id specifying fault
    
    string msg                # string specifying why the fault occurred
    
    ff_msgs/FaultData[] data  # Data used for fault analysis
    
    ================================================================================
    MSG: ff_msgs/FaultData
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
    # Fault data messsage contains information of why the fault occurred
    
    uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float
    uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int
    uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string
    
    string key  # Specifies what the data in the msg is, can only be 32 chars long
    
    uint8 data_type   # Specifies the type of data in the message
    
    float32 f   # Value used for fault analysis, data_type must be 0
    int32 i     # Value used for fault analysis, data_type must be 1
    string s    # String used for fault analysis, data_type must be 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FaultState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.hr_state !== undefined) {
      resolved.hr_state = msg.hr_state;
    }
    else {
      resolved.hr_state = ''
    }

    if (msg.faults !== undefined) {
      resolved.faults = new Array(msg.faults.length);
      for (let i = 0; i < resolved.faults.length; ++i) {
        resolved.faults[i] = Fault.Resolve(msg.faults[i]);
      }
    }
    else {
      resolved.faults = []
    }

    return resolved;
    }
};

// Constants for message
FaultState.Constants = {
  STARTING_UP: 0,
  FUNCTIONAL: 1,
  FAULT: 2,
  BLOCKED: 3,
  RELOADING_NODELETS: 4,
}

module.exports = FaultState;

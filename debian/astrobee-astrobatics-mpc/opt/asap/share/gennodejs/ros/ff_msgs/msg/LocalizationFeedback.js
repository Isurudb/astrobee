// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LocalizationState = require('./LocalizationState.js');

//-----------------------------------------------------------

class LocalizationFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = new LocalizationState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocalizationFeedback
    // Serialize message field [state]
    bufferOffset = LocalizationState.serialize(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocalizationFeedback
    let len;
    let data = new LocalizationFeedback(null);
    // Deserialize message field [state]
    data.state = LocalizationState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += LocalizationState.getMessageSize(object.state);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/LocalizationFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9a8c4aa9ea2bc31b1a056756dbbcdb96';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    # Feedback
    ff_msgs/LocalizationState state
    
    
    ================================================================================
    MSG: ff_msgs/LocalizationState
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
    # The state of the localization system
    
    # Header with timestamp
    std_msgs/Header header
    
    # Tee current state
    int32 state                                 # Current state
    int32 INITIALIZING                    = 0   # Waiting on dependencies
    int32 DISABLED                        = 1   # Localization disabled
    int32 LOCALIZING                      = 2   # Localization enabled
    int32 SWITCH_WAITING_FOR_PIPELINE     = 3   # Waiting for pipeline to stabilize
    int32 SWITCH_WAITING_FOR_FILTER       = 4   # Waiting for filter to stabilize
    int32 BIAS_WAITING_FOR_FILTER         = 5   # Waiting for bias estimation
    int32 RESET_WAITING_FOR_FILTER        = 6   # Waiting for EKF stability
    int32 UNSTABLE                        = 7   # Fallback pipeline unstable
    
    # A human readable version of the (event) -> [state] transition
    string fsm_event
    string fsm_state
    
    # The current localization pipeline being used
    ff_msgs/LocalizationPipeline pipeline
    
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
    MSG: ff_msgs/LocalizationPipeline
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
    # Information about a pipeline
    
    string id                     # Short id for the pipeline
    uint8 mode                    # EKF mode for the pipeline
    string name                   # Long name for the pipe
    bool requires_filter          # Does this pipeline require the EKF
    bool requires_optical_flow    # Does this pipeline require optical flow
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LocalizationFeedback(null);
    if (msg.state !== undefined) {
      resolved.state = LocalizationState.Resolve(msg.state)
    }
    else {
      resolved.state = new LocalizationState()
    }

    return resolved;
    }
};

module.exports = LocalizationFeedback;

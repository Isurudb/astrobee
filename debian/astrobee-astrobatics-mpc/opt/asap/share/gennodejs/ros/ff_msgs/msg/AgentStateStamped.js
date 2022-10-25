// Auto-generated. Do not edit!

// (in-package ff_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let OpState = require('./OpState.js');
let ExecState = require('./ExecState.js');
let MobilityState = require('./MobilityState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AgentStateStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.operating_state = null;
      this.plan_execution_state = null;
      this.guest_science_state = null;
      this.mobility_state = null;
      this.proximity = null;
      this.profile_name = null;
      this.flight_mode = null;
      this.target_linear_velocity = null;
      this.target_linear_accel = null;
      this.target_angular_velocity = null;
      this.target_angular_accel = null;
      this.collision_distance = null;
      this.holonomic_enabled = null;
      this.check_obstacles = null;
      this.check_zones = null;
      this.auto_return_enabled = null;
      this.immediate_enabled = null;
      this.planner = null;
      this.replanning_enabled = null;
      this.world = null;
      this.boot_time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('operating_state')) {
        this.operating_state = initObj.operating_state
      }
      else {
        this.operating_state = new OpState();
      }
      if (initObj.hasOwnProperty('plan_execution_state')) {
        this.plan_execution_state = initObj.plan_execution_state
      }
      else {
        this.plan_execution_state = new ExecState();
      }
      if (initObj.hasOwnProperty('guest_science_state')) {
        this.guest_science_state = initObj.guest_science_state
      }
      else {
        this.guest_science_state = new ExecState();
      }
      if (initObj.hasOwnProperty('mobility_state')) {
        this.mobility_state = initObj.mobility_state
      }
      else {
        this.mobility_state = new MobilityState();
      }
      if (initObj.hasOwnProperty('proximity')) {
        this.proximity = initObj.proximity
      }
      else {
        this.proximity = 0.0;
      }
      if (initObj.hasOwnProperty('profile_name')) {
        this.profile_name = initObj.profile_name
      }
      else {
        this.profile_name = '';
      }
      if (initObj.hasOwnProperty('flight_mode')) {
        this.flight_mode = initObj.flight_mode
      }
      else {
        this.flight_mode = '';
      }
      if (initObj.hasOwnProperty('target_linear_velocity')) {
        this.target_linear_velocity = initObj.target_linear_velocity
      }
      else {
        this.target_linear_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('target_linear_accel')) {
        this.target_linear_accel = initObj.target_linear_accel
      }
      else {
        this.target_linear_accel = 0.0;
      }
      if (initObj.hasOwnProperty('target_angular_velocity')) {
        this.target_angular_velocity = initObj.target_angular_velocity
      }
      else {
        this.target_angular_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('target_angular_accel')) {
        this.target_angular_accel = initObj.target_angular_accel
      }
      else {
        this.target_angular_accel = 0.0;
      }
      if (initObj.hasOwnProperty('collision_distance')) {
        this.collision_distance = initObj.collision_distance
      }
      else {
        this.collision_distance = 0.0;
      }
      if (initObj.hasOwnProperty('holonomic_enabled')) {
        this.holonomic_enabled = initObj.holonomic_enabled
      }
      else {
        this.holonomic_enabled = false;
      }
      if (initObj.hasOwnProperty('check_obstacles')) {
        this.check_obstacles = initObj.check_obstacles
      }
      else {
        this.check_obstacles = false;
      }
      if (initObj.hasOwnProperty('check_zones')) {
        this.check_zones = initObj.check_zones
      }
      else {
        this.check_zones = false;
      }
      if (initObj.hasOwnProperty('auto_return_enabled')) {
        this.auto_return_enabled = initObj.auto_return_enabled
      }
      else {
        this.auto_return_enabled = false;
      }
      if (initObj.hasOwnProperty('immediate_enabled')) {
        this.immediate_enabled = initObj.immediate_enabled
      }
      else {
        this.immediate_enabled = false;
      }
      if (initObj.hasOwnProperty('planner')) {
        this.planner = initObj.planner
      }
      else {
        this.planner = '';
      }
      if (initObj.hasOwnProperty('replanning_enabled')) {
        this.replanning_enabled = initObj.replanning_enabled
      }
      else {
        this.replanning_enabled = false;
      }
      if (initObj.hasOwnProperty('world')) {
        this.world = initObj.world
      }
      else {
        this.world = '';
      }
      if (initObj.hasOwnProperty('boot_time')) {
        this.boot_time = initObj.boot_time
      }
      else {
        this.boot_time = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AgentStateStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [operating_state]
    bufferOffset = OpState.serialize(obj.operating_state, buffer, bufferOffset);
    // Serialize message field [plan_execution_state]
    bufferOffset = ExecState.serialize(obj.plan_execution_state, buffer, bufferOffset);
    // Serialize message field [guest_science_state]
    bufferOffset = ExecState.serialize(obj.guest_science_state, buffer, bufferOffset);
    // Serialize message field [mobility_state]
    bufferOffset = MobilityState.serialize(obj.mobility_state, buffer, bufferOffset);
    // Serialize message field [proximity]
    bufferOffset = _serializer.float32(obj.proximity, buffer, bufferOffset);
    // Serialize message field [profile_name]
    bufferOffset = _serializer.string(obj.profile_name, buffer, bufferOffset);
    // Serialize message field [flight_mode]
    bufferOffset = _serializer.string(obj.flight_mode, buffer, bufferOffset);
    // Serialize message field [target_linear_velocity]
    bufferOffset = _serializer.float32(obj.target_linear_velocity, buffer, bufferOffset);
    // Serialize message field [target_linear_accel]
    bufferOffset = _serializer.float32(obj.target_linear_accel, buffer, bufferOffset);
    // Serialize message field [target_angular_velocity]
    bufferOffset = _serializer.float32(obj.target_angular_velocity, buffer, bufferOffset);
    // Serialize message field [target_angular_accel]
    bufferOffset = _serializer.float32(obj.target_angular_accel, buffer, bufferOffset);
    // Serialize message field [collision_distance]
    bufferOffset = _serializer.float32(obj.collision_distance, buffer, bufferOffset);
    // Serialize message field [holonomic_enabled]
    bufferOffset = _serializer.bool(obj.holonomic_enabled, buffer, bufferOffset);
    // Serialize message field [check_obstacles]
    bufferOffset = _serializer.bool(obj.check_obstacles, buffer, bufferOffset);
    // Serialize message field [check_zones]
    bufferOffset = _serializer.bool(obj.check_zones, buffer, bufferOffset);
    // Serialize message field [auto_return_enabled]
    bufferOffset = _serializer.bool(obj.auto_return_enabled, buffer, bufferOffset);
    // Serialize message field [immediate_enabled]
    bufferOffset = _serializer.bool(obj.immediate_enabled, buffer, bufferOffset);
    // Serialize message field [planner]
    bufferOffset = _serializer.string(obj.planner, buffer, bufferOffset);
    // Serialize message field [replanning_enabled]
    bufferOffset = _serializer.bool(obj.replanning_enabled, buffer, bufferOffset);
    // Serialize message field [world]
    bufferOffset = _serializer.string(obj.world, buffer, bufferOffset);
    // Serialize message field [boot_time]
    bufferOffset = _serializer.uint32(obj.boot_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AgentStateStamped
    let len;
    let data = new AgentStateStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [operating_state]
    data.operating_state = OpState.deserialize(buffer, bufferOffset);
    // Deserialize message field [plan_execution_state]
    data.plan_execution_state = ExecState.deserialize(buffer, bufferOffset);
    // Deserialize message field [guest_science_state]
    data.guest_science_state = ExecState.deserialize(buffer, bufferOffset);
    // Deserialize message field [mobility_state]
    data.mobility_state = MobilityState.deserialize(buffer, bufferOffset);
    // Deserialize message field [proximity]
    data.proximity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [profile_name]
    data.profile_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [flight_mode]
    data.flight_mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_linear_velocity]
    data.target_linear_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_linear_accel]
    data.target_linear_accel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_angular_velocity]
    data.target_angular_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_angular_accel]
    data.target_angular_accel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [collision_distance]
    data.collision_distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [holonomic_enabled]
    data.holonomic_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [check_obstacles]
    data.check_obstacles = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [check_zones]
    data.check_zones = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [auto_return_enabled]
    data.auto_return_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [immediate_enabled]
    data.immediate_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [planner]
    data.planner = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [replanning_enabled]
    data.replanning_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [world]
    data.world = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [boot_time]
    data.boot_time = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.profile_name.length;
    length += object.flight_mode.length;
    length += object.planner.length;
    length += object.world.length;
    return length + 58;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ff_msgs/AgentStateStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '156487b23e377e3a1dc7ef079f0e327d';
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
    # State of the Astrobee, based off of rapid::ext::astrobee::AgentState
    
    # Header with timestamp
    std_msgs/Header header
    
    # Operating state of the Astrobee
    ff_msgs/OpState operating_state
    
    # Plan execution state. State is idle when there is no plan to be executed. Once
    # a plan is uploaded, the state change to paused. Upon a run plan command, the
    # state will change to executing. If a stop or pause plan command is received or
    # a fault occurs, the state will be set to pause. Once the plan is completed,
    # the state will go back to idle
    ff_msgs/ExecState plan_execution_state
    
    # Guest science state. If a primary guest science apk is started, the state
    # will go from idle to executing. Once the primarty apk is finished, the state
    # will go back to idle
    ff_msgs/ExecState guest_science_state
    
    # Mobility state of the Astrobee
    ff_msgs/MobilityState mobility_state
    
    # Proximity to the dock when docking and undocking. Proximity to a handrail when
    # perching or unperching. 0 the rest of the time.
    float32 proximity
    
    # Name of profile configuration, i.e. Nominal, IgnoreObstacles, Faceforward,
    # Quiet, etc. Profiles specify stuff like target velocity and acceleration,
    # collision distance, etc.
    string profile_name
    
    #Defines GN&C gains, hard limits, tolerances, etc.
    string flight_mode
    
    # Maximum linear velocity to target while translating
    float32 target_linear_velocity
    # Maximum linear acceleration to target while translating
    float32 target_linear_accel
    # Maximum angular velocity to target while rotating
    float32 target_angular_velocity
    # Maximum angular acceleration to target while rotating
    float32 target_angular_accel
    # Minimum distance margin to maintain away from obstacles
    float32 collision_distance
    
    # Specifies whether the Astrobee is allowed to fly blind i.e. not faceforward
    bool holonomic_enabled
    
    # Specifies whether the Astrobee is checking for obstacles
    bool check_obstacles
    
    # Specifies whether the Astrobee is checking the keepin and keepout zones
    bool check_zones
    
    # Specifies whether the Astrobee is allowed to auto return. Please note,
    # Astrobee will only use this flags when deciding if it should auto return. If
    # the astrobee gets a command to auto return from the operator or guest science,
    # it will auto return without checking this flag
    bool auto_return_enabled
    
    # Specifies whether the choreographer should execute a segment immediately or
    # based on the time stamp in the segement
    bool immediate_enabled
    
    # Specifies the current planner being used
    string planner
    
    # Specifies whether re-planning is allowed
    bool replanning_enabled
    
    # Specifies the current world being used
    string world
    
    # Number of seconds since Unix Epoch
    uint32 boot_time
    
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
    MSG: ff_msgs/OpState
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
    # Operating States, based off of the enumeration constants
    # in rapid::ext::astrobee::AgentState.
    #
    # *MUST* be kept in sync with the DDS IDL file in astrobee_common
    
    uint8 READY            = 0  # Freeflyer is ready to accept commands
    uint8 PLAN_EXECUTION   = 1  # Freeflyer is executing a plan
    uint8 TELEOPERATION    = 2  # Freeflyer is executing a teleop command
    uint8 AUTO_RETURN      = 3  # Freeflyer is autonomously returning to the dock
    # The freeflyer is either executing a fault response or there is a fault
    # occurring in the system that prevents the freeflyer from moving
    uint8 FAULT            = 4
    
    # Operating state
    uint8 state
    
    ================================================================================
    MSG: ff_msgs/ExecState
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
    # Execution States, based off of the enumeration constants in
    # rapid::ext::astrobee::AgentState
    #
    # *MUST* be kept in sync with the DDS IDL file in astrobee_common
    
    uint8 IDLE      = 0   # Process is idle
    uint8 EXECUTING = 1   # Process is executing
    uint8 PAUSED    = 2   # Process is paused
    uint8 ERROR     = 3   # Process encountered an error
    
    # Execution state
    uint8 state
    
    ================================================================================
    MSG: ff_msgs/MobilityState
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
    # Mobility states, based off the enumeration constants in
    # rapid::ext::astrobee::AgentState
    #
    # *MUST* be kept in sync with the DDS IDL file in astrobee_common
    
    uint8 DRIFTING        = 0   # Astrobee's propulsion is off
    uint8 STOPPING        = 1   # Astrobee is either stopping or stopped
    uint8 FLYING          = 2   # Astrobee is flying
    uint8 DOCKING         = 3   # Astrobee is either docking or undocking
    uint8 PERCHING        = 4   # Astrobee is either perching or unperching
    
    # Mobility state
    uint8 state
    
    # Specifies the progress of the action. For docking, this value can be N to -N
    # where N through 1 specifies the progress of a docking action, 0 is docked, and
    # -1 through -N specifies the process of an undocking action. For stopping, this
    # value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means
    # the robot is stopped. For perching, this value can be N to -N where N through
    # 1 specifies the progress of a perching action, 0 is perched, and -1 through
    # -N specifies the process of an unperching action.
    int32 sub_state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AgentStateStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.operating_state !== undefined) {
      resolved.operating_state = OpState.Resolve(msg.operating_state)
    }
    else {
      resolved.operating_state = new OpState()
    }

    if (msg.plan_execution_state !== undefined) {
      resolved.plan_execution_state = ExecState.Resolve(msg.plan_execution_state)
    }
    else {
      resolved.plan_execution_state = new ExecState()
    }

    if (msg.guest_science_state !== undefined) {
      resolved.guest_science_state = ExecState.Resolve(msg.guest_science_state)
    }
    else {
      resolved.guest_science_state = new ExecState()
    }

    if (msg.mobility_state !== undefined) {
      resolved.mobility_state = MobilityState.Resolve(msg.mobility_state)
    }
    else {
      resolved.mobility_state = new MobilityState()
    }

    if (msg.proximity !== undefined) {
      resolved.proximity = msg.proximity;
    }
    else {
      resolved.proximity = 0.0
    }

    if (msg.profile_name !== undefined) {
      resolved.profile_name = msg.profile_name;
    }
    else {
      resolved.profile_name = ''
    }

    if (msg.flight_mode !== undefined) {
      resolved.flight_mode = msg.flight_mode;
    }
    else {
      resolved.flight_mode = ''
    }

    if (msg.target_linear_velocity !== undefined) {
      resolved.target_linear_velocity = msg.target_linear_velocity;
    }
    else {
      resolved.target_linear_velocity = 0.0
    }

    if (msg.target_linear_accel !== undefined) {
      resolved.target_linear_accel = msg.target_linear_accel;
    }
    else {
      resolved.target_linear_accel = 0.0
    }

    if (msg.target_angular_velocity !== undefined) {
      resolved.target_angular_velocity = msg.target_angular_velocity;
    }
    else {
      resolved.target_angular_velocity = 0.0
    }

    if (msg.target_angular_accel !== undefined) {
      resolved.target_angular_accel = msg.target_angular_accel;
    }
    else {
      resolved.target_angular_accel = 0.0
    }

    if (msg.collision_distance !== undefined) {
      resolved.collision_distance = msg.collision_distance;
    }
    else {
      resolved.collision_distance = 0.0
    }

    if (msg.holonomic_enabled !== undefined) {
      resolved.holonomic_enabled = msg.holonomic_enabled;
    }
    else {
      resolved.holonomic_enabled = false
    }

    if (msg.check_obstacles !== undefined) {
      resolved.check_obstacles = msg.check_obstacles;
    }
    else {
      resolved.check_obstacles = false
    }

    if (msg.check_zones !== undefined) {
      resolved.check_zones = msg.check_zones;
    }
    else {
      resolved.check_zones = false
    }

    if (msg.auto_return_enabled !== undefined) {
      resolved.auto_return_enabled = msg.auto_return_enabled;
    }
    else {
      resolved.auto_return_enabled = false
    }

    if (msg.immediate_enabled !== undefined) {
      resolved.immediate_enabled = msg.immediate_enabled;
    }
    else {
      resolved.immediate_enabled = false
    }

    if (msg.planner !== undefined) {
      resolved.planner = msg.planner;
    }
    else {
      resolved.planner = ''
    }

    if (msg.replanning_enabled !== undefined) {
      resolved.replanning_enabled = msg.replanning_enabled;
    }
    else {
      resolved.replanning_enabled = false
    }

    if (msg.world !== undefined) {
      resolved.world = msg.world;
    }
    else {
      resolved.world = ''
    }

    if (msg.boot_time !== undefined) {
      resolved.boot_time = msg.boot_time;
    }
    else {
      resolved.boot_time = 0
    }

    return resolved;
    }
};

module.exports = AgentStateStamped;

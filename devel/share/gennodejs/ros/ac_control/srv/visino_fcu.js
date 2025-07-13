// Auto-generated. Do not edit!

// (in-package ac_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class visino_fcuRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vision_flag = null;
      this.fcu_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('vision_flag')) {
        this.vision_flag = initObj.vision_flag
      }
      else {
        this.vision_flag = false;
      }
      if (initObj.hasOwnProperty('fcu_flag')) {
        this.fcu_flag = initObj.fcu_flag
      }
      else {
        this.fcu_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type visino_fcuRequest
    // Serialize message field [vision_flag]
    bufferOffset = _serializer.bool(obj.vision_flag, buffer, bufferOffset);
    // Serialize message field [fcu_flag]
    bufferOffset = _serializer.bool(obj.fcu_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type visino_fcuRequest
    let len;
    let data = new visino_fcuRequest(null);
    // Deserialize message field [vision_flag]
    data.vision_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fcu_flag]
    data.fcu_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ac_control/visino_fcuRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0fabbef3999e74196457a8185716d089';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 客户端请求时发送的两个数字
    bool vision_flag  # 请求中包含一个布尔值，表示视觉标志位
    bool fcu_flag  # 请求中包含一个布尔值，表示飞控标志位
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new visino_fcuRequest(null);
    if (msg.vision_flag !== undefined) {
      resolved.vision_flag = msg.vision_flag;
    }
    else {
      resolved.vision_flag = false
    }

    if (msg.fcu_flag !== undefined) {
      resolved.fcu_flag = msg.fcu_flag;
    }
    else {
      resolved.fcu_flag = false
    }

    return resolved;
    }
};

class visino_fcuResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.vision_flag = null;
      this.fcu_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('vision_flag')) {
        this.vision_flag = initObj.vision_flag
      }
      else {
        this.vision_flag = false;
      }
      if (initObj.hasOwnProperty('fcu_flag')) {
        this.fcu_flag = initObj.fcu_flag
      }
      else {
        this.fcu_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type visino_fcuResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [vision_flag]
    bufferOffset = _serializer.bool(obj.vision_flag, buffer, bufferOffset);
    // Serialize message field [fcu_flag]
    bufferOffset = _serializer.bool(obj.fcu_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type visino_fcuResponse
    let len;
    let data = new visino_fcuResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [vision_flag]
    data.vision_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fcu_flag]
    data.fcu_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ac_control/visino_fcuResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '870dc7ec1852923b2539416414dfa0c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 服务器响应发送的数据
    bool success       # 响应中包含一个布尔值，表示操作是否成功
    bool vision_flag  # 响应中包含一个布尔值，表示视觉标志位
    bool fcu_flag  # 响应中包含一个布尔值，表示飞控标志位
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new visino_fcuResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.vision_flag !== undefined) {
      resolved.vision_flag = msg.vision_flag;
    }
    else {
      resolved.vision_flag = false
    }

    if (msg.fcu_flag !== undefined) {
      resolved.fcu_flag = msg.fcu_flag;
    }
    else {
      resolved.fcu_flag = false
    }

    return resolved;
    }
};

module.exports = {
  Request: visino_fcuRequest,
  Response: visino_fcuResponse,
  md5sum() { return '5e11117ad059078a9af4671921a90b5c'; },
  datatype() { return 'ac_control/visino_fcu'; }
};

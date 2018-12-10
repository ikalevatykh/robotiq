// Copyright (c) 2016, Toyota Research Institute. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "robotiq_3f_gripper_control/robotiq_3f_gripper_modbus_client.h"

using namespace robotiq_3f_gripper_control;

Robotiq3FGripperModbusClient::Robotiq3FGripperModbusClient(const std::string& ip_address)
    : ip_address_(ip_address)
{
    modbus_ = modbus_new_tcp(ip_address.c_str(), 502);
}

Robotiq3FGripperModbusClient::~Robotiq3FGripperModbusClient()
{
    modbus_close(modbus_);
    modbus_free(modbus_);
}

void Robotiq3FGripperModbusClient::init(ros::NodeHandle nh)
{
    int rc = modbus_connect(modbus_);
    if (rc == -1) {
        ROS_ERROR_STREAM("MODBUS connection failed: " << modbus_strerror(errno));
        throw std::runtime_error(modbus_strerror(errno));
    }        
}

void Robotiq3FGripperModbusClient::writeOutputs(const GripperOutput &output)
{
    const uint8_t map[] = {
        output.rACT + (output.rMOD << 1) + (output.rGTO << 3) + (output.rATR << 4),
        output.rGLV + (output.rICF << 2) + (output.rICS << 3),
        0,
        output.rPRA,
        output.rSPA,
        output.rFRA,
        output.rPRB,
        output.rSPB,
        output.rFRB,
        output.rPRC,
        output.rSPC,
        output.rFRC,
        output.rPRS,
        output.rSPS,
        output.rFRS,
        0
    };

    int rc = modbus_write_registers(modbus_, 0, 8, (const uint16_t *) map);
    if (rc == -1) {
        ROS_ERROR_STREAM("MODBUS error: " << modbus_strerror(errno));
        throw std::runtime_error(modbus_strerror(errno));
    }    
    output_ = output;
}

Robotiq3FGripperModbusClient::GripperInput Robotiq3FGripperModbusClient::readInputs() const
{
    uint8_t map[16];

    int rc = modbus_read_registers(modbus_, 0, 8, (uint16_t *) map);
    if (rc == -1) {
        ROS_ERROR_STREAM("MODBUS error: " << modbus_strerror(errno));
        throw std::runtime_error(modbus_strerror(errno));
    }

    GripperInput input;

    // Gripper Status
    input.gACT = map[0] & 0x1;
    input.gMOD = (map[0] >> 0x1) & 0x3;
    input.gGTO = (map[0] >> 0x3) & 0x1;
    input.gIMC = (map[0] >> 0x4) & 0x3;
    input.gSTA = (map[0] >> 0x6) & 0x3;

    // Object Status
    input.gDTA = map[1] & 0x3;
    input.gDTB = (map[1] >> 0x2) & 0x3;
    input.gDTC = (map[1] >> 0x4) & 0x3;
    input.gDTS = (map[1] >> 0x6) & 0x3;

    // Fault Status
    input.gFLT = map[2] & 0xF;

    // Requested Position, Speed and Force (Finger A).
    input.gPRA = map[3];
    input.gPOA = map[4];
    input.gCUA = map[5];

    // Finger B
    input.gPRB = map[6];
    input.gPOB = map[7];
    input.gCUB = map[8];

    // Finger C
    input.gPRC = map[9];
    input.gPOC = map[10];
    input.gCUC = map[11];

    // Scissor Mode
    input.gPRS = map[12];
    input.gPOS = map[13];
    input.gCUS = map[14];

    return input;    
}

Robotiq3FGripperModbusClient::GripperOutput Robotiq3FGripperModbusClient::readOutputs() const
{
    return output_;
}

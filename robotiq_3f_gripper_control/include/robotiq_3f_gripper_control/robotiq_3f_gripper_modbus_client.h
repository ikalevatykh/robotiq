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

#ifndef ROBOTIQ_3F_GRIPPER_MODBUS_CLIENT_H
#define ROBOTIQ_3F_GRIPPER_MODBUS_CLIENT_H

#include <robotiq_3f_gripper_control/robotiq_3f_gripper_client_base.h>
#include <modbus/modbus.h>

namespace robotiq_3f_gripper_control
{

/**
 * \brief This class provides a client for the EtherCAT manager object that
 *        can translate robot input/output messages and translate them to
 *        the underlying IO Map.
 */

class Robotiq3FGripperModbusClient : public Robotiq3FGripperClientBase
{
public:
    Robotiq3FGripperModbusClient(const std::string& ip_address);

    virtual ~Robotiq3FGripperModbusClient();


    void init(ros::NodeHandle nh);

    /**
     * \brief Write the given set of control flags to the memory of the gripper
     *
     * @param[in] output The set of output-register values to write to the gripper
     */
    void writeOutputs(const GripperOutput& output);

    /**
     * \brief Reads set of input-register values from the gripper.
     * \return The gripper input registers as read from the controller IOMap
     */
    GripperInput readInputs() const;

    /**
     * \brief Reads set of output-register values from the gripper.
     * \return The gripper output registers as read from the controller IOMap
     */
    GripperOutput readOutputs() const;

private:
    std::string ip_address_;
    modbus_t *modbus_;

    GripperInput input_;
    GripperOutput output_;
};

} //end namespace robotiq_3f_gripper_control

#endif // ROBOTIQ_3F_GRIPPER_MODBUS_CLIENT_H

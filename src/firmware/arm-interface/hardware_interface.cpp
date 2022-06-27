#include "snowbots_arm_hardware_drivers/TeensyDriver.h"

    // Joint Space Mode Hardware Driver Functions

    void TeensyDriver::jointSpaceMotion(int axis, char dir)
    {
        std::string outMsg = "JM";
        outMsg += "M"
        outMsg += dir
        outMsg += std::to_string(axis);
        outMsg += "\n";
        sendMsg(outMsg);
    }

    void TeensyDriver::changeSpeed(char dir)
    {
        std::string outMsg = "JM";
        outMsg = "S";
        outMsg += dir;
        outMsg += "\n";
        sendMsg(outMsg);
    }

    void TeensyDriver::changeAxis(char axis)
    {
        std::string outMsg = "JM";
        outMsg += "A";
        outMsg += axis;
        outMsg += "\n";
        sendMsg(outMsg);
    }

    void TeensyDriver::releaseAxis(char axis)
    {
        std::string outMsg = "JM";
        outMsg += "R";
        outMsg += axis;
        outMsg += "\n";
        sendMsg(outMsg);
    }


    // End Effector Hardware Driver Functions

    void TeensyDriver::endEffector(char dir)
    {
        std::string outMsg = "EE";
        outMsg += dir;
        outMsg += "\n";
        sendMsg(outMsg);
    }

    void TeensyDriver::getEndEffectorForce()
    {
        std::string outMsg = "EEF\n";
        sendMsg(outMsg);
    }


    // Drilling Mode Hardware Driver Functions

    void TeensyDriver::prepareDrilling()
    {
        std::string outMsg = "DMP\n";
        sendMsg(outMsg);
    }

    void TeensyDriver::collectSample()
    {
        std::string outMsg = "DMC\n";
        sendMsg(outMsg);
    }

    void TeensyDriver::depositSample()
    {
        std::string outMsg = "DMD\n";
        sendMsg(outMsg);
    }

    void TeensyDriver::manualDrill(char dir)
    {
        std::string outMsg = "DM";
        outMsg += dir;
        outMsg += "\n";
        sendMsg(outMsg);
    }


    // Cartesian Mode Hardware Driver Functions

    void TeensyDriver::getArmPosition()
    {
        std::string outMsg = "GP\n";
        sendMsg(outMsg);
    }

    void TeensyDriver::cartesianPositionUpdate(std::vector<double>& pos_commands, std::vector<double>& joint_positions)
    {
        
        // get updated position commands
        jointPosToEncSteps(pos_commands, enc_commands_);

        // construct update message
        std::string outMsg = "CM";
        for (int i = 0; i < num_joints_; ++i)
        {
            outMsg += 'A' + i;
            outMsg += std::to_string(enc_commands_[i]);
        }
        outMsg += "\n";

        // run the communication with board
        sendMsg(outMsg);

        // return updated joint_positions
        encStepsToJointPos(enc_steps_ , joint_positions);

    }

    void TeensyDriver::getJointPositions(std::vector<double>& joint_positions)
    {
        // get current joint positions
        std::string outMsg = "JP\n";
        sendMsg(outMsg);
        encStepsToJointPos(enc_steps_, joint_positions);
    }

    void TeensyDriver::encStepsToJointPos(std::vector<int>& enc_steps, std::vector<double>& joint_positions)
    {
        for (int i = 0; i < enc_steps.size(); ++i)
        {
            // convert enc steps to joint deg
            joint_positions[i] = static_cast<double>(enc_steps[i]) / enc_steps_per_deg_[i];
        }
    }

    void TeensyDriver::jointPosToEncSteps(std::vector<double>& joint_positions, std::vector<int>& enc_steps)
    {
        for (int i = 0; i < joint_positions.size(); ++i)
        {
            // convert joint deg to enc steps
            enc_steps[i] = static_cast<int>(joint_positions[i] * enc_steps_per_deg_[i]);
        }
    }


    // Serial Communication Protocols

    void sendMsg(std::string outMsg)
    {

    }

    void recieveMsg(std::string inMsg)
    {
        
    }

    void initCommunication()
    {

    }
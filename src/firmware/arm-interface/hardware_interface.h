#include <vector>
#include "math.h"
#include "time.h"
#include <ros/ros.h>

class TeensyDriver 
{
    public:
        void jointSpaceMotion(int axis, char dir);
        void changeSpeed(char dir);
        void changeAxis(char axis);
        void releaseAxis(char axis);
        void getEndEffectorForce();
        void prepareDrilling();
        void collectSample();
        void depositSample();
        void manualDrill(char dir);
        void getArmPosition();
        void cartesianPositionUpdate(std::vector<double>& pos_commands, std::vector<double>& joint_positions);

    private:
        int num_joints_;
        std::vector<int> enc_commands_;
        std::vector<int> enc_steps_;
        std::vector<double> enc_steps_per_deg_;

        // Comms with teensy
        void sendMsg(std::string outMsg);
        void recieveMsg(std::string inMsg);
        void initCommunication();

        // Convert between encoder steps and joint angle degrees
        void encStepsToJointPos(std::vector<int>& enc_steps, std::vector<double>& joint_positions);
        void jointPosToEncSteps(std::vector<double>& joint_positions, std::vector<int>& enc_steps);
}
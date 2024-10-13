#ifndef HEXAPODCONTROL_H
#define HEXAPODCONTROL_H

#ifdef USE_SIMULATOR
    #include "../modules/custom/raisimSimulatorFull/raisimSimulator.h"
    #include <raisim/Path.hpp>  // Include Path from raisim only if simulator is enabled
#endif

#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


using namespace std;

class HexapodControl
{
public:
    #ifdef USE_SIMULATOR
        HexapodControl(unsigned int id, unique_ptr<ArduinoController> arduino, RSTimedLoop& rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep, Path binaryPath);
    #else
        HexapodControl(unsigned int id, unique_ptr<ArduinoController> arduino, RSTimedLoop& rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep);
    #endif
    ~HexapodControl();

    Eigen::MatrixXd getJacobian(int legNum) const;

    void updatePos();
    Eigen::VectorXd doFK(Eigen::VectorXd angs);
    void printPos() const;

    bool active;
    int operationDuration = 0; 

    // Instant set position
    void setAngs(const Eigen::VectorXd& angs);

    // Velocity set position
    void moveLegToPos(const Eigen::Vector3d& desiredPos, const int legNum);
    void moveLegsToPos(const Eigen::VectorXd& desiredPos, float dur);


    // Instant positions
    void jumpToZero();
    void jumpToBasic();
    void jumpToOff();
    void jumpToCurled();

    // Velocity positions
    void stand();
    void moveToStand(float dur);
    void moveToOff();

    // Velocity Path
    void jacobianTest(const int &style);
    void walk(double, double);

    bool directed = 0;

    int id;
    Eigen::VectorXd pos;
    Eigen::VectorXd currentAngles;
    Eigen::VectorXd currentAngularVelocities;
    Eigen::VectorXd desiredAngles;

    double moveVectorMag;
    RSTimedLoop& rsLoop;
    
    #ifdef USE_SIMULATOR
        unique_ptr<RaisimSimulator> simulator;  // Simulator object
    #endif

    double minStepDuration = 2000;
    double jacDuration = 5000;
    double standDuration = 1000;

private:
    float rsStep;
    void sendAngs();
    
    unique_ptr<ArduinoController> arduino;
    

    bool arduinoConnected;

    // constexpr static float coxaX = 0.044925;
    // constexpr static float coxaZ = 0.01065;
    // constexpr static float femurX = 0.118314;
    // constexpr static float tibiaX = 0.221426;

    /********* MATHEMATICAL MODEL VARIABLES */

    constexpr static float coxaX = 0.044924;
    constexpr static float coxaZ = 0.010656;
    constexpr static float femurX = 0.118313;
    constexpr static float tibiaX = 0.221170;

    // const vector<int> angleInits = {96, 94, 17};

    const vector<int> angleInits = {90, 45, 17, 90, 45, 17, 90, 45, 17, 90, 45, 17, 90, 45, 17, 90, 45, 17};

    const vector<float> bodyLegOffsets = {0.180, 0.130, 0.180, 0.180, 0.130, 0.180};
    const vector<float> femurRotation= {M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2};

    const vector<double> bodyLegBaseAngles = {57*M_PI/180.0, 0*M_PI/180, -57*M_PI/180, -(180-57)*M_PI/180, 180*M_PI/180, (180-57)*M_PI/180};
    int diagLegAngle = 57;   // Body -> Leg = 57 deg
    const vector<double> bodyLegAngles = {diagLegAngle*M_PI/180.0, 0*M_PI/180, -diagLegAngle*M_PI/180, -(180-diagLegAngle)*M_PI/180, 180*M_PI/180, (180-diagLegAngle)*M_PI/180};

    /********* GAIT CONTROL VARIABLES */

    double stepRadius = 0.08;
    double stepHeight = 0.08;

    // OG Z Stand Value = -0.113248
    float vertStandVal = 0.14;
    float horiDiagStandVal = 0.25255;
    float horiStraightStandVal = 0.232555;

    const vector<double> legStepOffsets = {0.25255, 0.25255, 0.25255, 0.232555, 0.25255, 0.232555};

    double standPos[18] = {
        bodyLegOffsets[0]*cos(bodyLegBaseAngles[0]) + legStepOffsets[0]*cos(bodyLegAngles[0]), bodyLegOffsets[0]*sin(bodyLegBaseAngles[0]) + legStepOffsets[0]*sin(bodyLegAngles[0]), -vertStandVal,
        bodyLegOffsets[1]*cos(bodyLegBaseAngles[1]) + legStepOffsets[1]*cos(bodyLegAngles[1]), bodyLegOffsets[1]*sin(bodyLegBaseAngles[1]) + legStepOffsets[1]*sin(bodyLegAngles[1]), -vertStandVal,
        bodyLegOffsets[2]*cos(bodyLegBaseAngles[2]) + legStepOffsets[2]*cos(bodyLegAngles[2]), bodyLegOffsets[2]*sin(bodyLegBaseAngles[2]) + legStepOffsets[2]*sin(bodyLegAngles[2]), -vertStandVal,
        bodyLegOffsets[3]*cos(bodyLegBaseAngles[3]) + legStepOffsets[3]*cos(bodyLegAngles[3]), bodyLegOffsets[3]*sin(bodyLegBaseAngles[3]) + legStepOffsets[3]*sin(bodyLegAngles[3]), -vertStandVal,
        bodyLegOffsets[4]*cos(bodyLegBaseAngles[4]) + legStepOffsets[4]*cos(bodyLegAngles[4]), bodyLegOffsets[4]*sin(bodyLegBaseAngles[4]) + legStepOffsets[4]*sin(bodyLegAngles[4]), -vertStandVal,
        bodyLegOffsets[5]*cos(bodyLegBaseAngles[5]) + legStepOffsets[5]*cos(bodyLegAngles[5]), bodyLegOffsets[5]*sin(bodyLegBaseAngles[5]) + legStepOffsets[5]*sin(bodyLegAngles[5]), -vertStandVal
        };

    double lastAngle = 0;
    vector<bool> standing = {0,1,0,1,0,1};
};

#endif
#ifndef HEXAPODCONTROL_H
#define HEXAPODCONTROL_H

#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "../modules/custom/raisimSimulator/raisimSimulator.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


using namespace std;

class HexapodControl
{
public:
    HexapodControl(unsigned int id, unique_ptr<ArduinoController> arduino, RSTimedLoop& rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep, Path binaryPath);
    ~HexapodControl();

    Eigen::MatrixXd getJacobian(int legNum) const;

    void updatePos();
    void printPos() const;

    void setAngs(const Eigen::VectorXd& angs);

    void moveLegToPos(const Eigen::Vector3d& desiredPos, const int legNum);
    void moveLegsToPos(const Eigen::VectorXd& desiredPos);


    void moveToZero();
    void moveToBasic();
    void moveToOff();
    void moveToCurled();

    void stand();

    int id;
    Eigen::VectorXd pos;
    Eigen::VectorXd currentAngles;
    Eigen::Vector3d currentAngularVelocities;

private:
    float rsStep;
    void sendAngs();
    
    unique_ptr<ArduinoController> arduino;
    RSTimedLoop& rsLoop;

    bool arduinoConnected;

    unique_ptr<RaisimSimulator> simulator;

    constexpr static float coxaX = 0.044925;
    constexpr static float coxaZ = 0.01065;
    constexpr static float femurX = 0.118314;
    constexpr static float tibiaX = 0.221426;

    const vector<int> angleInits = {96, 94, 17};

    const vector<float> bodyLegOffsets = {0.180, 0.130, 0.180, 0.180, 0.130, 0.180};
    const vector<float> femurRotation= {M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, -M_PI_2};
    const vector<float> bodyLegAngles = {57*M_PI/180, 0*M_PI/180, -57*M_PI/180, -123*M_PI/180, 180*M_PI/180, 123*M_PI/180};

    const vector<bool> legStatus = {0,1,0,1,0,1};
};

#endif
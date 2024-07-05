#ifndef HEXAPODLEG_H
#define HEXAPODLEG_H

#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "../modules/custom/raisimSimulator/raisimSimulator.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


using namespace std;

class HexapodLeg
{
public:
    HexapodLeg(unsigned int id, unique_ptr<ArduinoController> arduino, RSTimedLoop& rsLoop, bool simulationMode, bool raisimSimulator, float rsStep, Path binaryPath);
    ~HexapodLeg();

    Eigen::MatrixXd getJacobian() const;
    Eigen::Vector3d doIK(float x, float y, float z) const;
    Eigen::Vector3d doFK() const;

    void setAngs(float coxa, float femur, float tibia);
    void setAngs(const Eigen::Vector3d& angs);
    void moveToPos(float x, float y, float z);
    void moveToPos(const Eigen::Vector3d& pos);

    void moveToZero();
    void moveToBasic();
    void moveToOff();

    void doJacobianTest(const int &style);
    void doIKTest();

    int id;
    Eigen::Vector3d pos;
    Eigen::Vector3d currentAngles;
    Eigen::Vector3d currentAngularVelocities;

private:
    float rsStep;
    void sendAngs();
    void sendPos(float x, float y, float z);
    
    unique_ptr<ArduinoController> arduino;
    RSTimedLoop& rsLoop;

    bool simulationMode;

    unique_ptr<RaisimSimulator> simulator;

    constexpr static float coxaX = 44.925;
    constexpr static float coxaZ = 10.650;
    constexpr static float femurX = 118.314;
    constexpr static float tibiaX = 221.426;

    constexpr static int coxaAngleOffset = 6;
    constexpr static int femurAngleOffset = 0;
    constexpr static int tibiaAngleOffset = -73;

    constexpr static int coxaAngleInit = 96;
    constexpr static int femurAngleInit = 94;
    constexpr static int tibiaAngleInit = 17;

    constexpr static int centreLegOffset = 125;
    constexpr static int diagLegOffset = 150;
    constexpr static int diagLegOffsetAngle = 57;
};

#endif
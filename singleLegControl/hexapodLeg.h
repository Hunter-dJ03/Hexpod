#ifndef HEXAPODLEG_H
#define HEXAPODLEG_H

#include "arduinoController.h"
#include "rs_timed_loop.h"
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


using namespace std;

class HexapodLeg
{
public:
    HexapodLeg(unsigned int id, ArduinoController& arduino, RSTimedLoop& rsLoop, bool simulationMode);
    ~HexapodLeg();
    void setAngs(float coxa, float femur, float tibia);
    void setAngs(const Eigen::Vector3d& angs);


    Eigen::MatrixXd getInverseJacobian();

    Eigen::Vector3d doIK(float x, float y, float z);
    Eigen::Vector3d doFK(float coxa, float femur, float tibia);

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

private:
    void sendAngs();
    void sendPos(float x, float y, float z);

    float constrain(float x, float a, float b);

    ArduinoController& arduino;
    RSTimedLoop& rsLoop;

    bool simulationMode;

    float coxaX = 44.925;
    float coxaZ = 10.650;
    float femurX = 118.314;
    float tibiaX = 221.426;

    int coxaAngleOffset = 6;
    int femurAngleOffset = 0;   // 6
    int tibiaAngleOffset = -73; //-65

    int coxaAngleInit = 96;
    int femurAngleInit = 94; // 6
    int tibiaAngleInit = 17; //-65

    int centreLegOffset = 125;
    int diagLegOffset = 150;
    int diagLegOffsetAngle = 57;
};

#endif
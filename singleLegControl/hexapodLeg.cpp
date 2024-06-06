#include "hexapodLeg.h"
#include "rs_timed_loop.h"

#include <iostream>
#include <fmt/core.h>
#include <math.h>
#include <chrono>
#include <thread>

using namespace std;
using namespace this_thread;     // sleep_for, sleep_until
using chrono::system_clock;

HexapodLeg::HexapodLeg(unsigned int _id, ArduinoController &arduino, RSTimedLoop &rsLoop) : arduino(arduino), rsLoop(rsLoop)
{
    id = _id;

    moveToZero();
}

HexapodLeg::~HexapodLeg()
{
}

Eigen::MatrixXd HexapodLeg::getInverseJacobian()
{
    Eigen::MatrixXd Jac(3,3);

    Jac(0,0) = -sin(currentAngles[0])*(0.221426*cos(currentAngles[1] + currentAngles[2]) + 0.1183145*cos(currentAngles[1]) + 0.044925);
    Jac(0,1) = -cos(currentAngles[0])*(0.221426*sin(currentAngles[1] + currentAngles[2]) + 0.1183145*sin(currentAngles[1]));
    Jac(0,2) = -0.221426*sin(currentAngles[1] + currentAngles[2])*cos(currentAngles[0]);
    Jac(1,0) = cos(currentAngles[0])*(0.221426*cos(currentAngles[1] + currentAngles[2]) + 0.1183145*cos(currentAngles[1]) + 0.044925);
    Jac(1,1) = -sin(currentAngles[0])*(0.221426*sin(currentAngles[1] + currentAngles[2]) + 0.1183145*sin(currentAngles[1]));
    Jac(1,2) = -0.221426*sin(currentAngles[1] + currentAngles[2])*sin(currentAngles[0]);
    Jac(2,0) = 0;
    Jac(2,1) = 0.221426*cos(currentAngles[1] + currentAngles[2]) + 0.1183145*cos(currentAngles[1]);
    Jac(2,2) = 0.221426*cos(currentAngles[1] + currentAngles[2]);

    return Jac;
}

// Perform inverse kinematics to get desired control engles
Eigen::Vector3d HexapodLeg::doIK(float x, float y, float z)
{

    // cout<<endl<<x<<","<<y<<","<<z<<","<<endl;

    float dx = x;
    float dy = y;
    float dz = z - coxaZ;

    // cout<<endl<<dx<<","<<dy<<","<<dz<<endl;

    float H = sqrt(pow(x, 2) + pow(y, 2)) - coxaX;
    float L = sqrt(pow(H, 2) + pow(dz, 2));

    // cout<<endl<<H<<","<<L<<","<<endl;

    float omega = atan2(dz, H);
    float beta = acos(constrain((pow(femurX, 2) + pow(L, 2) - pow(tibiaX, 2)) / (2 * femurX * L), -1, 1));
    float phi = acos(constrain((pow(femurX, 2) + pow(tibiaX, 2) - pow(L, 2)) / (2 * femurX * tibiaX), -1, 1));

    // cout<<endl<<omega<<","<<beta<<","<<phi<<","<<endl;

    float th1 = atan2(y, x);
    float th2 = beta + omega;
    float th3 = M_PI - phi;

    Eigen::Vector3d angs = {th1, th2, th3};

    return angs*180/M_PI;
}

// Perform forward kinematics of the hexapod leg to get end affector position
Eigen::Vector3d HexapodLeg::doFK(float coxa, float femur, float tibia)
{
    Eigen::Vector3d pos = {
        cos(coxa) * (0.221426 * cos(femur + tibia) + 0.1183145 * cos(femur) + 0.044925),
        sin(coxa) * (0.221426 * cos(femur + tibia) + 0.1183145 * cos(femur) + 0.044925),
        0.221426 * sin(femur + tibia) + 0.1183145 * sin(femur) + 0.01065};

    return pos;
}

// Move to straight leg position
void HexapodLeg::moveToZero()
{
    setAngs(0, 0, 0);
}
// Move to basic standing position
void HexapodLeg::moveToBasic()
{
    setAngs(0, 40, 102);
}
// Move to position that should fold back past limit when power disabled
void HexapodLeg::moveToOff()
{
    setAngs(0, 90, 163);
}

void HexapodLeg::doJacobianTest(const int &style)
{
    float radius = 0.05;  // meters
    double angular_velocity = 0.5 * M_PI;   // HZ
    float testingJac = 0;
    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d desiredAngularVelocities;
    // Eigen::Matrix3d Jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;
    // long currentTime;

    while (testingJac <= 1000 / angular_velocity * M_PI) {
        // currentTime = chrono::high_resolution_clock::now().time_since_epoch().count();

        desiredSpatialVelocity << 0,
            -radius * angular_velocity * cos(angular_velocity * (testingJac)*0.001),
            radius * angular_velocity * sin(angular_velocity * (testingJac)*0.001);

        jacobianPseudoInverse = getInverseJacobian();
        desiredAngularVelocities = jacobianPseudoInverse * desiredSpatialVelocity;

        testingJac++;

        rsLoop.realTimeDelay();
    }
}

// Test the IK position control in X Y Z axis individually
void HexapodLeg::doIKTest()
{
    // Set Start Position
    Eigen::Vector3d posik = {220, 0, -200};
    moveToPos(posik);

    sleep_for(chrono::milliseconds(2000));

    // Set interpolation scale
    int scale = 160;

    // Test X movement
    for (int i = 1; i <= scale; i++)
    {
        posik[0] += 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[0] -= 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    for (int i = 1; i <= scale; i++)
    {
        posik[0] += 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    sleep_for(chrono::milliseconds(2000));

    // Test Y movement
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[1] += 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    for (int i = 1; i <= scale * 4; i++)
    {
        posik[1] -= 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[1] += 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    sleep_for(chrono::milliseconds(2000));

    // Test Z Movement
    for (int i = 1; i <= scale; i++)
    {
        posik[2] += 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[2] -= 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    for (int i = 1; i <= scale; i++)
    {
        posik[2] += 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }
    sleep_for(chrono::milliseconds(2000));
}

// Set Angles Overload for 3 individual angle input values
void HexapodLeg::setAngs(float coxa, float femur, float tibia)
{
    currentAngles(0) = coxa;
    currentAngles(1) = femur;
    currentAngles(2) = tibia;
    sendAngs();
}
// Set Angles Overload for 1x3 eigen vector
void HexapodLeg::setAngs(const Eigen::Vector3d& angs)
{
    currentAngles = angs;
    sendAngs();
}
// Send the angles of the servo motors to the arduino
void HexapodLeg::sendAngs()
{
    cout << fmt::format("setAngs({}|{}/{})\n", currentAngles[0], currentAngles[1], currentAngles[2]);
    arduino.sendCommand(fmt::format("setAngs({}|{}/{})\r", currentAngles[0], currentAngles[1], currentAngles[2]));
}

// Move to Position Overload for 3 individual position input values
void HexapodLeg::moveToPos(float x, float y, float z) {
    sendPos(x,y,z);
};
// Move to Position Overload for 1x3 eigen vector
void HexapodLeg::moveToPos(const Eigen::Vector3d& pos) {
    // cout <<endl<< pos <<endl;
    sendPos(pos[0],pos[1],pos[2]);
}
// Move to a desired 3D coordinate
void HexapodLeg::sendPos(float x, float y, float z)
{
    // cout<<endl<<x<<","<<y<<","<<z<<","<<endl;

    // Get Angles for set position
    Eigen::Vector3d angs = doIK(x, y, z);

    // cout <<endl << angs <<endl;

    // Move to angles
    setAngs(angs);
}

// Utility function to constrain a value to a range with set max and min
float HexapodLeg::constrain(float val, float min, float max)
{
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

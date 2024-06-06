#include "hexapodLeg.h"
#include <iostream>
#include <fmt/core.h>
#include <math.h>
#include <chrono>
#include <thread>

using namespace std;
using namespace this_thread;     // sleep_for, sleep_until
using chrono::system_clock;

HexapodLeg::HexapodLeg(unsigned int _id, ArduinoController &arduino) : arduino(arduino)
{
    id = _id;

    moveToZero();
}

HexapodLeg::~HexapodLeg()
{
}

Eigen::MatrixXd HexapodLeg::getInverseJacobian(float coxa, float femur, float tibia)
{
    Eigen::MatrixXd Jac(3,3);

    Jac(0,0) = -sin(coxa)*(0.221426*cos(femur + tibia) + 0.1183145*cos(femur) + 0.044925);
    Jac(0,1) = -cos(coxa)*(0.221426*sin(femur + tibia) + 0.1183145*sin(femur));
    Jac(0,2) = -0.221426*sin(femur + tibia)*cos(coxa);
    Jac(1,0) = cos(coxa)*(0.221426*cos(femur + tibia) + 0.1183145*cos(femur) + 0.044925);
    Jac(1,1) = -sin(coxa)*(0.221426*sin(femur + tibia) + 0.1183145*sin(femur));
    Jac(1,2) = -0.221426*sin(femur + tibia)*sin(coxa);
    Jac(2,0) = 0;
    Jac(2,1) = 0.221426*cos(femur + tibia) + 0.1183145*cos(femur);
    Jac(2,2) = 0.221426*cos(femur + tibia);

    return Jac;
}

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

Eigen::Vector3d HexapodLeg::doFK(float coxa, float femur, float tibia)
{
    Eigen::Vector3d pos = {
        cos(coxa) * (0.221426 * cos(femur + tibia) + 0.1183145 * cos(femur) + 0.044925),
        sin(coxa) * (0.221426 * cos(femur + tibia) + 0.1183145 * cos(femur) + 0.044925),
        0.221426 * sin(femur + tibia) + 0.1183145 * sin(femur) + 0.01065};

    return pos;
}

void HexapodLeg::moveToZero()
{
    setAngs(0, 0, 0);
}

void HexapodLeg::moveToBasic()
{
    setAngs(0, 40, 102);
}

void HexapodLeg::moveToOff()
{
    setAngs(0, 90, 163);
}

void HexapodLeg::doJacobianTest(const int &style)
{
}

void HexapodLeg::doIKTest()
{
    Eigen::Vector3d posik = {220, 0, -200};

    cout << posik <<endl;

    moveToPos(posik);


    sleep_for(chrono::milliseconds(2000));

    int scale = 160;

    for (int i = 1; i <= scale; i++)
    {
        cout<<i;
        posik[0] += 0.5;
        moveToPos(posik);
        sleep_for(chrono::milliseconds(1));
    }

    for (int i = 1; i <= scale * 2; i++)
    {
        cout<<i;
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
    setAngsOverload(coxa,femur,tibia);
}
// Set Angles Overload for 1x3 eigen vector
void HexapodLeg::setAngs(const Eigen::Vector3d& angs)
{
    setAngsOverload(angs[0],angs[1],angs[2]);
}
// Send the angles of the servo motors to the arduino
void HexapodLeg::setAngsOverload(float coxa, float femur, float tibia)
{
    cout << fmt::format("setAngs({}|{}/{})\n", coxa, femur, tibia);
    arduino.sendCommand(fmt::format("setAngs({}|{}/{})\r", coxa, femur, tibia));
}

// Move to Position Overload for 3 individual position input values
void HexapodLeg::moveToPos(float x, float y, float z) {
    moveToPosOverload(x,y,z);
};
// Move to Position Overload for 1x3 eigen vector
void HexapodLeg::moveToPos(const Eigen::Vector3d& pos) {
    // cout <<endl<< pos <<endl;
    moveToPosOverload(pos[0],pos[1],pos[2]);
}
// Move to a desired 3D coordinate
void HexapodLeg::moveToPosOverload(float x, float y, float z)
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

#include "hexapodLeg.h"
#include "rs_timed_loop.h"

#include "matplotlibcpp.h"

#include <iostream>
#include <fmt/core.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <cmath>

using namespace std;
namespace plt = matplotlibcpp;
using namespace this_thread;     // sleep_for, sleep_until
using chrono::system_clock;

HexapodLeg::HexapodLeg(unsigned int id, ArduinoController &arduino, RSTimedLoop &rsLoop, bool simulationMode, float rsStep) : id(id), arduino(arduino), rsLoop(rsLoop), simulationMode(simulationMode), rsStep(rsStep)
{
    // moveToZero();
}

HexapodLeg::~HexapodLeg()
{
}

Eigen::MatrixXd HexapodLeg::getJacobian()
{
    Eigen::MatrixXd Jac(3,3);

    // float tibia = -currentAngles[2]+2*M_PI;x    

    // cout << "Tibia: " <<tibia <<endl;

    // Jac(0,0) = -sin(currentAngles[0])*(0.221426*cos(currentAngles[1] - currentAngles[2]) + 0.1183145*cos(currentAngles[1]) + 0.044925);
    // Jac(0,1) = -cos(currentAngles[0])*(0.221426*sin(currentAngles[1] - currentAngles[2]) + 0.1183145*sin(currentAngles[1]));
    // Jac(0,2) = 0.221426*sin(currentAngles[1] - currentAngles[2])*cos(currentAngles[0]);
    // Jac(1,0) = cos(currentAngles[0])*(0.221426*cos(currentAngles[1] - currentAngles[2]) + 0.1183145*cos(currentAngles[1]) + 0.044925);
    // Jac(1,1) = -sin(currentAngles[0])*(0.221426*sin(currentAngles[1] - currentAngles[2]) + 0.1183145*sin(currentAngles[1]));
    // Jac(1,2) = 0.221426*sin(currentAngles[1] - currentAngles[2])*sin(currentAngles[0]);
    // Jac(2,0) = 0;
    // Jac(2,1) = 0.221426*cos(currentAngles[1] - currentAngles[2]) + 0.1183145*cos(currentAngles[1]);
    // Jac(2,2) = -0.221426*cos(currentAngles[1] - currentAngles[2]);

    Jac(0,0) = -sin(currentAngles[0])*(0.221426*cos(currentAngles[1] + currentAngles[2]) + 0.1183145*cos(currentAngles[1]) + 0.044925);
    Jac(0,1) = -cos(currentAngles[0])*(0.221426*sin(currentAngles[1] + currentAngles[2]) + 0.1183145*sin(currentAngles[1]));
    Jac(0,2) = -0.221426*sin(currentAngles[1] + currentAngles[2])*cos(currentAngles[0]);
    Jac(1,0) = cos(currentAngles[0])*(0.221426*cos(currentAngles[1] + currentAngles[2]) + 0.1183145*cos(currentAngles[1]) + 0.044925);
    Jac(1,1) = -sin(currentAngles[0])*(0.221426*sin(currentAngles[1] + currentAngles[2]) + 0.1183145*sin(currentAngles[1]));
    Jac(1,2) = -0.221426*sin(currentAngles[1] + currentAngles[2])*sin(currentAngles[0]);
    Jac(2,0) = 0;
    Jac(2,1) = 0.221426*cos(currentAngles[1] + currentAngles[2]) + 0.1183145*cos(currentAngles[1]);
    Jac(2,2) = 0.221426*cos(currentAngles[1] + currentAngles[2]);


    // cout << "Jac" << endl << Jac << endl;

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
    float th3 = M_PI + phi;

    Eigen::Vector3d angs = {th1, th2, th3};

    return angs;
}

// Perform forward kinematics of the hexapod leg to get end affector position
Eigen::Vector3d HexapodLeg::doFK() {
    Eigen::Vector3d pos(
        cos(currentAngles[0]) * (0.221426 * cos(currentAngles[1] + currentAngles[2]) + 0.1183145 * cos(currentAngles[1]) + 0.044925),
        sin(currentAngles[0]) * (0.221426 * cos(currentAngles[1] + currentAngles[2]) + 0.1183145 * cos(currentAngles[1]) + 0.044925),
        0.221426 * sin(currentAngles[1] + currentAngles[2]) + 0.1183145 * sin(currentAngles[1]) + 0.01065);

    return pos;
}

// Move to straight leg position
void HexapodLeg::moveToZero()
{
    setAngs(0, 0, 360*M_PI/180);
}
// Move to basic standing position
void HexapodLeg::moveToBasic()
{
    setAngs(0*M_PI/180, 40*M_PI/180, (360-102)*M_PI/180);
}
// Move to position that should fold back past limit when power disabled
void HexapodLeg::moveToOff()
{
    setAngs(0*M_PI/180, 90*M_PI/180, (360-163)*M_PI/180);
}

void HexapodLeg::doJacobianTest(const int &style)
{

    float radius = 0.15;  // meters
    double period = 10;   // HZ
    double cycles = 1;

    int dur = period*cycles*1000; //ms
    vector<double> t(dur/rsStep);
    vector<vector<double>> jointVelocity(3, vector<double>(dur/rsStep));
    vector<vector<double>> jointPosition(3, vector<double>(dur/rsStep));
    vector<vector<double>> spatialVelocity(3, vector<double>(dur/rsStep));
    vector<vector<double>> spatialPosition(3, vector<double>(dur/rsStep));

    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d desiredAngularVelocities;
    Eigen::Vector3d nextAngles;
    Eigen::Vector3d nextPos;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;
    // long currentTime;

    // Set Start Position
    setAngs(0, 134.072/2*M_PI/180, (360-143/2)*M_PI/180);
    cout<<doFK()<<endl;

    if (!simulationMode) {
        sleep_for(chrono::milliseconds(2000));
    }
    rsLoop.updateTimeDelay();

    // Simulation Cycle
    for (int i = 0; i < dur/rsStep; i++) {
        // cout<<chrono::high_resolution_clock::now().time_since_epoch().count()<<endl;

        // currentTime = chrono::high_resolution_clock::now().time_since_epoch().count();
        
        desiredSpatialVelocity << 0,
            radius * 2/period*M_PI * cos(2/period*M_PI * (i)*(rsStep/1000)),
            -radius * 2/period*M_PI * sin(2/period*M_PI * (i)*(rsStep/1000));

        // desiredSpatialVelocity << 0,0,0;

        jacobian = getJacobian();
        jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        desiredAngularVelocities = jacobianPseudoInverse * desiredSpatialVelocity;
        // desiredAngularVelocities[2] = -desiredAngularVelocities[2];

        nextAngles = currentAngles + desiredAngularVelocities*(rsStep/1000);
        nextPos = doFK();


        t[i] = i*rsStep;        
        for (int j = 0; j <=2;j++) {
            // pos
            jointVelocity[j][i] = desiredAngularVelocities[j];
            jointPosition[j][i] = nextAngles[j];
            spatialVelocity[j][i] = desiredSpatialVelocity[j];
            spatialPosition[j][i] = nextPos[j];
        }

        if (true) {
        // if (i > 37400 && i < 37420) {
        // if (abs(desiredAngularVelocities[0]) > 10 || abs(desiredAngularVelocities[1]) > 10 || abs(desiredAngularVelocities[2]) > 10) {
            // cout <<rsStep<<endl;
            cout << endl << i*rsStep << endl;
            // cout << endl << i << endl;
            // cout << "Current Angles" << endl<< currentAngles <<endl;
            // cout << "Desired Spatial Velocity" << endl<<desiredSpatialVelocity<<endl;
            // cout << "Jacobian" << endl<<jacobian<<endl;
            // cout << "Inverse Jacobian" << endl<<jacobianPseudoInverse<<endl;
            // cout << "Desired Angular Velocity" << endl << desiredAngularVelocities <<endl;
            // cout << "Next Angles" << endl << nextAngles <<endl;
            // cout << "Next Pos" << endl << nextPos <<endl;
        }
        
        setAngs(nextAngles);

        rsLoop.realTimeDelay();
    }

    // plot
    if (false) {

        /****** Spacial Position*/
        plt::figure_size(1200, 380*1.5);

        plt::subplot(3,1,1);
        plt::plot(t, spatialPosition[0]);
        plt::title("Spatial Position - Position over time");
        plt::ylabel("X (m)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3,1,2);
        plt::plot(t, spatialPosition[1]);
        plt::ylabel("Y (m)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3,1,3);       
        plt::plot(t, spatialPosition[2]);
        plt::ylabel("Z (m)");
        plt::xlabel("Time (ms)");

        /****** Spacial Velocity*/
        plt::figure_size(1200, 380*1.5);

        plt::subplot(3,1,1);
        plt::plot(t, spatialVelocity[0]);
        plt::title("Spacial Velocity - Velocity over time");
        plt::ylabel("X (m/s)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3,1,2);
        plt::plot(t, spatialVelocity[1]);
        plt::ylabel("Y (m/s)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3,1,3);       
        plt::plot(t, spatialVelocity[2]);
        plt::ylabel("Z (m/s)");
        plt::xlabel("Time (ms)");

        /****** Joint Velocity*/
        plt::figure_size(1200, 380*1.5);

        plt::subplot(3,1,1);
        plt::plot(t, jointVelocity[0]);
        plt::title("Joint Velocity - Velocity over time");
        plt::ylabel("Coxa (rad/s)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3,1,2);
        plt::plot(t, jointVelocity[1]);
        plt::ylabel("Femur (rad/s)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3,1,3);       
        plt::plot(t, jointVelocity[2]);
        plt::ylabel("Tibia (rad/s)");
        plt::xlabel("Time (ms)");

        /****** Joint Position*/
        plt::figure_size(1200, 380*1.5);

        plt::subplot(3,1,1);
        plt::plot(t, jointPosition[0]);
        plt::title("Joint Position - Position over time");
        plt::ylabel("Coxa (rads)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3,1,2);
        plt::plot(t, jointPosition[1]);
        plt::ylabel("Femur (rads)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3,1,3);       
        plt::plot(t, jointPosition[2]);
        plt::ylabel("Tibia (rads)");
        plt::xlabel("Time (ms)");

        plt::show();
    }

    return;
}

// Test the IK position control in X Y Z axis individually
void HexapodLeg::doIKTest()
{
    // Set Start Position
    Eigen::Vector3d posik = {220, 0, -170};
    moveToPos(posik);

    // Set interpolation scale
    int scale = 160;

    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test X movement
    for (int i = 1; i <= scale; i++)
    {
        posik[0] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[0] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    for (int i = 1; i <= scale; i++)
    {
        posik[0] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test Y movement
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[1] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    for (int i = 1; i <= scale * 4; i++)
    {
        posik[1] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[1] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test Z Movement
    for (int i = 1; i <= scale; i++)
    {
        posik[2] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[2] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    for (int i = 1; i <= scale; i++)
    {
        posik[2] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();
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
    cout << fmt::format("({}|{}/{})\n", currentAngles[0]*180/M_PI, currentAngles[1]*180/M_PI, -currentAngles[2]*180/M_PI + 360);
    if (!simulationMode) {
        arduino.sendCommand(fmt::format("({}|{}/{})\r", roundToDecimalPlaces(currentAngles[0]*180/M_PI, 2), roundToDecimalPlaces(currentAngles[1]*180/M_PI, 2), roundToDecimalPlaces(-currentAngles[2]*180/M_PI + 360, 2)));
    }
    
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

    cout <<endl << angs <<endl;

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

float HexapodLeg::roundToDecimalPlaces(double value, int decimalPlaces) {
    double scale = std::pow(10.0, decimalPlaces);
    return std::round(value * scale) / scale;
}
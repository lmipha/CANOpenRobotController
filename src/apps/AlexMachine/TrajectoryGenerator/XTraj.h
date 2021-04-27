// Code 
// Xiruo Function Generation
/* Header file for calculating Xiruo's trajectory,
*/

#ifndef XTraj_H_INCLUDED
#define XTraj_H_INCLUDED

#include <vector>
#include <cmath>
#include <iostream>

//#include "TrajectoryGenerator.h"

//Define constant
const int NUM_COEFFICIENTS=6;
const int NUM_VIAPOINTS=5;
const int NUM_JOINTS=5;
const int NUM_DETECT_JOINTS=6;
const int DEFAULTVALUE=0;
const int ORIGIN=0;
const int T0=0;
const double T1=0.4;
const double T2=0.6;
const double T3=0.9;
const int TF=1;
#define TORSOANGLE deg2rad(5)


#define deg2rad(deg) ((deg)*M_PI / 180.0)
#define rad2deg(rad) ((rad)*180.0 / M_PI)

// Define Data Types
// Pilot Parameters 
struct PilotParams{
    double shank_length, thigh_length, shoulder_width, foot_length, hip_width, torso_length;
    double crutch_length;
};

//Determine stance foot
/*
enum class stanceFoot {
    Left,
    Right
};*/
enum class Foot {
    Left,
    Right
};

// Step Parameters
struct StepParams{
    double duration;
    double stepHeight;
    double strideLength;
    Foot sF;
};



// position of each point
typedef struct point {
    double x, y, z;
} point;

// Xiruo Task Space Trajectory State
struct XTSTS {
    // hip position/velocity/acceleration
    point hipPos;
    point hipVel;
    point hipAcc;
    
    point rAnklePos;
    point rAnkleVel;    
    point rAnkleAcc;

    point lAnklePos;
    point lAnkleVel;
    point lAnkleAcc;
    
    double torsoAngle;

    double time;
}; 


// Exoskeleton Polynomial Definition
struct polyDef{
    std::vector<double> hipPosCoeff_x;
    std::vector<double> hipPosCoeff_y;
    std::vector<double> hipPosCoeff_z;
    std::vector<double> lAnklePosCoeff_x;
    std::vector<double> lAnklePosCoeff_y;
    std::vector<double> lAnklePosCoeff_z;
    std::vector<double> rAnklePosCoeff_x;
    std::vector<double> rAnklePosCoeff_y;
    std::vector<double> rAnklePosCoeff_z;  
    double torsoAngle;
    // beginning and ending time of corresponding coeffient
    double time[2];
};

class XiruoWalk{
    // Constant step parameters
//    static const toeClearance = 12;


    private:
        polyDef TrajectoryPolynomials[NUM_VIAPOINTS-1];       
        StepParams step_parameters;
        Foot stanceSide;
        PilotParams pilot;

          
    public:
        //set stanceSide
        void SetStanceSide(Foot a);
        //get stanceSide
        Foot GetStanceSide(); 
        //set pilot parameters
        void SetPilotParams(double height,double weight, double foot_length,double crutchLength);
        void SetPilotParams(double shank,double thigh,double shoulder,double foot,double hip,double torso,double crutchLength);
        //read pilot parameters
        PilotParams GetPilotParams();
        //convert to current kinematic model
        std::vector<double> XJoint2CurrJoint(std::vector<double> Xjoints);
        std::vector<double> XJoint2CurrJointRad(std::vector<double> Xjoints);
        //convert to X kinematic model
        std::vector<double> CurrJoint2XJoint(std::vector<double> joints);
        std::vector<double> CurrJoint2XJointRad(std::vector<double> joints);
        //right side transfer to left sdie
        std::vector<double> Right2Left(std::vector<double> Rjoints);
        std::vector<double> Left2Right(std::vector<double> Rjoints);
        
        //initial task state
        XTSTS setTS(point hipPos,point hipVel,point hipAcc,point rAnklePos,point rAnkleVel,point rAnkleAcc,point lAnklePos,point lAnkleVel,point lAnkleAcc,double torsoAngle, double t);
        //generate 5th polynomials equation (position)
        std::vector<double> getPosEquation(double a);
        //generate first derivative 5th polynomials (velocity)
        std::vector<double> getVelEquation(double a);
        //generate second derivative 5th polynomials (acceleration)
        std::vector<double> getAccEquation(double a);
        //calculate the position once having coefficient
        double getPos(std::vector<double> s,double a);
        double getVel(std::vector<double> s,double a);
        double getAcc(std::vector<double> s,double a);
        // Calculate coefficients 
        std::vector<double> getCoefficient(int n_unknown,std::vector<double> Arow0,std::vector<double> Arow1,std::vector<double> Arow2,std::vector<double> Arow3,std::vector<double> Arow4,std::vector<double> Arow5,std::vector<double> Brow);
        
        // Generates coefficient based on task space states
        std::vector<polyDef> generateTS(XTSTS initialPos, PilotParams pp, StepParams sp);
/*  
        // Saves to TrajectoryPolynomials
        XiruoWalk(Eigen::VectorXd initialPos, PilotParams pp, StepParams sp);*/
        std::vector<double> inverseKinematics(point hip,point rAnkle,point lAnkle,double L_thigh,double L_shank,double torsoAngle,Foot sf);

        // Returns a task space setpoint
        std::vector<point> getTaskState(double time);

        // Returns a joint space setpoint
        std::vector<double> getSetPoint(double time);
        //transfer joint space to task space
        std::vector<point> joint2TaskSpace(std::vector<double> joints,PilotParams pp,Foot sf);
 

};

#endif
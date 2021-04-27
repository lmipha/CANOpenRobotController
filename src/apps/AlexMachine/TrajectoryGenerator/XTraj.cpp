#include "XTraj.h"
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <vector>   
#include <algorithm>
#include <iostream>
#include "Eigen\Dense"


using namespace std; 
//set pilot parameters
void XiruoWalk::SetPilotParams(double height,double weight, double foot_length,double crutchLength){
    PilotParams pp;
    pp.shank_length = 0.271*height; 
    pp.thigh_length = 0.24*height;
    pp.shoulder_width = 0.259*height;
    pp.foot_length = foot_length;
    pp.hip_width= 0.2*height;
    pp.torso_length = 0.359*height;
    pp.crutch_length = crutchLength;
    XiruoWalk::pilot = pp;
}

PilotParams XiruoWalk::GetPilotParams(){
    return XiruoWalk::pilot;
}

void XiruoWalk::SetPilotParams(double shank,double thigh,double shoulder,double foot,double hip,double torso,double crutchLength){
    PilotParams pp;
    pp.shank_length = shank; 
    pp.thigh_length = thigh;
    pp.shoulder_width = shoulder;
    pp.foot_length = foot;
    pp.hip_width= hip;
    pp.torso_length = torso;
    pp.crutch_length = crutchLength;
    XiruoWalk::pilot = pp;
}
std::vector<double> XiruoWalk::XJoint2CurrJoint(std::vector<double> Xjoints){
    std::vector<double> result = {0,0,0,0,0,0};    
    result[0] = 90 + Xjoints[1] - Xjoints[0];
    result[1] = 180 - Xjoints[1];
    result[2] = 180 - Xjoints[3];
    result[3] = 180 - Xjoints[4];
    result[4] = Xjoints[0];
    result[5] = Xjoints[5];    
    return result;
}
std::vector<double> XiruoWalk::XJoint2CurrJointRad(std::vector<double> Xjoints){
    std::vector<double> result = {0,0,0,0,0,0};
 //   result[0] = M_PI/2 - Xjoints[2] + Xjoints[1] - Xjoints[0];
//   result[0] = M_PI/2 + Xjoints[1] - Xjoints[0];
    result[0] = M_PI - Xjoints[2];
    result[1] = M_PI - Xjoints[1];
    result[2] = M_PI - Xjoints[3];
    result[3] = M_PI - Xjoints[4];
    result[4] = Xjoints[0];
    result[5] = Xjoints[5];    
    return result;
}

std::vector<double> XiruoWalk::Right2Left(std::vector<double> Rjoints){
    std::vector<double> result = {0,0,0,0,0,0};
    result[0] = Rjoints[5];
    result[1] = Rjoints[4];
    result[2] = Rjoints[2];
    result[3] = M_PI/2 - Rjoints[1] + Rjoints[0];
    result[4] = Rjoints[1];
    result[5] = Rjoints[0];    
    return result;
}


std::vector<double> XiruoWalk::Left2Right(std::vector<double> Ljoints){
    std::vector<double> result = {0,0,0,0,0,0};
    result[0] = Ljoints[5];
    result[1] = Ljoints[4];
/*    result[2] = Ljoints[2];
    result[3] = M_PI/2- Ljoints[4] + Ljoints[5];*/
    result[2] = Ljoints[3];
    result[3] = Ljoints[2];
    result[4] = Ljoints[1];
    result[5] = Ljoints[0];    
    return result;
}


std::vector<double> XiruoWalk::CurrJoint2XJoint(std::vector<double> joints){
    std::vector<double> result = {0,0,0,0,0,0};
/*    result[0] = joints[4];
    result[1] = 180 - joints[1];
    result[2] = -joints[0] + 270 - joints[1]- joints[4];
    result[3] = 180 - joints[2];
    result[4] = 180 - joints[3];
    result[5] = joints[5];*/
    result[0] = 270 -joints[1] - joints[0];
    result[1] = 180 - joints[1];
    result[2] = rad2deg(TORSOANGLE);
    result[3] = 180 - joints[2];
    result[4] = 180 - joints[3];
    result[5] = 270 -joints[3] - joints[2];  
    return result;
}

std::vector<double> XiruoWalk::CurrJoint2XJointRad(std::vector<double> joints){
    std::vector<double> result = {0,0,0,0,0,0};
/*    result[0] = joints[4];
    result[1] = M_PI - joints[1];
    result[2] = -joints[0] + M_PI*3/2 - joints[1]- joints[4];
    result[3] = M_PI - joints[2];
    result[4] = M_PI - joints[3];
    result[5] = joints[5]; */
    result[0] = M_PI*3/2 -joints[1] - joints[0];
    result[1] = M_PI - joints[1];
    result[2] = TORSOANGLE;
    result[3] = M_PI - joints[2];
    result[4] = M_PI - joints[3];
    result[5] = M_PI*3/2 -joints[3] - joints[2];  
    return result;
}



//initial task state
XTSTS XiruoWalk::setTS(point hipPos,point hipVel,point hipAcc,point rAnklePos,point rAnkleVel,point rAnkleAcc,point lAnklePos,point lAnkleVel,point lAnkleAcc,double torsoAngle, double t){
    XTSTS returnState;
    returnState.hipPos.x = hipPos.x;
    returnState.hipPos.y = hipPos.y;
    returnState.hipPos.z = hipPos.z;
    returnState.hipVel.x = hipVel.x; 
    returnState.hipVel.y = hipVel.y; 
    returnState.hipVel.z = hipVel.z; 
    returnState.hipAcc.x = hipAcc.x;
    returnState.hipAcc.y = hipAcc.y; 
    returnState.hipAcc.z = hipAcc.z; 

    returnState.rAnklePos.x = rAnklePos.x;
    returnState.rAnklePos.y = rAnklePos.y;
    returnState.rAnklePos.z = rAnklePos.z;
    returnState.rAnkleVel.x = rAnkleVel.x; 
    returnState.rAnkleVel.y = rAnkleVel.y; 
    returnState.rAnkleVel.z = rAnkleVel.z; 
    returnState.rAnkleAcc.x = rAnkleAcc.x;
    returnState.rAnkleAcc.y = rAnkleAcc.y; 
    returnState.rAnkleAcc.z = rAnkleAcc.z;  

    returnState.lAnklePos.x = lAnklePos.x;
    returnState.lAnklePos.y = lAnklePos.y;
    returnState.lAnklePos.z = lAnklePos.z;
    returnState.lAnkleVel.x = lAnkleVel.x; 
    returnState.lAnkleVel.y = lAnkleVel.y; 
    returnState.lAnkleVel.z = lAnkleVel.z; 
    returnState.lAnkleAcc.x = lAnkleAcc.x;
    returnState.lAnkleAcc.y = lAnkleAcc.y; 
    returnState.lAnkleAcc.z = lAnkleAcc.z;
  
    returnState.torsoAngle = torsoAngle;
    returnState.time = t;
    return returnState;
}
//generate 5th polynomials equation (position)
std::vector<double> XiruoWalk::getPosEquation(double a){
   std::vector<double> result(NUM_COEFFICIENTS);
   result = {1,a,a*a,a*a*a,a*a*a*a,a*a*a*a*a};
   return result;
}

//generate first derivative 5th polynomials (velocity)
std::vector<double> XiruoWalk::getVelEquation(double a){
    std::vector<double> result(NUM_COEFFICIENTS);
    result = {0,1,2*a,3*a*a,4*a*a*a,5*a*a*a*a};
    return result;
}
    
//generate second derivative 5th polynomials (acceleration)
std::vector<double> XiruoWalk::getAccEquation(double a){
    std::vector<double> result(NUM_COEFFICIENTS);
    result = {0,0,2,6*a,12*a*a,20*a*a*a};
    return result;
}

double XiruoWalk::getPos(std::vector<double> s,double a){
    double result = s[0]+s[1]*a+s[2]*a*a+s[3]*a*a*a+s[4]*a*a*a*a+s[5]*a*a*a*a*a;
    return result;
}
double XiruoWalk::getVel(std::vector<double> s,double a){
    double result = s[1]+2*s[2]*a+3*s[3]*a*a+4*s[4]*a*a*a+5*s[5]*a*a*a*a;
    return result;
}
double XiruoWalk::getAcc(std::vector<double> s,double a){
    double result = 2*s[2]+6*s[3]*a+12*s[4]*a*a+20*s[5]*a*a*a;
    return result;
}

// Calculate coefficients 
std::vector<double> XiruoWalk::getCoefficient(int n_unknown, std::vector<double> Arow0,std::vector<double> Arow1,std::vector<double> Arow2,std::vector<double> Arow3,std::vector<double> Arow4,std::vector<double> Arow5,std::vector<double> Brow){
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A;
    A.resize(n_unknown,n_unknown);
    Eigen::Matrix<double,Eigen::Dynamic,1> B;
    B.resize(n_unknown,1);
    std::vector<double> result(NUM_COEFFICIENTS); 
    
//initialise equation matrix A and B
    for (int i = 0; i < n_unknown; i++) {      
        A(0,i) = Arow0[i];    }
         
    for (int i = 0; i < n_unknown; i++) 
        A(1,i) = Arow1[i]; 
    for (int i = 0; i < n_unknown; i++) 
        A(2,i) = Arow2[i]; 
    for (int i = 0; i < n_unknown; i++) 
        A(3,i) = Arow3[i]; 
    for (int i = 0; i < n_unknown; i++) 
        A(4,i) = Arow4[i]; 
    if (n_unknown == NUM_COEFFICIENTS){
        for (int i = 0; i < n_unknown; i++) 
            A(5,i) = Arow5[i]; 
    } 
    /*
    for (int i = 0; i < n_unknown; i++) { 
        for(int j=0; j<n_unknown;j++)     
            cout<<A(i,j)<<" ";
        cout<<"\n";
    }  */
    
    
    for (int i = 0; i < n_unknown; i++){
        B(i) = Brow[i]; 
      //  cout<<i<<" "<<B[i]<<" ";
    } 
        

    Eigen::MatrixXd C = A.inverse() *B;    
    for (int i = 0; i < n_unknown; i++){
        result[i] = C(i); 
        
    } 
        
    if (n_unknown < NUM_COEFFICIENTS)
        result[n_unknown] = DEFAULTVALUE;
    return result;
}

// Generates coefficient based on task space states
std::vector<polyDef> XiruoWalk::generateTS(XTSTS initialPos, PilotParams pp, StepParams sp){ 
    // HipAcc, RankleAcc are default at 0  
    point DEFAULT_STATE;
//    sp.sF = Foot::Left;
    DEFAULT_STATE.x = DEFAULTVALUE;
    DEFAULT_STATE.y = DEFAULTVALUE;
    DEFAULT_STATE.z = DEFAULTVALUE;
    step_parameters = sp;
    // Initialise Return value
    XTSTS returnStates[NUM_VIAPOINTS];
    vector<polyDef> returnCoefficients(NUM_VIAPOINTS-1);  
    // Define TS based on Step Parameters
    point hip_position_t1,hip_position_t2,  hip_position_tf; 
    point ankle_swing_t1, ankle_stance_t1,ankle_swing_t2,ankle_swing_t3; 
    
// calculate swing ankle and hip position at t1 (heel lift)
//stance foot stay still       
    double ankle_swing_t0_x; 
    double z_ankle_swing_t1 = sp.stepHeight/3*2;
    double x_ankle_swing_t1 = initialPos.rAnklePos.x + pp.foot_length-sqrt(pp.foot_length*pp.foot_length- z_ankle_swing_t1*z_ankle_swing_t1); 
    ankle_swing_t1.x = x_ankle_swing_t1;
    ankle_swing_t1.y = initialPos.rAnklePos.y;
    ankle_swing_t1.z = z_ankle_swing_t1;
    ankle_swing_t0_x = initialPos.rAnklePos.x;

    ankle_swing_t3.y = initialPos.rAnklePos.y;

    ankle_stance_t1.x = initialPos.lAnklePos.x;
    ankle_stance_t1.y = initialPos.lAnklePos.y;
    ankle_stance_t1.z = initialPos.lAnklePos.z;

    // calculate swing ankle at t3 (heel strikes) 
    ankle_swing_t3.x = sp.strideLength + ankle_swing_t0_x;
    if ((initialPos.lAnklePos.x - initialPos.rAnklePos.x)*(initialPos.lAnklePos.x - initialPos.rAnklePos.x) < 0.0001) 
        ankle_swing_t3.x = sp.strideLength/2 + ankle_swing_t0_x;
    ankle_swing_t3.z = ORIGIN;
    
  //  hip_position_t2.z = sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150)));
    ankle_swing_t2.x = initialPos.rAnklePos.x + (ankle_swing_t3.x-initialPos.rAnklePos.x)/2;
    ankle_swing_t2.y = initialPos.rAnklePos.y;
    ankle_swing_t2.z = sp.stepHeight; 

    //calculate hip position    
       
   // if (initialPos.hipPos.x < 0.6*sp.strideLength + ankle_swing_t0_x){       
     //   hip_position_tf.x = 0.6*sp.strideLength + ankle_swing_t0_x;
    if (initialPos.hipPos.x < (initialPos.lAnklePos.x + ankle_swing_t3.x)/2){ 
        hip_position_tf.x = (initialPos.lAnklePos.x + ankle_swing_t3.x)/2;
        hip_position_t1.x = initialPos.hipPos.x + 0.15*(hip_position_tf.x-initialPos.hipPos.x);             
        hip_position_t2.x = initialPos.hipPos.x + 0.3*(hip_position_tf.x-initialPos.hipPos.x);
    }else{
        hip_position_tf.x = initialPos.hipPos.x;
        hip_position_t1.x = initialPos.hipPos.x;             
        hip_position_t2.x = initialPos.hipPos.x;
    }    
 /*   if ((initialPos.lAnkle.x > hip_position_tf.x)&&(ankle_swing_t3.x > hip_position_tf.x)){
        hip_position_tf.x =(initialPos.lAnkle.x + ankle_swing_t3.x)/2;
        hip_position_t1.x = initialPos.hipPos.x + 0.15*(hip_position_tf.x-initialPos.hipPos.x);             
        hip_position_t2.x = initialPos.hipPos.x + 0.3*(hip_position_tf.x-initialPos.hipPos.x);   
    }*/

    hip_position_tf.y = initialPos.hipPos.y;
    hip_position_tf.z = sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150))-(ankle_swing_t3.x-hip_position_tf.x)*(ankle_swing_t3.x-hip_position_tf.x));
    if (sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150))-hip_position_tf.x*hip_position_tf.x)<hip_position_tf.z)
        hip_position_tf.z = sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150))-hip_position_tf.x*hip_position_tf.x);
    hip_position_t2.y = initialPos.hipPos.y;
 //   double z_hip_t2 = pp.shank_length + pp.thigh_length;     
//    double temp = pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150))-(hip_position_t2.x-ankle_swing_t2.x)*(hip_position_t2.x-ankle_swing_t2.x);
//    if (temp <= z_hip_t2*z_hip_t2)
    double z_hip_t2 = ankle_swing_t2.z + sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(140))-( hip_position_t2.x-ankle_swing_t2.x)*( hip_position_t2.x-ankle_swing_t2.x));
    if (sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(140)))-( hip_position_t2.x-ankle_swing_t2.x)*( hip_position_t2.x-ankle_swing_t2.x) < z_hip_t2)
        z_hip_t2 = sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(140)))-( hip_position_t2.x-ankle_swing_t2.x)*( hip_position_t2.x-ankle_swing_t2.x); 
    hip_position_t2.z = z_hip_t2;   
    
    
    
    hip_position_t1.y = initialPos.hipPos.y;  
  //  double z_hip_t1 = pp.shank_length + pp.thigh_length;     
 //   temp = pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150))-(hip_position_t1.x-x_ankle_swing_t1)*(hip_position_t1.x-x_ankle_swing_t1);
 //   if (temp <= z_hip_t1*z_hip_t1)
     double z_hip_t1 = z_ankle_swing_t1 + sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150))-(hip_position_t1.x-x_ankle_swing_t1)*( hip_position_t1.x-x_ankle_swing_t1));
     if (sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150)))-hip_position_t1.x*hip_position_t1.x < z_hip_t1)
        z_hip_t1 = sqrt(pp.shank_length*pp.shank_length+pp.thigh_length*pp.thigh_length-2*pp.shank_length*pp.thigh_length*cos(deg2rad(150)))-hip_position_t1.x*hip_position_t1.x; 
    hip_position_t1.z = z_hip_t1;
      
      
    
    
    
    // First state, initial pose
    returnStates[0] = setTS(initialPos.hipPos,initialPos.hipVel,initialPos.hipAcc,initialPos.rAnklePos,initialPos.rAnkleVel,initialPos.rAnkleAcc,initialPos.lAnklePos,initialPos.lAnkleVel,initialPos.lAnkleAcc,initialPos.torsoAngle,initialPos.time);
               
    // Second state, ankle lift
 //   if (sp.sF == Foot::Left){
        returnStates[1] = setTS(hip_position_t1,DEFAULT_STATE,DEFAULT_STATE,ankle_swing_t1,DEFAULT_STATE,DEFAULT_STATE,ankle_stance_t1,DEFAULT_STATE,DEFAULT_STATE,initialPos.torsoAngle,T1*sp.duration);
  /*  }else{
        returnStates[1] = setTS(hip_position_t1,DEFAULT_STATE,DEFAULT_STATE,ankle_stance_t1,DEFAULT_STATE,DEFAULT_STATE,ankle_swing_t1,DEFAULT_STATE,DEFAULT_STATE,initialPos.torsoAngle,T1*sp.duration); 
    }*/
 
// calculate coefficient between t1 and t0
    returnCoefficients[0].time[0] = T0;
    returnCoefficients[0].time[1] = T1;
    returnCoefficients[0].torsoAngle = initialPos.torsoAngle;
    //hip x 
    //set equations 
    int n_unknown = NUM_COEFFICIENTS;
    std::vector<double> Arow0 = getPosEquation(T0);
    std::vector<double> Arow1 = getVelEquation(T0);
    std::vector<double> Arow2 = getAccEquation(T0);
    std::vector<double> Arow3 = getPosEquation(T1*sp.duration);
    std::vector<double> Arow4 = getVelEquation(T1*sp.duration);
    std::vector<double> Arow5 = getAccEquation(T1*sp.duration);
 
 
    std::vector<double> Brow = {returnStates[0].hipPos.x,returnStates[0].hipVel.x,returnStates[0].hipAcc.x,returnStates[1].hipPos.x,returnStates[1].hipVel.x,returnStates[1].hipAcc.x};
    returnCoefficients[0].hipPosCoeff_x = XiruoWalk::getCoefficient(n_unknown,Arow0, Arow1, Arow2,Arow3,Arow4, Arow5,Brow);

    //hip y
    returnCoefficients[0].hipPosCoeff_y = {0,0,0,0,0,0}; 

    
    //hip z
    n_unknown = NUM_COEFFICIENTS - 1;
    Arow4 = getAccEquation(T1*sp.duration);
    Brow = {returnStates[0].hipPos.z,returnStates[0].hipVel.z,returnStates[0].hipAcc.z,returnStates[1].hipPos.z,returnStates[1].hipVel.z,returnStates[1].hipAcc.z};    
    returnCoefficients[0].hipPosCoeff_z = getCoefficient(n_unknown,Arow0,Arow1, Arow2, Arow3, Arow4, Arow5,Brow);
    returnStates[1].hipVel.z = getVel(returnCoefficients[0].hipPosCoeff_z,T1*sp.duration);



   //swing ankle
    // To ensure heel lift(heel go through a circle), add an viapoint between t0 and t1
    double swing_ankle_halft1_x = ankle_swing_t0_x + pp.foot_length - pp.foot_length*cos(asin(ankle_swing_t1.z/pp.foot_length)/2);    
    double swing_ankle_halft1_z = pp.foot_length*sin(asin(ankle_swing_t1.z/pp.foot_length)/2);

    n_unknown = NUM_COEFFICIENTS;
    Arow4 = getPosEquation(0.25*sp.duration);
 //   if (sp.sF == Foot::Left){  
        //swing ankle x         
        Brow = {returnStates[0].rAnklePos.x,returnStates[0].rAnkleVel.x,returnStates[0].rAnkleAcc.x,returnStates[1].rAnklePos.x,swing_ankle_halft1_x,returnStates[1].rAnkleAcc.x};
        returnCoefficients[0].rAnklePosCoeff_x = getCoefficient(n_unknown, Arow0,Arow1, Arow2,Arow3,Arow4, Arow5,Brow);

        //swing ankle y 
        returnCoefficients[0].rAnklePosCoeff_y = {initialPos.rAnklePos.y,0,0,0,0,0}; 

        //swing ankle z
        Brow = {returnStates[0].rAnklePos.z,returnStates[0].rAnkleVel.z,returnStates[0].rAnkleAcc.z,returnStates[1].rAnklePos.z,swing_ankle_halft1_z,returnStates[1].rAnkleAcc.z};
        returnCoefficients[0].rAnklePosCoeff_z = getCoefficient(n_unknown, Arow0, Arow1,Arow2, Arow3, Arow4,Arow5, Brow);

        returnStates[1].rAnkleVel.x = getVel(returnCoefficients[0].rAnklePosCoeff_x,T1*sp.duration);
        returnStates[1].rAnkleAcc.x = getAcc(returnCoefficients[0].rAnklePosCoeff_x,T1*sp.duration);

        returnStates[1].rAnkleVel.z = getVel(returnCoefficients[0].rAnklePosCoeff_z,T1*sp.duration);
        returnStates[1].rAnkleAcc.z = getVel(returnCoefficients[0].rAnklePosCoeff_z,T1*sp.duration);

        //stance ankle 
        returnCoefficients[0].lAnklePosCoeff_x = {0,0,0,0,0,0};
        returnCoefficients[0].lAnklePosCoeff_y = {initialPos.lAnklePos.y,0,0,0,0,0};
        returnCoefficients[0].lAnklePosCoeff_z = {0,0,0,0,0,0};
//        returnStates[1].rAnklePos.x = getVel(returnCoefficients[0].rAnklePosCoeff_x,T1*sp.duration);
//        returnStates[1].rAnklePos.z = getVel(returnCoefficients[0].rAnklePosCoeff_z,T1*sp.duration); 
 
 /*   }else{
        //swing ankle x 
        Brow = {returnStates[0].lAnklePos.x,returnStates[0].lAnkleVel.x,returnStates[0].lAnkleAcc.x,returnStates[1].lAnklePos.x,swing_ankle_halft1_x,returnStates[1].lAnkleAcc.x};
        returnCoefficients[0].lAnklePosCoeff_x = getCoefficient(n_unknown,Arow0, Arow1, Arow2, Arow3,Arow4, Arow5, Brow);
        //swing ankle y 
        returnCoefficients[0].rAnklePosCoeff_y = {initialPos.lAnklePos.y,0,0,0,0,0}; 
        //swing ankle z
        Brow = {returnStates[0].lAnklePos.z,returnStates[0].lAnkleVel.z,returnStates[0].lAnkleAcc.z,returnStates[1].lAnklePos.z,swing_ankle_halft1_z,returnStates[1].lAnkleAcc.z};
        returnCoefficients[0].lAnklePosCoeff_z = getCoefficient(n_unknown,Arow0, Arow1, Arow2, Arow3,Arow4, Arow5,Brow);
       

        returnStates[1].lAnkleVel.x = getVel(returnCoefficients[0].lAnklePosCoeff_x,T1*sp.duration);
        returnStates[1].lAnkleAcc.x = getAcc(returnCoefficients[0].lAnklePosCoeff_x,T1*sp.duration);

        returnStates[1].lAnkleVel.z = getVel(returnCoefficients[0].lAnklePosCoeff_z,T1*sp.duration);
        returnStates[1].lAnkleAcc.z = getVel(returnCoefficients[0].lAnklePosCoeff_z,T1*sp.duration);
       
        //stance ankle 
    
        returnCoefficients[0].rAnklePosCoeff_x = {0,0,0,0,0,0};
        returnCoefficients[0].rAnklePosCoeff_y = {initialPos.rAnklePos.y,0,0,0,0,0};
        returnCoefficients[0].rAnklePosCoeff_z = {0,0,0,0,0,0};
 //       returnStates[1].lAnklePos.x = getVel(returnCoefficients[0].lAnklePosCoeff_x,T1*sp.duration);
 //       returnStates[1].lAnklePos.z = getVel(returnCoefficients[0].lAnklePosCoeff_z,T1*sp.duration);
    }*/

/*    cout<<"state 0 hip:"<<returnStates[0].hipPos.x<<" "<<returnStates[0].hipPos.y<<" "<<returnStates[0].hipPos.z<<" ";
    cout<<"state 1 rAnkle:"<<returnStates[0].rAnklePos.x<<" "<<returnStates[0].rAnklePos.y<<" "<<returnStates[0].rAnklePos.z<<" ";
    cout<<"state 2 lAnkle:"<<returnStates[0].lAnklePos.x<<" "<<returnStates[0].lAnklePos.y<<" "<<returnStates[0].lAnklePos.z<<" ";
    
    cout<<"state 0 hip:"<<returnStates[1].hipPos.x<<" "<<returnStates[1].hipPos.y<<" "<<returnStates[1].hipPos.z<<" ";
    cout<<"state 1 rAnkle:"<<returnStates[1].rAnklePos.x<<" "<<returnStates[1].rAnklePos.y<<" "<<returnStates[1].rAnklePos.z<<" ";
    cout<<"state 2 lAnkle:"<<returnStates[1].lAnklePos.x<<" "<<returnStates[1].lAnklePos.y<<" "<<returnStates[1].lAnklePos.z<<" ";
    
    cout<<" hipPostCoeff_x: "; 
    for (int j =0; j <NUM_COEFFICIENTS;j++)
        cout<<returnCoefficients[0].hipPosCoeff_x[j]<<" ";  

    cout<<" hipPosCoeff_y: "; 
    for (int j =0; j <NUM_COEFFICIENTS;j++)
        cout<<returnCoefficients[0].hipPosCoeff_y[j]<<" ";  

    cout<<"hipPosCoeff_z: ";
    for (int j =0; j <NUM_COEFFICIENTS;j++)
        cout<<returnCoefficients[0].hipPosCoeff_z[j]<<" ";         
               
    cout<<"rAnklePosCoeff_x";        
    for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[0].rAnklePosCoeff_x[j]<<" ";
    cout<<"rAnklePosCoeff_y";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[0].rAnklePosCoeff_y[j]<<" ";
     cout<<"rAnklePosCoeff_z";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[0].rAnklePosCoeff_z[j]<<" ";
     cout<<"lAlnklePosCoeff_x";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[0].lAnklePosCoeff_x[j]<<" ";  
    cout<<"lAlnklePosCoeff_y";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[0].lAnklePosCoeff_y[j]<<" "; 
    cout<<"llnklePosCoeff_z";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[0].lAnklePosCoeff_z[j]<<" ";    */
              
            

         
        
        
  

 // Third state, max toe clearance    
    //stance foot stay still

    
   //     if (sp.sF == Foot::Left){
            if (ankle_swing_t2.x < returnStates[1].rAnklePos.x )
                ankle_swing_t2.x = returnStates[1].rAnklePos.x;
            returnStates[2] = setTS(hip_position_t2,DEFAULT_STATE,DEFAULT_STATE,ankle_swing_t2,DEFAULT_STATE,DEFAULT_STATE,ankle_stance_t1,DEFAULT_STATE,DEFAULT_STATE,initialPos.torsoAngle,T2*sp.duration);
    /*     }else{
            if (ankle_swing_t2.x < returnStates[1].lAnklePos.x )
                ankle_swing_t2.x = returnStates[1].lAnklePos.x;
            returnStates[2] = setTS(hip_position_t2,DEFAULT_STATE,DEFAULT_STATE,ankle_stance_t1,DEFAULT_STATE,DEFAULT_STATE,ankle_swing_t2,DEFAULT_STATE,DEFAULT_STATE,initialPos.torsoAngle,T2*sp.duration); 
      }*/
  // calculate coefficient between t1 and t2
      
              
              
    //hip x 
    //set equations 
    n_unknown = NUM_COEFFICIENTS;
    Arow0 = getPosEquation(T1*sp.duration);
    Arow1 = getVelEquation(T1*sp.duration);
    Arow2 = getAccEquation(T1*sp.duration);
    Arow3 = getPosEquation(TF*sp.duration);
    Arow4 = getVelEquation(TF*sp.duration);
    Arow5 = getAccEquation(TF*sp.duration);
    Brow= {returnStates[1].hipPos.x,returnStates[1].hipVel.x,returnStates[1].hipAcc.x,hip_position_tf.x,DEFAULTVALUE,DEFAULTVALUE};
    returnCoefficients[1].hipPosCoeff_x = getCoefficient(n_unknown,Arow0, Arow1, Arow2,Arow3,Arow4, Arow5,Brow);
    returnStates[2].hipPos.x = XiruoWalk::getPos(returnCoefficients[1].hipPosCoeff_x,T2*sp.duration);
    returnStates[2].hipVel.x = getVel(returnCoefficients[1].hipPosCoeff_x,T2*sp.duration);
    returnStates[2].hipAcc.x = getAcc(returnCoefficients[1].hipPosCoeff_x,T2*sp.duration);
 

    //hip y
    returnCoefficients[1].hipPosCoeff_y = {0,0,0,0,0,0};
    
    //hip z
    n_unknown = NUM_COEFFICIENTS - 1;
    Arow3 = getPosEquation(T2*sp.duration);
    Arow4 = getAccEquation(T2*sp.duration);    
    Brow = {returnStates[1].hipPos.z,returnStates[1].hipVel.z,returnStates[1].hipAcc.z,returnStates[2].hipPos.z,returnStates[2].hipAcc.z,DEFAULTVALUE};    
    returnCoefficients[1].hipPosCoeff_z = getCoefficient(n_unknown,Arow0,Arow1, Arow2, Arow3, Arow4, Arow5,Brow);
    returnStates[2].hipPos.z = XiruoWalk::getPos(returnCoefficients[1].hipPosCoeff_z,T2*sp.duration);
    returnStates[2].hipVel.z = getVel(returnCoefficients[1].hipPosCoeff_z,T2*sp.duration);
    returnStates[2].hipAcc.z = getAcc(returnCoefficients[1].hipPosCoeff_z,T2*sp.duration);
    
    
   
    n_unknown = NUM_COEFFICIENTS;
    Arow3 = getPosEquation(T3*sp.duration);
    Arow4 = getVelEquation(T3*sp.duration);
    Arow5 = getAccEquation(T3*sp.duration);
 //   if (sp.sF == Foot::Left){  
        //swing ankle x 
        Brow = {returnStates[1].rAnklePos.x,returnStates[1].rAnkleVel.x,returnStates[1].rAnkleAcc.x,ankle_swing_t3.x,DEFAULTVALUE,DEFAULTVALUE};
        returnCoefficients[1].rAnklePosCoeff_x = getCoefficient(n_unknown, Arow0,Arow1, Arow2,Arow3,Arow4, Arow5,Brow);
        returnStates[2].rAnklePos.x = XiruoWalk::getPos(returnCoefficients[1].rAnklePosCoeff_x,T2*sp.duration);
        returnStates[2].rAnkleVel.x = getVel(returnCoefficients[1].rAnklePosCoeff_x,T2*sp.duration);
        returnStates[2].rAnkleAcc.x = getAcc(returnCoefficients[1].rAnklePosCoeff_x,T2*sp.duration);
        
        
        //swing ankle y 
        returnCoefficients[1].rAnklePosCoeff_y = {initialPos.rAnklePos.y,0,0,0,0,0};
        //swing ankle z
        n_unknown = NUM_COEFFICIENTS-1;
        Arow3 = getPosEquation(T2*sp.duration);
        Arow4 = getAccEquation(T2*sp.duration);
        Arow5 = getAccEquation(T2*sp.duration);
        Brow = {returnStates[1].rAnklePos.z,returnStates[1].rAnkleVel.z,returnStates[1].rAnkleAcc.z,returnStates[2].rAnklePos.z,returnStates[2].rAnkleAcc.z,DEFAULTVALUE};
        returnCoefficients[1].rAnklePosCoeff_z = getCoefficient(n_unknown, Arow0, Arow1,Arow2, Arow3, Arow4,Arow5, Brow);
        returnStates[2].rAnklePos.z = XiruoWalk::getPos(returnCoefficients[1].rAnklePosCoeff_z,T2*sp.duration);
        returnStates[2].rAnkleVel.z = getVel(returnCoefficients[1].rAnklePosCoeff_z,T2*sp.duration);
        returnStates[2].rAnkleAcc.z = getAcc(returnCoefficients[1].rAnklePosCoeff_z,T2*sp.duration);
        //stance ankle 
        returnCoefficients[1].lAnklePosCoeff_x = {0,0,0,0,0,0};
        returnCoefficients[1].lAnklePosCoeff_y = {initialPos.lAnklePos.y,0,0,0,0,0};
        returnCoefficients[1].lAnklePosCoeff_z = {0,0,0,0,0,0};
                 
 /*   }else{
        //swing ankle x 
        Brow = {returnStates[1].rAnklePos.x,returnStates[1].rAnkleVel.x,returnStates[1].rAnkleAcc.x,ankle_swing_t3.x,DEFAULTVALUE,DEFAULTVALUE};
        returnCoefficients[1].lAnklePosCoeff_x = getCoefficient(n_unknown,Arow0, Arow1, Arow2, Arow3,Arow4, Arow5, Brow);
        returnStates[2].lAnklePos.x = XiruoWalk::getPos(returnCoefficients[1].lAnklePosCoeff_x,T2*sp.duration);
        returnStates[2].lAnkleVel.x = getVel(returnCoefficients[1].lAnklePosCoeff_x,T2*sp.duration);
        returnStates[2].lAnkleAcc.x = getAcc(returnCoefficients[1].lAnklePosCoeff_x,T2*sp.duration);
        
        
        //swing ankle y 
        returnCoefficients[1].lAnklePosCoeff_y = {initialPos.lAnklePos.y,0,0,0,0,0};
        //swing ankle z
        Arow3 = getPosEquation(T2*sp.duration);
        Arow4 = getAccEquation(T2*sp.duration);
        Arow5 = getAccEquation(T2*sp.duration);
        Brow = {returnStates[1].lAnklePos.z,returnStates[1].lAnkleVel.z,returnStates[1].lAnkleAcc.z,returnStates[2].lAnklePos.z,returnStates[2].lAnkleAcc.z,DEFAULTVALUE};
        returnCoefficients[1].lAnklePosCoeff_z = getCoefficient(n_unknown,Arow0, Arow1, Arow2, Arow3,Arow4, Arow5,Brow);
        returnStates[2].lAnklePos.z = getPos(returnCoefficients[1].lAnklePosCoeff_z,T2*sp.duration);
        returnStates[2].lAnkleVel.z = getVel(returnCoefficients[1].lAnklePosCoeff_z,T2*sp.duration);
        returnStates[2].lAnkleAcc.z = getAcc(returnCoefficients[1].lAnklePosCoeff_z,T2*sp.duration);        
       
        //stance ankle    
        returnCoefficients[1].rAnklePosCoeff_x = {0,0,0,0,0,0};
        returnCoefficients[1].rAnklePosCoeff_y = {initialPos.rAnklePos.y,0,0,0,0,0};
        returnCoefficients[1].rAnklePosCoeff_z = {0,0,0,0,0,0};
    }*/
    returnCoefficients[1].time[0] = T1*sp.duration;
    returnCoefficients[1].time[1] = T2*sp.duration;
    returnCoefficients[1].torsoAngle = initialPos.torsoAngle;
/*
    cout<<"state 2 hip:"<<returnStates[2].hipPos.x<<" "<<returnStates[2].hipPos.y<<" "<<returnStates[2].hipPos.z<<" ";
    cout<<"state 2 rAnkle:"<<returnStates[2].rAnklePos.x<<" "<<returnStates[2].rAnklePos.y<<" "<<returnStates[2].rAnklePos.z<<" ";
    cout<<"state 2 lAnkle:"<<returnStates[2].lAnklePos.x<<" "<<returnStates[2].lAnklePos.y<<" "<<returnStates[2].lAnklePos.z<<" ";

                
    cout<<"hipPosCoeff_x ";        
    for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].hipPosCoeff_x[j]<<" ";
    cout<<"hipePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].hipPosCoeff_y[j]<<" ";
     cout<<"hipPosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].hipPosCoeff_z[j]<<" ";
     cout<<"llnklePosCoeff_x ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].lAnklePosCoeff_x[j]<<" ";  
    cout<<"llnklePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].lAnklePosCoeff_y[j]<<" "; 
    cout<<"llnklePosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].lAnklePosCoeff_z[j]<<" ";                        
    cout<<"rAnklePosCoeff_x ";        
    for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].rAnklePosCoeff_x[j]<<" ";
    cout<<"rAnklePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].rAnklePosCoeff_y[j]<<" ";
     cout<<"rAnklePosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[1].rAnklePosCoeff_z[j]<<" ";*/
     


    //Fourth state, heel strikes
    point hip_position_t3;
    hip_position_t3.x = XiruoWalk::getPos(returnCoefficients[1].hipPosCoeff_x,T2*sp.duration);
    hip_position_t3.y = initialPos.hipPos.y;
    hip_position_t3.z = XiruoWalk::getPos(returnCoefficients[1].hipPosCoeff_x,T2*sp.duration);
   
 //   if (sp.sF == Foot::Left){
        returnStates[3] = setTS(hip_position_t3,DEFAULT_STATE,DEFAULT_STATE,ankle_swing_t3,DEFAULT_STATE,DEFAULT_STATE,ankle_stance_t1,DEFAULT_STATE,DEFAULT_STATE,initialPos.torsoAngle,T2*sp.duration);
 /*   }else{
        returnStates[3] = setTS(hip_position_t3,DEFAULT_STATE,DEFAULT_STATE,ankle_stance_t1,DEFAULT_STATE,DEFAULT_STATE,ankle_swing_t3,DEFAULT_STATE,DEFAULT_STATE,initialPos.torsoAngle,T2*sp.duration); 
    }*/
    //calculate coefficient between t2 and t3
    returnCoefficients[2].time[0] = T2*sp.duration;
    returnCoefficients[2].time[1] = T3*sp.duration;
    returnCoefficients[2].torsoAngle = initialPos.torsoAngle;
    //hip x 
    returnCoefficients[2].hipPosCoeff_x = returnCoefficients[1].hipPosCoeff_x;
    returnStates[3].hipPos.x = XiruoWalk::getPos(returnCoefficients[2].hipPosCoeff_x,T3*sp.duration);
    returnStates[3].hipVel.x = getVel(returnCoefficients[2].hipPosCoeff_x,T3*sp.duration);
    returnStates[3].hipAcc.x = getAcc(returnCoefficients[2].hipPosCoeff_x,T3*sp.duration);

    //hip y
    returnCoefficients[2].hipPosCoeff_y = {initialPos.hipPos.y,0,0,0,0,0};

    //hip z
    n_unknown = NUM_COEFFICIENTS;
    Arow0 = getPosEquation(T2*sp.duration);
    Arow1 = getVelEquation(T2*sp.duration);
    Arow2 = getAccEquation(T2*sp.duration);
    Arow3 = getPosEquation(TF*sp.duration);
    Arow4 = getVelEquation(TF*sp.duration);
    Arow5 = getAccEquation(TF*sp.duration);    
    Brow = {returnStates[2].hipPos.z,returnStates[2].hipVel.z,returnStates[2].hipAcc.z,hip_position_tf.z,DEFAULTVALUE,DEFAULTVALUE};    
    returnCoefficients[2].hipPosCoeff_z = getCoefficient(n_unknown,Arow0,Arow1, Arow2, Arow3, Arow4, Arow5,Brow);
    returnStates[3].hipPos.z = XiruoWalk::getPos(returnCoefficients[2].hipPosCoeff_z,T3*sp.duration);
    returnStates[3].hipVel.z = getVel(returnCoefficients[2].hipPosCoeff_z,T3*sp.duration);
    returnStates[3].hipAcc.z = getAcc(returnCoefficients[2].hipPosCoeff_z,T3*sp.duration);

    
   
    n_unknown = NUM_COEFFICIENTS;
    Arow3 = getPosEquation(T3*sp.duration);
    Arow4 = getVelEquation(T3*sp.duration);
    Arow5 = getAccEquation(T3*sp.duration);
 //   if (sp.sF == Foot::Left){  
        //swing ankle x 
        returnCoefficients[2].rAnklePosCoeff_x = returnCoefficients[1].rAnklePosCoeff_x;
        returnStates[3].rAnklePos.x = XiruoWalk::getPos(returnCoefficients[2].rAnklePosCoeff_x,T3*sp.duration);
        returnStates[3].rAnkleVel.x = getVel(returnCoefficients[2].rAnklePosCoeff_x,T3*sp.duration);
        returnStates[3].rAnkleAcc.x = getAcc(returnCoefficients[2].rAnklePosCoeff_x,T3*sp.duration);
        //swing ankle y 
        returnCoefficients[2].rAnklePosCoeff_y = {initialPos.rAnklePos.y,0,0,0,0,0};
        //swing ankle z
        Brow = {returnStates[2].rAnklePos.z,returnStates[2].rAnkleVel.z,returnStates[2].rAnkleAcc.z,returnStates[3].rAnklePos.z,returnStates[3].rAnkleVel.z,returnStates[3].rAnkleAcc.z};
        //Brow.assign(temp,temp + n_unknown);
        returnCoefficients[2].rAnklePosCoeff_z = getCoefficient(n_unknown, Arow0, Arow1,Arow2, Arow3, Arow4,Arow5, Brow);
        returnStates[3].rAnklePos.z = XiruoWalk::getPos(returnCoefficients[2].rAnklePosCoeff_z,T3*sp.duration);
        returnStates[3].rAnkleVel.z = getVel(returnCoefficients[2].rAnklePosCoeff_z,T3*sp.duration);
        returnStates[3].rAnkleAcc.z = getAcc(returnCoefficients[2].rAnklePosCoeff_z,T3*sp.duration);
        //stance ankle 
        returnCoefficients[2].lAnklePosCoeff_x = {0,0,0,0,0,0};
        returnCoefficients[2].lAnklePosCoeff_y = {initialPos.lAnklePos.y,0,0,0,0,0};
        returnCoefficients[2].lAnklePosCoeff_z = {0,0,0,0,0,0};
                   
 /*   }else{
        //swing ankle x 
        returnCoefficients[2].lAnklePosCoeff_x = returnCoefficients[1].lAnklePosCoeff_x;
        returnStates[3].lAnklePos.x = XiruoWalk::getPos(returnCoefficients[2].lAnklePosCoeff_x,T3*sp.duration);
        returnStates[3].lAnkleVel.x = getVel(returnCoefficients[2].lAnklePosCoeff_x,T3*sp.duration);
        returnStates[3].lAnkleAcc.x = getAcc(returnCoefficients[2].lAnklePosCoeff_x,T3*sp.duration);
        
        
        //swing ankle y 
        returnCoefficients[2].lAnklePosCoeff_y = {initialPos.lAnklePos.y,0,0,0,0,0};
        //swing ankle z
        Brow= {returnStates[2].lAnklePos.z,returnStates[2].lAnkleVel.z,returnStates[2].lAnkleAcc.z,returnStates[3].lAnklePos.z,returnStates[3].lAnkleVel.z,returnStates[3].lAnkleAcc.z};
        returnCoefficients[2].lAnklePosCoeff_z = getCoefficient(n_unknown,Arow0, Arow1, Arow2, Arow3,Arow4, Arow5,Brow);
        returnStates[3].lAnklePos.z = getPos(returnCoefficients[2].lAnklePosCoeff_z,T3*sp.duration);
        returnStates[3].lAnkleVel.z = getVel(returnCoefficients[2].lAnklePosCoeff_z,T3*sp.duration);
        returnStates[3].lAnkleAcc.z = getAcc(returnCoefficients[2].lAnklePosCoeff_z,T3*sp.duration);        
       
        //stance ankle   
        returnCoefficients[2].rAnklePosCoeff_x = {0,0,0,0,0,0};
        returnCoefficients[2].rAnklePosCoeff_y = {initialPos.rAnklePos.y,0,0,0,0,0};
        returnCoefficients[2].rAnklePosCoeff_z = {0,0,0,0,0,0};
    }   */
/*    cout<<"state 3 hip:"<<returnStates[3].hipPos.x<<" "<<returnStates[3].hipPos.y<<" "<<returnStates[3].hipPos.z<<" ";
    cout<<"state 3 rAnkle:"<<returnStates[3].rAnklePos.x<<" "<<returnStates[3].rAnklePos.y<<" "<<returnStates[3].rAnklePos.z<<" ";
    cout<<"state 3 lAnkle:"<<returnStates[3].lAnklePos.x<<" "<<returnStates[3].lAnklePos.y<<" "<<returnStates[3].lAnklePos.z<<" ";

                
    cout<<"hipPosCoeff_x ";        
    for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].hipPosCoeff_x[j]<<" ";
    cout<<"hipePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].hipPosCoeff_y[j]<<" ";
     cout<<"hipPosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].hipPosCoeff_z[j]<<" ";
    cout<<"rAnklePosCoeff_x ";        
    for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].rAnklePosCoeff_x[j]<<" ";
    cout<<"rAnklePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].rAnklePosCoeff_y[j]<<" ";
     cout<<"rAnklePosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].rAnklePosCoeff_z[j]<<" ";
     cout<<"lAnklePosCoeff_x ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].lAnklePosCoeff_x[j]<<" ";  
    cout<<"lAnklePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].lAnklePosCoeff_y[j]<<" "; 
    cout<<"lAnklePosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[2].lAnklePosCoeff_z[j]<<" ";    */ 


    //FIFTH state
//    if (sp.sF == Foot::Left){
        returnStates[4] = setTS(hip_position_tf,DEFAULT_STATE,DEFAULT_STATE,ankle_swing_t3,DEFAULT_STATE,DEFAULT_STATE,ankle_stance_t1,DEFAULT_STATE,DEFAULT_STATE,initialPos.torsoAngle,sp.duration);
 /*   }else{
        returnStates[4] = setTS(hip_position_tf,DEFAULT_STATE,DEFAULT_STATE,ankle_stance_t1,DEFAULT_STATE,DEFAULT_STATE,ankle_swing_t3,DEFAULT_STATE,DEFAULT_STATE,initialPos.torsoAngle,sp.duration); 
    }*/
    //calculate coefficient between t3 and tf
    returnCoefficients[3].time[0] = T3*sp.duration;
    returnCoefficients[3].time[1] = sp.duration;
    returnCoefficients[3].torsoAngle = initialPos.torsoAngle;
    //hip x 
    returnCoefficients[3].hipPosCoeff_x = returnCoefficients[2].hipPosCoeff_x;
    returnStates[4].hipPos.x = XiruoWalk::getPos(returnCoefficients[3].hipPosCoeff_x,sp.duration);
    returnStates[4].hipVel.x = getVel(returnCoefficients[3].hipPosCoeff_x,sp.duration);
    returnStates[4].hipAcc.x = getAcc(returnCoefficients[3].hipPosCoeff_x,sp.duration);

    //hip y
    returnCoefficients[3].hipPosCoeff_y = {initialPos.hipPos.y,0,0,0,0,0};

    //hip z 
    returnCoefficients[3].hipPosCoeff_z = returnCoefficients[2].hipPosCoeff_z;
    returnStates[4].hipPos.z = XiruoWalk::getPos(returnCoefficients[3].hipPosCoeff_z,sp.duration);
    returnStates[4].hipVel.z = getVel(returnCoefficients[3].hipPosCoeff_z,sp.duration);
    returnStates[4].hipAcc.z = getAcc(returnCoefficients[3].hipPosCoeff_z,sp.duration); 

    //swing ankle x 
    returnCoefficients[3].rAnklePosCoeff_x = {returnStates[3].rAnklePos.x,0,0,0,0,0};
    returnStates[4].rAnklePos.x = XiruoWalk::getPos(returnCoefficients[3].rAnklePosCoeff_x,sp.duration);
    returnStates[4].rAnkleVel.x = getVel(returnCoefficients[3].rAnklePosCoeff_x,sp.duration);
    returnStates[4].rAnkleAcc.x = getAcc(returnCoefficients[3].rAnklePosCoeff_x,sp.duration);
        
    //swing ankle y 
    returnCoefficients[3].rAnklePosCoeff_y = {initialPos.rAnklePos.y,0,0,0,0,0};
    //swing ankle z
    returnCoefficients[3].rAnklePosCoeff_z = {returnStates[3].rAnklePos.z,0,0,0,0,0};;
    returnStates[4].rAnklePos.z = getPos(returnCoefficients[3].rAnklePosCoeff_z,sp.duration);
    returnStates[4].rAnkleVel.z = getVel(returnCoefficients[3].rAnklePosCoeff_z,sp.duration);
    returnStates[4].rAnkleAcc.z = getAcc(returnCoefficients[3].rAnklePosCoeff_z,sp.duration);        
       
    //stance ankle   
    returnCoefficients[3].lAnklePosCoeff_x = {0,0,0,0,0,0};
    returnCoefficients[3].lAnklePosCoeff_y = {initialPos.lAnklePos.y,0,0,0,0,0};
    returnCoefficients[3].lAnklePosCoeff_z = {0,0,0,0,0,0}; 
/*
    cout<<"state 4 hip:"<<returnStates[4].hipPos.x<<" "<<returnStates[4].hipPos.y<<" "<<returnStates[4].hipPos.z<<" ";
    cout<<"state 4 rAnkle:"<<returnStates[4].rAnklePos.x<<" "<<returnStates[4].rAnklePos.y<<" "<<returnStates[4].rAnklePos.z<<" ";
    cout<<"state 4 lAnkle:"<<returnStates[4].lAnklePos.x<<" "<<returnStates[4].lAnklePos.y<<" "<<returnStates[4].lAnklePos.z<<" ";

                
    cout<<"hipPosCoeff_x ";        
    for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].hipPosCoeff_x[j]<<" ";
    cout<<"hipePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].hipPosCoeff_y[j]<<" ";
     cout<<"hipPosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].hipPosCoeff_z[j]<<" ";
    cout<<"rAnklePosCoeff_x ";        
    for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].rAnklePosCoeff_x[j]<<" ";
    cout<<"rAnklePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].rAnklePosCoeff_y[j]<<" ";
     cout<<"rAnklePosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].rAnklePosCoeff_z[j]<<" ";
     cout<<"lAnklePosCoeff_x ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].lAnklePosCoeff_x[j]<<" ";  
    cout<<"lAnklePosCoeff_y ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].lAnklePosCoeff_y[j]<<" "; 
    cout<<"lAnklePosCoeff_z ";        
        for (int j =0; j <NUM_COEFFICIENTS;j++)
            cout<<returnCoefficients[3].lAnklePosCoeff_z[j]<<" ";    */ 
     

    
    for (int i=0;i<NUM_VIAPOINTS-1;i++)
        TrajectoryPolynomials[i] = returnCoefficients[i];
    
    return returnCoefficients;
}


 std::vector<double> XiruoWalk::inverseKinematics(point hip,point rAnkle,point lAnkle,double L_thigh,double L_shank,double torsoAngle,Foot sf){
    std::vector<double> joint(NUM_DETECT_JOINTS);
    double temp;
        //stance side ankle
        if(hip.x < lAnkle.x) {
            temp = atan2(hip.z,-hip.x) + acos((hip.x*hip.x+hip.z*hip.z+L_shank*L_shank-L_thigh*L_thigh)/(2*sqrt(hip.x*hip.x+hip.z*hip.z)*L_shank));
            joint[0] = M_PI - temp;}
        else if (hip.x > lAnkle.x){
            temp = atan2(hip.z,hip.x) - acos((hip.x*hip.x+hip.z*hip.z+L_shank*L_shank-L_thigh*L_thigh)/(2*sqrt(hip.x*hip.x+hip.z*hip.z)*L_shank));
            joint[0] = temp;
        }else{
            temp = 0.5*acos(((-hip.x*hip.x-hip.z*hip.z+L_shank*L_shank+L_thigh*L_thigh)/(2*L_shank*L_thigh)));
            joint[0] = temp;
        }
         
        // stance side knee
        temp = acos(((-hip.x*hip.x-hip.z*hip.z+L_shank*L_shank+L_thigh*L_thigh)/(2*L_shank*L_thigh)));
        joint[1] = temp;
       
        //stance side hip 
        joint[2] = acos(((hip.x-lAnkle.x)*(hip.x-lAnkle.x)+(hip.z-lAnkle.z)*(hip.z-lAnkle.z)+L_thigh*L_thigh-L_shank*L_shank)/(2*sqrt((hip.x-lAnkle.x)*(hip.x-lAnkle.x)+(hip.z-lAnkle.z)*(hip.z-lAnkle.z))*L_thigh))+ atan((lAnkle.x-hip.x)/(hip.z-lAnkle.z)); 
        //swing side hip & vertical line
        joint[3] = acos(((hip.x-rAnkle.x)*(hip.x-rAnkle.x)+(hip.z-rAnkle.z)*(hip.z-rAnkle.z)+L_thigh*L_thigh-L_shank*L_shank)/(2*sqrt((hip.x-rAnkle.x)*(hip.x-rAnkle.x)+(hip.z-rAnkle.z)*(hip.z-rAnkle.z))*L_thigh))+ atan((rAnkle.x-hip.x)/(hip.z-rAnkle.z));   
     
        // swing side knee
        temp = acos((-(hip.x-rAnkle.x)*(hip.x-rAnkle.x)-(hip.z-rAnkle.z)*(hip.z-rAnkle.z)+L_shank*L_shank+L_thigh*L_thigh)/(2*L_shank*L_thigh));
        joint[4] = temp;
       
        // swing ankle
        if(hip.x > rAnkle.x)
            joint[5] = atan((hip.z-rAnkle.z)/(hip.x-rAnkle.x)) - acos(((hip.x-rAnkle.x)*(hip.x-rAnkle.x)+(hip.z-rAnkle.z)*(hip.z-rAnkle.z)+L_shank*L_shank-L_thigh*L_thigh)/(2*sqrt((hip.x-rAnkle.x)*(hip.x-rAnkle.x)+(hip.z-rAnkle.z)*(hip.z-rAnkle.z))*L_shank));
        else if(hip.x == rAnkle.x)
            joint[5] = M_PI/2 - acos(((hip.x-rAnkle.x)*(hip.x-rAnkle.x)+(hip.z-rAnkle.z)*(hip.z-rAnkle.z)+L_shank*L_shank-L_thigh*L_thigh)/(2*sqrt((hip.x-rAnkle.x)*(hip.x-rAnkle.x)+(hip.z-rAnkle.z)*(hip.z-rAnkle.z))*L_shank));
        else 
            joint[5] = M_PI - atan((hip.z-rAnkle.z)/(rAnkle.x-hip.x)) - acos(((hip.x-rAnkle.x)*(hip.x-rAnkle.x)+(hip.z-rAnkle.z)*(hip.z-rAnkle.z)+L_shank*L_shank-L_thigh*L_thigh)/(2*sqrt((hip.x-rAnkle.x)*(hip.x-rAnkle.x)+(hip.z-rAnkle.z)*(hip.z-rAnkle.z))*L_shank));
              
    return joint;     
 }
 std::vector<point> XiruoWalk::getTaskState(double time){
     XTSTS currentTaskState;
     polyDef currentCoefficients;
     if (time <= T1*XiruoWalk::step_parameters.duration)
        currentCoefficients = XiruoWalk::TrajectoryPolynomials[0];
     else if ((time >T1*step_parameters.duration)&&(time <= T2*XiruoWalk::step_parameters.duration))
        currentCoefficients = XiruoWalk::TrajectoryPolynomials[1];
     else if ((time >T2*step_parameters.duration)&&(time <= T3*XiruoWalk::step_parameters.duration))
        currentCoefficients = XiruoWalk::TrajectoryPolynomials[2];
     else if (time >T3*step_parameters.duration)
        currentCoefficients = XiruoWalk::TrajectoryPolynomials[3];
 //   for(int i = 0;i<NUM_COEFFICIENTS;i++)
 //       cout<<" "<< currentCoefficients.hipPosCoeff_z[i];
    point hip_pos_current;
    hip_pos_current.x = XiruoWalk::getPos(currentCoefficients.hipPosCoeff_x,time);
    hip_pos_current.y = XiruoWalk::getPos(currentCoefficients.hipPosCoeff_y,time);
    hip_pos_current.z = XiruoWalk::getPos(currentCoefficients.hipPosCoeff_z,time);
//    cout<<"hip  "<<hip_pos_current.x<<" "<<hip_pos_current.y<<" "<<hip_pos_current.z;
    point rAnkle_pos_current;
    rAnkle_pos_current.x = XiruoWalk::getPos(currentCoefficients.rAnklePosCoeff_x,time);
    rAnkle_pos_current.y = XiruoWalk::getPos(currentCoefficients.rAnklePosCoeff_y,time);
    rAnkle_pos_current.z = XiruoWalk::getPos(currentCoefficients.rAnklePosCoeff_z,time);
//   cout<<" rankle  "<<rAnkle_pos_current.x<<" "<<rAnkle_pos_current.y<<" "<<rAnkle_pos_current.z;
/*     for(int i = 0;i<NUM_COEFFICIENTS;i++)
        cout<<" x: "<< currentCoefficients.rAnklePosCoeff_x[i];
         for(int i = 0;i<NUM_COEFFICIENTS;i++)
        cout<<" y: "<< currentCoefficients.rAnklePosCoeff_y[i];
         for(int i = 0;i<NUM_COEFFICIENTS;i++)
        cout<<" z: "<< currentCoefficients.rAnklePosCoeff_z[i];*/
    point lAnkle_pos_current;
    lAnkle_pos_current.x = XiruoWalk::getPos(currentCoefficients.lAnklePosCoeff_x,time);
    lAnkle_pos_current.y = XiruoWalk::getPos(currentCoefficients.lAnklePosCoeff_y,time);
    lAnkle_pos_current.z = XiruoWalk::getPos(currentCoefficients.lAnklePosCoeff_z,time);
//    cout<<" lankle  "<<lAnkle_pos_current.x<<" "<<lAnkle_pos_current.y<<" "<<lAnkle_pos_current.z;
    std::vector<point> Tstate = {hip_pos_current,rAnkle_pos_current,lAnkle_pos_current};  
    return Tstate;
 }
 
std::vector<point> XiruoWalk::joint2TaskSpace(std::vector<double> joints,PilotParams pp,Foot sf){
    //state order: {hip,rAnkle,lAnkle}
    std::vector<point> Tstate(3);
    //joint order: {stance ankle,stance knee,torso,swing hip,swing knee,swing ankle}
    point stanceAnkle,stanceKnee, stanceHip,swingKnee,swingHip,swingAnkle;
        stanceAnkle = {0,0,0};
        stanceKnee.x = pp.shank_length*cos(joints[0]);
        stanceKnee.y = ORIGIN;
        stanceKnee.z = pp.shank_length*sin(joints[0]);
        stanceHip.x = stanceKnee.x - pp.thigh_length*cos(joints[1]-joints[0]);
        stanceHip.y = ORIGIN; 
        stanceHip.z = stanceKnee.z + pp.thigh_length*sin(joints[1]-joints[0]);
        swingHip.x = stanceHip.x;    
        swingHip.y = -pp.hip_width;
        swingHip.z = stanceHip.z ;
        swingKnee.x = swingHip.x + pp.thigh_length*sin(joints[3]);
        swingKnee.y = swingHip.y;
        swingKnee.z = swingHip.z - pp.thigh_length*cos(joints[3]);
        swingAnkle.x = swingKnee.x- pp.shank_length*sin(M_PI-joints[3]-joints[4]);
        swingAnkle.y = swingHip.y;
        swingAnkle.z = swingKnee.z- pp.shank_length*cos(M_PI-joints[3]-joints[4]);           
    
        
 //   if(sf == Foot::Left){
        Tstate={stanceHip,swingAnkle,stanceAnkle};
 //   }else
 //       Tstate={stanceHip,stanceAnkle,swingAnkle};

    return Tstate;
 }
 /*
 std::vector<point> XiruoWalk::joint2TaskSpace(std::vector<double> joints,PilotParams pp,Foot sf){
    //state order: {hip,rAnkle,lAnkle}
    std::vector<point> Tstate(3);
    //joint order: {left ankle,left knee,torso,right hip,swing knee,swing ankle}
    point lAnkle,lKnee, lHip,rKnee,rHip,rAnkle;
    lAnkle = {0,0,0};
    lKnee.x = pp.shank_length*cos(joints[0]);
    lKnee.y = ORIGIN;
    lKnee.z = pp.shank_length*sin(joints[0]);
    lHip.x = lKnee.x - pp.thigh_length*cos(joints[1]-joints[0]);
    lHip.y = ORIGIN; 
    lHip.z = lKnee.z + pp.thigh_length*sin(joints[1]-joints[0]);
    rHip.x = lHip.x;
    rHip.y = pp.hip_width;
    rHip.z = lHip.z ;
    rKnee.x = rHip.x + pp.thigh_length*sin(joints[3]);
    rKnee.y = rHip.y;
    rKnee.z = rHip.z - pp.thigh_length*cos(joints[3]);
    rAnkle.x = rKnee.x- pp.shank_length*sin(M_PI-joints[3]-joints[4]);
    rAnkle.y = rHip.y;
    rAnkle.z = rKnee.z- pp.shank_length*cos(M_PI-joints[3]-joints[4]);
    
    Tstate={lHip,rAnkle,lAnkle};
    return Tstate;
 }*/

//set stanceSide
void XiruoWalk::SetStanceSide(Foot a){
    this->stanceSide = a;
}
//get stanceSide
Foot XiruoWalk::GetStanceSide(){
    return this->stanceSide;
} 


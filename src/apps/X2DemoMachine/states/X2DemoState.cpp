#include "X2DemoState.h"

X2DemoState::X2DemoState(StateMachine *m, X2Robot *exo, X2DemoMachineROS *x2DemoMachineRos, const char *name) :
        State(m, name), robot_(exo), x2DemoMachineRos_(x2DemoMachineRos) {
    desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);

}

void X2DemoState::entry(void) {
    std::cout << "Example State Entered " << std::endl
              << "===================" << std::endl
              << "===================" << std::endl;

    // set dynamic parameter server
    dynamic_reconfigure::Server<CORC::dynamic_paramsConfig>::CallbackType f;
    f = boost::bind(&X2DemoState::dynReconfCallback, this, _1, _2);
    server_.setCallback(f);

    std::cout << robot_->getInteractionForce()[1] << std::endl;
    robot_->calibrateForceSensors();

    time0 = std::chrono::steady_clock::now();
}

void X2DemoState::during(void) {

//    std::cout<<"force 1: "<<robot_->getInteractionForce()<<std::endl;

    if(controller_mode_ == 1){ // zero torque mode
        desiredJointTorques_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setTorque(desiredJointTorques_);

    } else if(controller_mode_ == 2){ // zero velocity mode
        desiredJointVelocities_ = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        robot_->setVelocity(desiredJointVelocities_);

    } else if(controller_mode_ == 3){ // feedforward model compensation
        int motionIntend;
        if(robot_->getPosition()[1]>M_PI/4.0) motionIntend = -1;
        else motionIntend = 1;

        desiredJointTorques_ = robot_->getFeedForwardTorque(motionIntend);
        robot_->setTorque(desiredJointTorques_);

    } else if(controller_mode_ == 4){ // virtual mass controller

        Eigen::VectorXd feedBackTorque = Eigen::VectorXd::Zero(X2_NUM_JOINTS);
        double J = 0.1; // distance between knee joint and force sensor

        int motionIntend;
        if(robot_->getPosition()[1]>M_PI/4.0) motionIntend = -1;
        else motionIntend = 1;

        feedBackTorque[1] = (1.0/virtualMassRatio_-1)*J*robot_->getInteractionForce()[1];
        desiredJointTorques_ = robot_->getFeedForwardTorque(motionIntend) + feedBackTorque;
        robot_->setTorque(desiredJointTorques_);
        std::cout<<"force: "<<robot_->getInteractionForce()[1]<<std::endl;
        std::cout<<"ff: "<<robot_->getFeedForwardTorque(motionIntend)[1]<<std::endl;
        std::cout<<"fb: "<<feedBackTorque[1]<<std::endl;
        std::cout<<"total: "<<desiredJointTorques_[1]<<std::endl;
        std::cout<<"***************"<<std::endl;
    }
}

void X2DemoState::exit(void) {
    std::cout << "Example State Exited" << std::endl;
}

Eigen::VectorXd &X2DemoState::getDesiredJointTorques() {
    return desiredJointTorques_;
}

void X2DemoState::dynReconfCallback(CORC::dynamic_paramsConfig &config, uint32_t level) {

    controller_mode_ = config.controller_mode;
    virtualMassRatio_ = config.virtual_mass_ratio;

    if(controller_mode_ == 1) robot_->initTorqueControl();
    if(controller_mode_ == 2) robot_->initVelocityControl();
    if(controller_mode_ == 3) robot_->initTorqueControl();

    return;



}
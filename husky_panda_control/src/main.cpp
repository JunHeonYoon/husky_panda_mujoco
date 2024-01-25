#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include "mujoco/mujoco_bridge.h"
#include "data_container/data_container.h"
#include "robot/robot_model.h"
#include "controller/controller.h"
#include "utils/suhan_benchmark.h"

int main(int argc, const char** argv)
{
    robot::RobotType robot_type = robot::RobotType::Manipulator;
    const float_t sim_hz = 1000; 

    std::shared_ptr<MujocoBridge> mb_;
    std::shared_ptr<robot::RobotModel> robot_;
    std::shared_ptr<Controller> control_;
    std::shared_ptr<DataContainer> robot_data_;
    
    // initialize mujoco bridge
    if(robot_type == robot::RobotType::Manipulator)
    {
        mb_ = std::make_shared<MujocoBridge>("/home/yoonjunheon/git/husky_panda_mujoco/husky_panda_description/panda.xml");
    }
    else if(robot_type == robot::RobotType::MobileManipulator)
    {
        mb_ = std::make_shared<MujocoBridge>("/home/yoonjunheon/git/husky_panda_mujoco/husky_panda_description/husky_panda.xml");
    }

    mb_->setSimFreq(sim_hz);

    // physics thread (for mujoco sim)
    std::thread physicsthreadhandle([&mb_]() {
        mb_->PhysicsThread(mb_->sim.get());
    });


    // initilize robot model (RBDL)
    robot_ = std::make_shared<robot::RobotModel>(robot_type);

    // initialize robot state data
    while(!mb_->is_model_loaded){} // wait until model loaded
    robot_data_ = std::make_shared<DataContainer>(mb_->getNumq(), mb_->getNumv(), mb_->getNumu());

    // initilize robot controller
    control_= std::make_shared<Controller>(robot_, robot_data_);
    

    std::thread computethreadhandle([&mb_, &control_, &robot_data_]()
    {
        const std::chrono::milliseconds desiredPeriod(1000 / int(mb_->getSimFreq()));
        while (true)
        {
            if(mb_->is_init_)
            {
                auto startTime = std::chrono::high_resolution_clock::now();
                // Getting Current state from mujoco
                if(mb_->state_mutex.try_lock() && !robot_data_->is_current_updated)
                {
                    mb_->state_mutex.unlock();
                    robot_data_->setCurrentState(mb_->getQpos(), mb_->getQvel(), mb_->getQforce());
                    robot_data_->is_current_updated = true;
                }
                // Compute desired control input from controller
                if(robot_data_->is_current_updated)
                {
                    control_->update();
                    robot_data_->is_current_updated = false;
                }
                // set control input to mujoco
                if(mb_->ctrl_mutex.try_lock())
                {
                    mb_->ctrl_mutex.unlock();
                    Eigen::VectorXd ctrl;
                    ctrl.resize(mb_->getNumu());
                    robot_data_->getDesiredState(ctrl);
                    mb_->setCtrlInput(ctrl);
                }
                
                // set controller loop time wrt simulation frequency
                auto endTime = std::chrono::high_resolution_clock::now();
                auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
                if (elapsedTime < desiredPeriod) 
                {
                    std::this_thread::sleep_for(desiredPeriod - elapsedTime);
                } 
                else 
                {
                    std::cerr << "Warning: Calculation time is longer than simulation dt!!" << std::endl;
                }

            }
        }
    });

    mb_->sim->RenderLoop();
    physicsthreadhandle.join();
}

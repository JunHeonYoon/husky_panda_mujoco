#include "mujoco_bridge.h"
#include <Eigen/Dense>
#include <thread>


int main(int argc, const char** argv)
{
    // initialize mujoco bridge
    MujocoBridge mb("/home/yoonjunheon/git/husky_panda_mujoco/husky_panda_description/husky_panda.xml");
    const float_t sim_hz = 1000; 
    mb.setSimFreq(sim_hz);
    // physics thread
    std::thread physicsthreadhandle(&MujocoBridge::PhysicsThread, &mb, mb.sim.get());

    // state thread
    std::thread statethreadhandle([&mb]()
    {
        while (true)
        {
            if(mb.is_init_ && mb.state_mutex.try_lock())
            {
                Eigen::VectorXd qpos = mb.getQpos();
                mb.state_mutex.unlock();

                // std::cout<<qpos.transpose()<<std::endl;
            }
        }
    });

    std::thread ctrlthreadhandle([&mb]()
    {
        while (true)
        {
            if(mb.is_init_)
            {
                Eigen::VectorXd ctrl;
                ctrl.resize(mb.getNumu());
                // std::cout<< "ctrl size: "<< ctrl.size() <<std::endl;
                ctrl << 1.0, -1.0, 1.0, -1.0, 0, 0, 0, -1.57079, 0, 1.57079, 0.7853, 255;
                
                mb.setCtrlInput(ctrl);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    mb.sim->RenderLoop();
    physicsthreadhandle.join();
}

#include "mujoco_bridge.h"
#include "suhan_benchmark.h"
#include "Eigen/Dense"

mjModel* MujocoBridge::m_ = NULL;
mjData* MujocoBridge::d_ = NULL;
mjvCamera MujocoBridge::cam_;
mjvOption MujocoBridge::opt_;
mjvScene MujocoBridge::scn_;
mjrContext MujocoBridge::con_;
bool MujocoBridge::button_left_ = false;
bool MujocoBridge::button_middle_ = false;
bool MujocoBridge::button_right_ = false;
double MujocoBridge::lastx_ = 0;
double MujocoBridge::lasty_ = 0;

int main(int argc, const char** argv)
{
    // initialize mujoco bridge
    MujocoBridge mjbg;
    mjbg.loadModel();
    const float_t hz = 1000.0;
    mjbg.init();
    mjbg.setCtrlFreq(hz);


    Eigen::VectorXd ctrl;
    ctrl.resize(mjbg.getNumu());
    // std::cout<< "ctrl size: "<< ctrl.size() <<std::endl;
    ctrl << 0, 0, 0, 0, 0, 0, 0, -1.57079, 0, 1.57079, 0.7853, 255;

    unsigned int step = 0;
    SuhanBenchmark timer;
    
    while(mjbg.isWindowOpen())
    { 
        /* advance interactive simulation for 1/60 sec
        Assuming MuJoCo can simulate faster than real-time, which it usually can,
        this loop will finish on time for the next frame to be rendered at 60 fps.
        Otherwise add a cpu timer and exit this loop when it is time to render. */
        double simtime = mjbg.getSimTime();
        while(mjbg.getSimTime() - simtime < 1.0/60.0)
        {
            // timer.reset();
            mjbg.updateState();

            // TODO: controller
            mjbg.setCtrlInput(ctrl);

            mjbg.updateInput();
            // std::cout<< "elapsed time[ms]: " << timer.elapsed()*1000 <<std::endl;
        }

        if(step % 50 == 0)
        {
            auto qpos = mjbg.getQpos();
            auto qvel = mjbg.getQvel();

            std::cout << "============================================="<<std::endl;
            std::cout<< "Sim time :" << mjbg.getSimTime() << std::endl;
            std::cout<< "qpos:"<<std::endl;
            std::cout<< qpos.transpose() <<std::endl;
            
            std::cout<<"\n"<<std::endl;

            std::cout<< "qvel:"<<std::endl;
            std::cout<< qvel.transpose()<<std::endl;
            std::cout << "============================================="<<std::endl;
            
            std::cout<<"\n\n"<<std::endl;
        }
        mjbg.updateUtil();
        step++;
    }
    return 0;
}
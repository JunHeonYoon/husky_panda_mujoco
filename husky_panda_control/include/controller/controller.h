#ifndef CONTROLLER_
#define CONTROLLER_

#include <thread>
#include <mutex>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "data_container/data_container.h"
#include "robot/robot_model.h"
#include "utils/math_type_define.h"
#include "utils/suhan_benchmark.h"
#include "fwd.h"

using namespace Eigen;

class Timer 
{
    public:
    explicit Timer(double rate = 30.0) : period_(1.0 / rate) {}

    bool trigger() 
    {
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - time_stamp_;

    if (elapsed_time.count() >= period_) 
    {
        time_stamp_ = current_time;
        return true;
    }

    return false;
    }

    private:
    std::chrono::high_resolution_clock::time_point time_stamp_;
    double period_;
};

enum CTRL_MODE{NONE, HOME};

class Controller
{   
    public:
        Controller(std::shared_ptr<robot::RobotModel> robot, std::shared_ptr<DataContainer> data);
        ~Controller();

        bool init();
        void starting();
        void update();
        void stopping();

        void getCurrentState();
        void setControlInput();
        VectorXd PDControl(VectorXd q_desired, VectorXd qdot_desired);

        void tmpPrintState()
        {
            robot_data_->getCurrentState(q_, qdot_, tau_);
            std::cout<<"State   : "<<q_.transpose()<<std::endl;
            std::cout<<"Velocity: "<<qdot_.transpose()<<std::endl;
            std::cout<<"Torque  : "<<tau_.transpose()<<std::endl;
            std::cout<<"\n";
        }

        void modeChangeReaderProc();
        void asyncCalculationProc(); // hqp calculation
        // void asyncNJSDFProc();

        int kbhit(void)
        {
            struct termios oldt, newt;
            int ch;
            int oldf;

            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

            ch = getchar();

            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            fcntl(STDIN_FILENO, F_SETFL, oldf);

            if(ch != EOF)
            {
            ungetc(ch, stdin);
            return 1;
            }

            return 0;
        }

        bool mode_change_ = false;
        CTRL_MODE ctrl_mode_{NONE};
        bool exit_flag_ = false;
        bool is_simulation_run_ = true;
        double play_time_= 0.;
        double control_start_time_;
        unsigned long tick_;
        // const double njsdf_hz_{100.0};
        const double hz_{1000.0};
        bool is_first = true;
        int DBG_CNT = 0;

        SuhanBenchmark timer_1, timer_2;


    private:
        VectorXd q_;
        VectorXd qdot_;
        VectorXd q_init_;
        VectorXd qdot_init_;
        VectorXd q_desired_;
        VectorXd qdot_desired_;
        Affine3d target_ee_pose_;
        Affine3d ee_pose_;
        MatrixXd j_;

        VectorXd tau_;
        VectorXd tau_desired_;

        MatrixXd m_;
        VectorXd nle_;

        // for mobile manipulator
        VectorXd v_;   // joint velocity
        VectorXd qv_;  // joint position
        VectorXd v_init_;   // joint velocity
        VectorXd qv_init_;  // joint position
        VectorXd v_desired_;   // joint velocity
        VectorXd qv_desired_;  // joint position
        MatrixXd jv_;  // jacobian in joint velocity space
        MatrixXd mv_;  // Mass matrix in joint velocity space
        VectorXd nlev_;  // Nonlinear Effect torque in joint velocity space


        


        std::shared_ptr<robot::RobotModel> robot_;
        std::shared_ptr<DataContainer> robot_data_;
        // std::shared_ptr<QP> qp_;
        std::shared_ptr<Timer> trigger_rate_ ;

        // Thread
        std::thread mode_change_thread_;
        std::thread async_calculation_thread_;
        // std::thread async_njsdf_thread_;
        // bool njsdf_thread_enabled_{false};

        // Mutex
        std::mutex calculation_mutex_;
        // std::mutex NJSDF_mutex_;
        // std::mutex NJSDF_input_mutex_;
        std::mutex input_mutex_;

        // bool quit_all_proc_{false};

        
};


#endif
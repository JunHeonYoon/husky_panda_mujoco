#include "controller/controller.h"
#include <iomanip>

Controller::Controller(std::shared_ptr<robot::RobotModel> robot, std::shared_ptr<DataContainer> data)
{
    robot_ = robot;
    robot_data_ = data;
    if(Controller::init()) std::cout<<"Controller started"<<std::endl;
}

Controller::~Controller()
{
    Controller::stopping();
}

bool Controller::init()
{
    mode_change_thread_ = std::thread(&Controller::modeChangeReaderProc, this);

    unsigned int nq = robot_-> getNumq();
    unsigned int nv = robot_-> getNumv();
    unsigned int nu = robot_-> getNumu();
    q_.resize(nq);
    qdot_.resize(nq);
    q_init_.resize(nq);
    qdot_init_.resize(nq);
    q_desired_.resize(nq);
    qdot_desired_.resize(nq);
    j_.resize(ee_dof, nq);
    tau_.resize(nu);
    tau_desired_.resize(nu);
    m_.resize(nq, nq);
    nle_.resize(nq);
    v_.resize(nv);
    qv_.resize(nv);
    v_init_.resize(nv);
    qv_init_.resize(nv);
    v_desired_.resize(nv);
    qv_desired_.resize(nv);
    jv_.resize(ee_dof, nv);
    mv_.resize(nv, nv);
    nlev_.resize(nv);

    q_.setZero();
    qdot_.setZero();
    q_init_.setZero();
    qdot_init_.setZero();
    q_desired_.setZero();
    qdot_desired_.setZero();
    j_.setZero();
    tau_.setZero();
    tau_desired_.setZero();
    m_.setZero();
    nle_.setZero();
    v_.setZero();
    qv_.setZero();
    v_init_.setZero();
    qv_init_.setZero();
    v_desired_.setZero();
    qv_desired_.setZero();
    jv_.setZero();
    mv_.setZero();
    nlev_.setZero();


    tick_ = 0;
    
    return true;
}

void Controller::starting()
{
    play_time_ = 0;
}

void Controller::update()
{
    if(is_first)
    {
        is_first = false;
        Controller::starting();
    }
    input_mutex_.lock();
    getCurrentState();
    robot_->getUpdateKinematics(q_, qdot_);
    ee_pose_ = robot_->getTransformation(panda_num_links);
    j_ = robot_->getJacobian(panda_num_links);
    m_ = robot_->getMassMatrix(); 
    nle_ = robot_->getNonlinearEffect();
    if(robot_->robot_type_ == robot::MobileManipulator)
    {
        v_ = qdot_.segment(3, v_.size()); // except x, y, theta
        qv_ = q_.segment(3, v_.size());   // except x, y, theta
        jv_ = robot_->getvJacobian(panda_num_links);
        mv_ = robot_->getvMassMatrix();
        nlev_ = robot_->getvNonlinearEffect();
    }
    input_mutex_.unlock();

    if(calculation_mutex_.try_lock())
    {
        calculation_mutex_.unlock();
        if(async_calculation_thread_.joinable()) async_calculation_thread_.join();
        async_calculation_thread_ = std::thread(&Controller::asyncCalculationProc, this);
    }
    for(size_t i=0; i<9; ++i)
    {
        auto sleep_duration = std::chrono::duration<double>(1.0 / 30000);
        std::this_thread::sleep_for(sleep_duration);
        if(calculation_mutex_.try_lock())
        {
            calculation_mutex_.unlock();
            if(async_calculation_thread_.joinable()) async_calculation_thread_.join();
            break;
        }
    }

    setControlInput();

    if (DBG_CNT++ > hz_ / 10)
        {
            DBG_CNT = 0;
            std::cout << "\n\n------------------------------------------------------------------" << std::endl;
            std::cout << "time       : " << std::fixed << std::setprecision(3) << play_time_ << std::endl;
            std::cout << "q now      :\t";
            std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
            std::cout << "q desired  :\t";
            std::cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << std::endl;
            std::cout << "x          :\t";
            std::cout << ee_pose_.translation().transpose() << std::endl;
            std::cout << "R          :\t" << std::endl;
            std::cout << std::fixed << std::setprecision(3) << ee_pose_.rotation() << std::endl;
            std::cout << "qdot now   :\t";
            std::cout << std::fixed << std::setprecision(3) << qdot_.transpose() << std::endl;
            std::cout << "tau now    :\t";
            std::cout << std::fixed << std::setprecision(3) << tau_.transpose() << std::endl;
            std::cout << "tau desired:\t";
            std::cout << std::fixed << std::setprecision(3) << tau_desired_.transpose() << std::endl;
            // std::cout << "J        :\t" << std::endl;
            // std::cout << std::fixed << std::setprecision(3) << j_ << std::endl;
            // std::cout << "M        :\t" << std::endl;
            // std::cout << std::fixed << std::setprecision(3) << m_ << std::endl;
            // std::cout << "NLE      :\t" << std::endl;
            // std::cout << std::fixed << std::setprecision(3) << nle_.transpose() << std::endl;
            std::cout << "------------------------------------------------------------------\n\n" << std::endl;
        }

        tick_++;
        play_time_ = tick_ / hz_;
}

void Controller::stopping()
{
    std::cout << "Controller::stopping" << std::endl;
}

void Controller::getCurrentState()
{
    Eigen::VectorXd mj_q, mj_v, mj_tau;
    robot_data_->getCurrentState(mj_q, mj_v, mj_tau);
    // Because state, joint velocity, control input dimension
    // are different with mujoco data, we have to modify the data
    // w.r.t our controller data
    if(robot_->robot_type_ == robot::Manipulator)
    {
        // ------------------ mujoco -----------------
        // nq = 9 -> joint1~7, left/right finger  : mj_q
        // nv = 9 -> joint1~7, left/right finger  : mj_v, mj_tau
        // nu = 8 -> joint1~7, finger(constrained)
        // ---------------- controller ----------------
        // TODO: add finger joint
        // nq = 7 -> joint1~7 : q_, q_dot_
        // nv = 7 -> joint1~7 : v_
        // nu = 7 -> joint1~7 : tau_
        // --------------------------------------------
        q_ = mj_q.head(panda_dof);
        qdot_ = mj_v.head(panda_dof);
        tau_ = mj_tau.head(panda_dof);

    }
    else if(robot_->robot_type_ == robot::MobileManipulator)
    {
        // ------------------ mujoco -----------------
        // nq = 20 -> floating base(trans, quaternion), 4 wheels, joint1~7, left/right finger : mj_q
        // nv = 19 -> floating base(trans, rotation), 4 wheels, joint1~7, left/right finger   : mj_v, mj_tau
        // nu = 12 -> 4 wheels, joint1~7, finger(constrained)
        // ---------------- controller ----------------
        // TODO: add finger joint
        // nq = 12 -> floating base(x, y, theta), 2 wheels, joint1~7 : q_, q_dot_
        // nv = 9  -> 2 wheels, joint1~7                             : v_
        // nu = 9  -> 2 wheels, joint1~7                             : tau_
        // --------------------------------------------
        double theta = DyrosMath::quaternionToYaw(mj_q.segment(3, 4));
        q_ << mj_q.head(2), theta, mj_q(7), mj_q(8), mj_q.segment(11, panda_dof);
        qdot_ << mj_v.head(2), mj_v(5), mj_v(7), mj_v(8), mj_v.segment(10, panda_dof);
        tau_ << mj_tau.segment(6, 2), mj_tau.segment(10, panda_dof);
    }
}

void Controller::setControlInput()
{
    Eigen::VectorXd mj_u_desired;
    // Because state, joint velocity, control input dimension
    // are different with mujoco data, we have to modify the data
    // w.r.t our controller data
    if(robot_->robot_type_ == robot::Manipulator)
    {
        // ------------------ mujoco -----------------
        // nu = 8 -> joint1~7, finger(constrained) : mj_u_desired (torque for joint, position for gripper)
        // ---------------- controller ----------------
        // TODO: add finger joint
        // nu = 7 -> joint1~7 : tau_desired_ (torque for joint)
        // --------------------------------------------
        mj_u_desired.resize(panda_dof + 1);
        mj_u_desired.setZero();
        mj_u_desired.head(panda_dof) = tau_desired_;
        // mj_u_desired(panda_dof) = 255;
    }
    else if(robot_->robot_type_ == robot::MobileManipulator)
    {
        // ------------------ mujoco -----------------
        // nu = 12 -> 4 wheels, joint1~7, finger(constrained) : mj_u_desired (velocity for wheels
        //                                                                    torque for joint, position for gripper)
        // ---------------- controller ----------------
        // TODO: add finger joint
        // TODO: add control input of mobile
        // nu = 9  -> 2 wheels, joint1~7 : tau_desired_ (torque for wheels, torque for joint)
        // --------------------------------------------
        mj_u_desired.resize(panda_dof + 5);
        mj_u_desired.setZero();
        // mj_u_desired.head(4) << 1, -1, 1, -1;
        mj_u_desired.segment(4, panda_dof) = tau_desired_.segment(2, panda_dof);
    }
    robot_data_ ->setDesiredState(mj_u_desired);
}

VectorXd Controller::PDControl(VectorXd q_desired, VectorXd qdot_desired)
{
    double kp, kv;
    kp = 400;
    kv = 40;

    Eigen::VectorXd tau_desired;

    if(robot_->robot_type_ == robot::Manipulator)
    {
        tau_desired = m_ * ( kp*(q_desired - q_) + kv*(qdot_desired - qdot_)) + nle_;
    }
    else if(robot_->robot_type_ == robot::MobileManipulator)
    {
        tau_desired = mv_ * ( kp*(q_desired - qv_) + kv*(qdot_desired - v_)) + nlev_;
    }

    return tau_desired;
}

void Controller::modeChangeReaderProc()
{
    while (!exit_flag_)
        {
            if(kbhit())
            {
                calculation_mutex_.lock();
                int key = getchar();
                switch (key)
                {
                    case 'h': // for home position joint ctrl
                        ctrl_mode_ = HOME;
                        mode_change_ = true;
                        break;
                    case '\t':
                        if (is_simulation_run_) {
                            std::cout << "Simulation Pause" << std::endl;
                            is_simulation_run_ = false;
                        }
                        else {
                            std::cout << "Simulation Run" << std::endl;
                            is_simulation_run_ = true;
                        }
                        break;
                    case 'q':
                        is_simulation_run_ = false;
                        exit_flag_ = true;
                        Controller::~Controller();
                        break;
                    default:
                        ctrl_mode_ = NONE;
                        mode_change_ = true;
                        break;
                }
                calculation_mutex_.unlock();
            }
        }
}

void Controller::asyncCalculationProc()
{
    calculation_mutex_.lock();
        
    if(ctrl_mode_ == HOME)
    {
        if(mode_change_)
        {
            std::cout << "================ Mode change: HOME position ================" <<std::endl;
            mode_change_ = false;
            q_init_ = q_;
            qdot_init_ = qdot_;
            v_init_ = v_;
            qv_init_ = qv_;
            control_start_time_ = play_time_;
        }
        // -- moveJointPosition --
        VectorXd target_q(panda_dof); 
        target_q << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, M_PI / 4;
        Eigen::VectorXd zero_vector(panda_dof);
        zero_vector.setZero();
        if(robot_->robot_type_ == robot::Manipulator)
        {
            q_desired_ = DyrosMath::cubicVector<panda_dof>(play_time_,
                                                    control_start_time_,
                                                    control_start_time_ + 2.0, q_init_, target_q, zero_vector, zero_vector);
            qdot_desired_ = DyrosMath::cubicDotVector<panda_dof>(play_time_,
                                                    control_start_time_,
                                                    control_start_time_ + 2.0, q_init_, target_q, zero_vector, zero_vector);
            
            tau_desired_ = PDControl(q_desired_, qdot_desired_);
        }
        else if(robot_->robot_type_ == robot::MobileManipulator)
        {
            qv_desired_.tail(panda_dof) =  DyrosMath::cubicVector<panda_dof>(play_time_,
                                                    control_start_time_,
                                                    control_start_time_ + 2.0, v_init_.tail(panda_dof), target_q, zero_vector, zero_vector);
            v_desired_.tail(panda_dof) = DyrosMath::cubicDotVector<panda_dof>(play_time_,
                                                    control_start_time_,
                                                    control_start_time_ + 2.0, v_init_.tail(panda_dof), target_q, zero_vector, zero_vector);
            tau_desired_ = PDControl(qv_desired_, v_desired_);
        }
    }
    else
    {
        tau_desired_.tail(panda_dof) = nle_.tail(panda_dof);
    }

    calculation_mutex_.unlock();
}

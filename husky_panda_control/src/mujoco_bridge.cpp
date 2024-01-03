#include "mujoco_bridge.h"

MujocoBridge::MujocoBridge()
{
    position_history_ = 0;
    previous_time_ = 0;
    ctrl_update_freq_ = 1000;
    last_update_ = 0.0;
}

MujocoBridge::~MujocoBridge()
{
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);

    mj_deleteData(MujocoBridge::d_);
    mj_deleteModel(MujocoBridge::m_);

    glfwTerminate();
}

void MujocoBridge::loadModel(const char *filename)
{
    char error[1000] = "Could not load binary model";
    m_ = mj_loadXML(filename, 0 , error, 1000);
    if(!m_) mju_error_s("Load model error: %s", error);

    // make data
    this->d_ = mj_makeData(m_);
}

void MujocoBridge::loadModel()
{
    char filename[] = "/home/yoonjunheon/git/husky_panda_mujoco/husky_panda_description/husky_panda.xml";
    MujocoBridge::loadModel(filename);
}

void MujocoBridge::setCtrlFreq(const float_t hz)
{
    ctrl_update_freq_ = hz;
    m_->opt.timestep = 1 / ctrl_update_freq_;
}

void MujocoBridge::init()
{
    if (!glfwInit()) 
    {
        mju_error("Could not initialize GLFW");
    }
    // create window, make OpenGL context current and request v-sync
    window_ = glfwCreateWindow(1200, 900, "Husky_Panda", NULL, NULL);
    glfwMakeContextCurrent(window_);

    // initialise mujoco visualization data structures
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    // create scene and context
    mjv_makeScene(m_, &scn_, 2000);
    mjr_makeContext(m_, &con_, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window_, MujocoBridge::keyboard);
    glfwSetCursorPosCallback(window_, MujocoBridge::mouse_move);
    glfwSetMouseButtonCallback(window_, MujocoBridge::mouse_button);
    glfwSetScrollCallback(window_, MujocoBridge::scroll);

    // it is just setting up the camera. Nothing else. If we even comment it out then also code works
    // with default configuration.
    double arr_view[] = {90, -45, 3.44, 0.0, 0.0, 0.0};
    cam_.azimuth = arr_view[0];
    cam_.elevation = arr_view[1];
    cam_.distance = arr_view[2];
    cam_.lookat[0] = arr_view[3];
    cam_.lookat[1] = arr_view[4];
    cam_.lookat[2] = arr_view[5];
}

void MujocoBridge::updateUtil()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // Making camera follow the robot
    cam_.lookat[0] = d_->qpos[0];
    cam_.lookat[1] = d_->qpos[1];

    // update scene and render
    mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);


    // swap OpenGL buffers
    glfwSwapBuffers(window_);

    glfwPollEvents();
}

void MujocoBridge::updateState()
{
    mj_checkPos(m_, d_);
    mj_checkVel(m_, d_);
    mj_fwdPosition(m_, d_);
    mj_sensorPos(m_, d_);
    mj_energyPos(m_, d_);
    mj_fwdVelocity(m_, d_);
    mj_sensorVel(m_, d_);
    mj_energyVel(m_, d_);
}

void MujocoBridge::updateInput()
{
    mj_fwdActuation(m_, d_);
    mj_fwdAcceleration(m_, d_);
    mj_fwdConstraint(m_, d_);
    mj_sensorAcc(m_, d_);
    mj_checkAcc(m_, d_);

    // compare forward and inverse solutions if enabled
    if( m_->opt.enableflags & (mjENBL_FWDINV) ) mj_compareFwdInv(m_, d_);
    mj_RungeKutta(m_, d_, 4);

}

Eigen::VectorXd MujocoBridge::getQpos()
{
    Eigen::VectorXd qpos;
    qpos.resize(MujocoBridge::getNumq());
    for (size_t i = 0; i < qpos.size(); i++)
    {
        qpos(i) = d_->qpos[i];
    }
    return qpos;

}

Eigen::VectorXd MujocoBridge::getQvel()
{
    Eigen::VectorXd qvel;
    qvel.resize(MujocoBridge::getNumv());
    for (size_t i = 0; i < qvel.size(); i++)
    {
        qvel(i) = d_->qvel[i];
    }
    return qvel;
}

void MujocoBridge::setCtrlInput(const Eigen::VectorXd &ctrl)
{
    if(ctrl.size() != MujocoBridge::getNumu())
    {
        std::cout<< "Your control input size "<< ctrl.size()
        << "and robot control input size "<< MujocoBridge::getNumu()
        <<" are not equal!"<<std::endl;
    }
    else
    {
        for(size_t i = 0; i < ctrl.size(); i++)
        {
            d_->ctrl[i] = ctrl[i];
        }
    }
}

// keyboard callback for OpenGL
void MujocoBridge::keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key==GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m_,d_);
        mj_forward(m_, d_);
    }
}

// mouse button callback
void MujocoBridge::mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left_ = (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle_ = (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right_ = (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx_, &lasty_);
}

// mouse move callback
void MujocoBridge::mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    if(!button_left_ && !button_middle_ && !button_right_)
        return;
    
    // compute mouse displacement
    double dx = xpos - lastx_;
    double dy = ypos - lasty_;
    lastx_ = xpos;
    lasty_ = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS || 
                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
    
    // determine action based on mouse button
    mjtMouse action;
    if(button_right_)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left_)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;
    
    // move camera
    mjv_moveCamera(m_, action, dx/height, dy/height, &scn_, &cam_);   
}

//scroll callback
void MujocoBridge::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn_, &cam_);
}

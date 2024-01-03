#include <iostream>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <Eigen/Dense>

#ifndef MUJOCO_BRIDGE_H
#define MUJOCO_BRIDGE_H
class MujocoBridge
{
    public:
        MujocoBridge();
        ~MujocoBridge();
        void loadModel(const char* filename);
        void loadModel();
        void setCtrlFreq(const float_t hz);
        void init();
        bool isWindowOpen() { return !glfwWindowShouldClose(window_); }
        void updateUtil();
        void updateState();
        void updateInput();
        double getSimTime() {return d_->time; }
        double getSimTimeStep() {return m_->opt.timestep; }
        int getNumq() { return m_->nq; }
        int getNumv() { return m_->nv; }
        int getNumu() { return m_->nu; }
        Eigen::VectorXd getQpos();
        Eigen::VectorXd getQvel();
        void setCtrlInput(const Eigen::VectorXd & ctrl);


    private:
        GLFWwindow* window_; 

        // Mujoco data structures
        static mjModel* m_;    // MujoCo model
        static mjData* d_;	  // MujoCo Data
        static mjvCamera cam_;		  // abstract camera
        static mjvOption opt_;		  // visualisation options
        static mjvScene scn_;		  // abstract scene
        static mjrContext con_;		  // custom GPU context

        // mouse interaction
        static bool button_left_;
        static bool button_middle_;
        static bool button_right_;
        static double lastx_;
        static double lasty_;

        // holders of one step history of time and position to calculate dervatives
        mjtNum position_history_;
        mjtNum previous_time_;

        // controller related variables
        float_t ctrl_update_freq_;
        mjtNum last_update_;
        mjtNum ctrl_;

        // keyboard callback for OpenGL
        static void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);

        // mouse button callback
        static void mouse_button(GLFWwindow *window, int button, int act, int mods);

        // mouse move callback
        static void mouse_move(GLFWwindow *window, double xpos, double ypos);

        // scroll callback
        static void scroll(GLFWwindow *window, double xoffset, double yoffset);

};

#endif
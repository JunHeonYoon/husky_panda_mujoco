#ifndef __ROBOT_MODEL__
#define __ROBOT_MODEL__

#include <rbdl/rbdl.h>
// #include <rbdl/rbdl_utils.h>
// #include <rbdl/addons/urdfreader/urdfreader.h>
#include "fwd.h"
#include "utils/motion.h"
#include "data_container/data_container.h"

#include <iostream>

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace std;


namespace robot
{   
    // Type of robot (manipulator, mobilemanipulator)
    enum RobotType : unsigned int 
    {
        Manipulator,
        MobileManipulator
    };

    class RobotModel
    {
        public:
            RobotModel(RobotType robot_type);
            ~RobotModel();

            void getUpdateKinematics(const VectorXd & q, const VectorXd & qdot);
            const unsigned int & getNumq() {return nq_;}
            const unsigned int & getNumv() {return nv_;}
            const unsigned int & getNumu() {return nu_;}
            const MatrixXd & getJacobian(const int & frame_id) 
            {
				Jacobian(frame_id);
				return j_;
			}
            const Vector3d & getPosition(const int & frame_id) 
            {
				Position(frame_id);
				return x_;
			}
			const Matrix3d & getOrientation(const int & frame_id) 
            {
				Orientation(frame_id);
				return rotation_;
			}
            const Affine3d & getTransformation(const int & frame_id) 
            {
				Transformation(frame_id);
				return trans_;
			}
            const VectorXd & getJointPosition() 
            {
				return q_rbdl_;
            }
			const MatrixXd & getMassMatrix()
			{
				MassMatrix();
				return m_;
			}
			const VectorXd & getNonlinearEffect()
			{
				NonlinearEffect();
				return nle_;
			}
            const MatrixXd & getSelectionMatrix()
            {
				getUpdateKinematics(q_rbdl_, qdot_rbdl_);
				return selection_;
			}
            const MatrixXd & getvJacobian(const int & frame_id) 
            {
				Jacobian(frame_id);
				return jv_;
			}
            const MatrixXd & getvMassMatrix()
			{
				MassMatrix();
				return mv_;
			}
			const VectorXd & getvNonlinearEffect()
			{
				NonlinearEffect();
				return nlev_;
			}

            RobotType robot_type_;
            
        private:
            void setRobot();
            void setPanda(unsigned int base_id, Vector3d joint_position, Matrix3d joint_rotataion);
            void setHusky();
            void Jacobian(const int & frame_id);
			void Position(const int & frame_id);
			void Orientation(const int & frame_id);
			void Transformation(const int & frame_id);
			void MassMatrix();
			void NonlinearEffect();

            std::shared_ptr<Model> model_;
            unsigned int base_id_;     // for mobile base
            unsigned int body_id_[panda_num_links];  // only for manipulator (link0~7, hand)
            // Vector3d link_position_[panda_num_links]; // only for manipulator (link0~7, hand)

            // Current state
            VectorXd q_rbdl_;
			VectorXd qdot_rbdl_;
			VectorXd qddot_rbdl_;

            // Task space (End Effector)
            Vector3d x_;
            Matrix3d rotation_;
            Affine3d trans_;
			MatrixXd j_;           // Full basic Jacobian matrix
            MatrixXd j_v_;	       // Linear velocity Jacobian matrix
            MatrixXd j_w_;	       // Angular veolicty Jacobain matrix
            // MatrixXd j_inverse_;   // Jacobain inverse storage 

            // Dynamics
            MatrixXd m_;         // Mass matrix
            MatrixXd m_inverse_; // Inverse of mass matrix
            VectorXd nle_;       // Nonlinear Effect (Gravity torque + Coriolis torque)

            // mobile manipulator
            VectorXd v_rbdl_;       // joint velocity
            MatrixXd selection_;    // Selection matrix that mapping joint velocity(v) to general coordinate(qdot)
            MatrixXd selection_dot_; 
            MatrixXd jv_;           // jacobian in joint velocity space
            MatrixXd jv_v_;       
            MatrixXd jv_w_;
            MatrixXd mv_;           // Mass matrix in joint velocity space
            VectorXd nlev_;         // Nonlinear Effect (Gravity torque + Coriolis torque) in joint velocity space



        protected:
			unsigned int nq_; // size of state
			unsigned int nv_; // size of joint velocity
			unsigned int nu_; // size of control input
    };
}

#endif
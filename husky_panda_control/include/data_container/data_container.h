#ifndef DataContainer_H
#define DataContainer_H

#include <Eigen/Dense>
#include <iostream>


class DataContainer
{
    public:
        DataContainer(unsigned int nq, unsigned int nv, unsigned int nu)
        {
            q_.resize(nq);
            v_.resize(nv);
            tau_.resize(nv);
            u_desired_.resize(nu);
            q_.setZero();
            v_.setZero();
            tau_.setZero();
            u_desired_.setZero();
        }
        void getCurrentState(Eigen::VectorXd &q, Eigen::VectorXd &v,Eigen::VectorXd &tau)
        {
            q = q_; 
            v = v_;
            tau = tau_;
        }
        void setCurrentState(const Eigen::VectorXd &q, const Eigen::VectorXd &v, const Eigen::VectorXd &tau)
        {
            q_ = q; 
            v_ = v; 
            tau_ = tau; 
        }
        void getDesiredState(Eigen::VectorXd &u_desired)
        {
            u_desired = u_desired_; 
        }
        void setDesiredState(const Eigen::VectorXd &u_desired)
        {
            u_desired_ = u_desired; 
        }

        bool is_current_updated = false;
 
        private:
            // current state
            Eigen::VectorXd q_;
            Eigen::VectorXd v_;
            Eigen::VectorXd tau_;

            // control input
            Eigen::VectorXd u_desired_;
};

#endif
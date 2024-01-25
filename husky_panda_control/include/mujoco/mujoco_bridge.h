// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MUJOCO_BRIDGE_H
#define MUJOCO_BRIDGE_H

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <functional> 
#include <Eigen/Dense>

#include <mujoco/mujoco.h>
#include "mujoco/glfw_adapter.h"
#include "mujoco/simulate.h"
#include "mujoco/array_safety.h"
#include "utils/suhan_benchmark.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" 
{
    #include <sys/errno.h>
    #include <unistd.h>
}


namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;
using Seconds = std::chrono::duration<double>;

class MujocoBridge
{
    public:
        MujocoBridge(const char* filename);
        ~MujocoBridge();
        void setSimFreq(const float_t hz) { ctrl_update_freq_ = hz; }
        const float_t getSimFreq() { return ctrl_update_freq_; }
        void PhysicsThread(mj::Simulate* sim);
        double getSimTime() {return d->time; }
        int getNumq() { return m->nq; }
        int getNumv() { return m->nv; }
        int getNumu() { return m->nu; }
        Eigen::VectorXd getQpos();
        Eigen::VectorXd getQvel();
        Eigen::VectorXd getQforce();
        void setCtrlInput(const Eigen::VectorXd & ctrl);

        // simulate object encapsulates the UI
        std::unique_ptr<mujoco::Simulate> sim;
        bool is_init_ = false;
        bool is_model_loaded = false;

        // thread mutex
        std::mutex state_mutex; 
        std::mutex ctrl_mutex; 
    private:
        // void PhysicsThread(mj::Simulate* sim);
        void LoadModel(const char* filename);
        std::string getExecutableDir();
        void scanPluginLibraries();
        mjModel* LoadModel(const char* file, mj::Simulate& sim);
        void PhysicsLoop(mj::Simulate& sim);

        // constants
        const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
        const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
        static const int kErrorLength = 1024;   // load error string length

        // model file path
        const char * filename_;
        // Simulation Frequency
        float_t ctrl_update_freq_ = 500; // default

        // model and data
        mjModel* m = nullptr;
        mjData* d = nullptr;

        // mujoco structre
        mjvCamera cam;
        mjvOption opt;
        mjvPerturb pert;

        // control noise variables
        mjtNum* ctrlnoise = nullptr;

        Eigen::VectorXd ctrl_;
};

#endif
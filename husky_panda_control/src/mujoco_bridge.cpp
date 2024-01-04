#include "mujoco_bridge.h"


MujocoBridge::MujocoBridge(const char* filename)
{
    // print version, check compatibility
    std::printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER!=mj_version()) 
    {
        mju_error("Headers and library have different versions");
    }

    filename_ = filename;

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    mjv_defaultCamera(&cam);  
    mjv_defaultOption(&opt);
    mjv_defaultPerturb(&pert);

    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

}

MujocoBridge::~MujocoBridge()
{
    // delete everything we allocated
    free(ctrlnoise);
    mj_deleteData(d);
    mj_deleteModel(m);
}


void MujocoBridge::PhysicsThread(mj::Simulate *sim)
{

    LoadModel(filename_);
    PhysicsLoop(*sim);

}

Eigen::VectorXd MujocoBridge::getQpos()
{
    Eigen::VectorXd qpos;
    qpos.resize(MujocoBridge::getNumq());
    for (size_t i = 0; i < qpos.size(); i++)
    {
        qpos(i) = d->qpos[i];
    }
    return qpos;

}

Eigen::VectorXd MujocoBridge::getQvel()
{
    Eigen::VectorXd qvel;
    qvel.resize(MujocoBridge::getNumv());
    for (size_t i = 0; i < qvel.size(); i++)
    {
        qvel(i) = d->qvel[i];
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
        ctrl_ = ctrl;
    }
}

void MujocoBridge::LoadModel(const char *filename)
{
    // request loadmodel if file given (otherwise drag-and-drop)
    if (filename != nullptr) 
    {
        std::cout<<"File name: "<<filename<<std::endl;
        sim->LoadMessage(filename);
        m = LoadModel(filename, *sim);
        if (m)
        {
            std::cout<< "Model Loaded!"<<std::endl;
            ctrl_.resize(getNumu());
            ctrl_.setZero();
            m->opt.timestep = 1 / ctrl_update_freq_;
            d = mj_makeData(m);
        } 
        if (d) 
        {
            sim->Load(m, d, filename);
            mj_forward(m, d);

            // allocate ctrlnoise
            free(ctrlnoise);
            ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
            mju_zero(ctrlnoise, m->nu);
        } 
        else 
        {
            sim->LoadMessageClear();
        }
    }
}

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string MujocoBridge::getExecutableDir()
{
    constexpr char kPathSep = '/';
    const char* path = "/proc/self/exe";
    std::string realpath = [&]() -> std::string 
    {
        std::unique_ptr<char[]> realpath(nullptr);
        std::uint32_t buf_size = 128;
        bool success = false;
        while (!success) 
        {
            realpath.reset(new(std::nothrow) char[buf_size]);
            if (!realpath) 
            {
                std::cerr << "cannot allocate memory to store executable path\n";
                return "";
            }

            std::size_t written = readlink(path, realpath.get(), buf_size);
            if (written < buf_size) 
            {
                realpath.get()[written] = '\0';
                success = true;
            } 
            else if (written == -1) 
            {
                if (errno == EINVAL) 
                {
                    // path is already not a symlink, just use it
                    return path;
                }

                std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
                return "";
            } 
            else 
            {
                // realpath is too small, grow and retry
                buf_size *= 2;
            }
        }
        return realpath.get();
    }();

    if (realpath.empty()) 
    {
        return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i) 
    {
        if (realpath.c_str()[i] == kPathSep) 
        {
            return realpath.substr(0, i);
        }
    }

    // don't scan through the entire file system's root
    return "";
}

// scan for libraries in the plugin directory to load additional plugins
void MujocoBridge::scanPluginLibraries()
{
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin) 
    {
        std::printf("Built-in plugins:\n");
        for (int i = 0; i < nplugin; ++i)
        {
            std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
    }

    const std::string sep = "/";

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty()) 
    {
        return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries
    (
        plugin_dir.c_str(), +[](const char* filename, int first, int count) 
        {
            std::printf("Plugins registered by library '%s':\n", filename);
            for (int i = first; i < first + count; ++i) {
                std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
            }
        }
    );
}

mjModel* MujocoBridge::LoadModel(const char* file, mj::Simulate& sim)
{
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0]) 
    {
        return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel* mnew = 0;
    // TODO: Debug this part
    if (mju::strlen_arr(filename)>4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                        mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
        mnew = mj_loadModel(filename, nullptr);
        if (!mnew) 
        {
            mju::strcpy_arr(loadError, "could not load binary model");
        }
    } else {
        mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
        // remove trailing newline character from loadError
        if (loadError[0]) {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length-1] == '\n') {
            loadError[error_length-1] = '\0';
        }
        }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew) {
        std::printf("%s\n", loadError);
        return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0]) {
        // mj_forward() below will print the warning message
        std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
        sim.run = 0;
    }

    return mnew;
}

void MujocoBridge::PhysicsLoop(mj::Simulate &sim)
{
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;
  SuhanBenchmark timer;

  // run until asked to exit
  while (!sim.exitrequest.load()) 
  {
    timer.reset();
    if (sim.droploadrequest.load()) 
    {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) 
      {
        sim.Load(mnew, dnew, sim.dropfilename);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
        mju_zero(ctrlnoise, m->nu);
      } 
      else 
      {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) 
    {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) 
      {
        sim.Load(mnew, dnew, sim.filename);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        mju_zero(ctrlnoise, m->nu);
      } 
      else 
      {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) 
    {
      std::this_thread::yield();
    } 
    else 
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) 
      {
        // running
        if (sim.run) 
        {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std) 
            {
                // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
                mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
                mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1-rate*rate);

            for (int i=0; i<m->nu; i++) 
            {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
            }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed) 
            {
                // re-sync
                syncCPU = startCPU;
                syncSim = d->time;
                sim.speed_changed = false;

                // run single step, let next iteration deal with timing
                 // call mj_step
                state_mutex.lock();
                mj_step1(m, d);
                state_mutex.unlock();

                ctrl_mutex.lock();
                for(size_t i = 0; i < ctrl_.size(); i++)
                {
                    d->ctrl[i] = ctrl_[i];
                }
                mj_step2(m, d);
                ctrl_mutex.unlock();
                stepped = true;
            }

            // in-sync: step until ahead of cpu
            else 
            {
                bool measured = false;
                mjtNum prevSim = d->time;

                double refreshTime = simRefreshFraction/sim.refresh_rate;

                // step while sim lags behind cpu and within refreshTime
                while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                        mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) 
                {
                    // measure slowdown before first step
                    if (!measured && elapsedSim) 
                    {
                        sim.measured_slowdown =
                            std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                        measured = true;
                    }

                    // call mj_step
                    state_mutex.lock();
                    mj_step1(m, d);
                    state_mutex.unlock();

                    ctrl_mutex.lock();
                    for(size_t i = 0; i < ctrl_.size(); i++)
                    {
                        d->ctrl[i] = ctrl_[i];
                    }
                    mj_step2(m, d);
                    ctrl_mutex.unlock();
                    stepped = true;

                    // break if reset
                    if (d->time < prevSim) 
                    {
                        break;
                    }
                }
            }

            // save current state to history buffer
            if (stepped) 
            {
                sim.AddToHistory();
            }
        }

        // paused
        else 
        {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
    if(!is_init_) is_init_ = true;
//   std::cout<< "elapsed time[ms]: " << timer.elapsed()*1000 <<std::endl;
  }
  
}
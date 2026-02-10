//
// File rosnodeinterface.cpp
//
// Code generated for Simulink model 'jetRacer_ROS_Wrap'.
//
// Model version                  : 1.19
// Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
// C/C++ source code generated on : Tue Feb 10 18:00:28 2026
//

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)

#else

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"

#endif                                 //_MSC_VER

#include "jetRacer_ROS_Wrap.h"
#include "rosnodeinterface.h"
#include <thread>
#include <chrono>
#include <utility>
#undef ROS_SET_RTM_ERROR_STATUS
#undef ROS_GET_RTM_ERROR_STATUS
#undef ROS_RTM_STEP_TASK
#define ROS_SET_RTM_ERROR_STATUS(status) mModel->getRTM()->setErrorStatus(status)
#define ROS_GET_RTM_ERROR_STATUS()     mModel->getRTM()->getErrorStatus()
#define ROS_RTM_STEP_TASK(id)          mModel->getRTM()->StepTask(id)

namespace ros
{
  namespace matlab
  {
    NodeInterface::NodeInterface()
      : mNode()
      , mModel()
      , mBaseRateSem()
      , mBaseRateThread()
      , mSchedulerTimer()
      , mStopSem()
      , mRunModel(true)
    {
    }

    NodeInterface::~NodeInterface()
    {
      terminate();
    }

    void NodeInterface::initialize(int argc, char ** argv)
    {
      try {
        mNode = std::make_shared<ros::NodeHandle>();
        ROS_INFO("** Starting the model \"jetRacer_ROS_Wrap\" **\n");

        //initialize the model which will initialize the publishers and subscribers
        mModel = std::make_shared<jetRacer_ROS_Wrap>(
          );
        ROS_SET_RTM_ERROR_STATUS(NULL);
        mModel->initialize();

        // create the threads for the rates in the Model
        mBaseRateThread = std::make_shared<std::thread>(&NodeInterface::
          baseRateTask, this);

        // create scheduler timer to run the scheduler callback
        mSchedulerTimer = std::make_shared<ros::WallTimer>
          (mNode->createWallTimer(ros::WallDuration(20000000*1e-9),
            boost::bind(&NodeInterface::schedulerCallback, this, _1)));
        mSchedulerTimer->start();
      } catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
        throw ex;
      }
    }

    int NodeInterface::run()
    {
      ros::spin();
      mRunModel = false;
      return 0;
    }

    boolean_T NodeInterface::getStopRequestedFlag(void)
    {

#ifndef rtmGetStopRequested

      return (!(ROS_GET_RTM_ERROR_STATUS()
                == (NULL)));

#else

      return (!(ROS_GET_RTM_ERROR_STATUS()
                == (NULL)) || rtmGetStopRequested(mModel->getRTM()));

#endif

    }

    void NodeInterface::terminate(void)
    {
      if (mBaseRateThread.get()) {
        mBaseRateSem.notify();
        mBaseRateThread->join();
        mRunModel = false;
        mBaseRateThread.reset();
        if (mSchedulerTimer.get()) {
          mSchedulerTimer->stop();
          mSchedulerTimer.reset();
        }

        if (mModel.get()) {
          mModel->terminate();
        }

        mNode.reset();
      }
    }

    //
    // Scheduler Task using ROS Wall clock timer to run base-rate
    //
    void NodeInterface::schedulerCallback(const ros::WallTimerEvent& ev)
    {
      if (mRunModel) {
        mBaseRateSem.notify();
      }
    }

    // Base-rate task
    void NodeInterface::baseRateTask(void)
    {
      mRunModel = (ROS_GET_RTM_ERROR_STATUS() == (NULL));
      while (mRunModel) {
        mBaseRateSem.wait();

#ifdef MW_DEBUG_LOG

        ROS_INFO("** Base rate task semaphore received\n");

#endif

        if (!mRunModel)
          break;
        mModel->step();
        mRunModel = !NodeInterface::getStopRequestedFlag();
      }

      // Shutdown the ROS node
      ros::shutdown();
    }
  }                                    //namespace matlab
}                                      //namespace ros

#ifdef _MSC_VER

#pragma warning(pop)

#else

#pragma GCC diagnostic pop

#endif                                 //_MSC_VER

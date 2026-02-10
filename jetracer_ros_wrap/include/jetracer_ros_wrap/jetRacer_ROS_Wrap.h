//
// Prerelease License - for engineering feedback and testing purposes
// only. Not for sale.
//
// File: jetRacer_ROS_Wrap.h
//
// Code generated for Simulink model 'jetRacer_ROS_Wrap'.
//
// Model version                  : 1.20
// Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
// C/C++ source code generated on : Tue Feb 10 20:08:01 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef JETRACER_ROS_WRAP_H_
#define JETRACER_ROS_WRAP_H_
#include "rtwtypes.h"
#include "jetRacer_ROS_Wrap_types.h"
#include <stddef.h>

// Block signals (default storage)
struct B_jetRacer_ROS_Wrap_T {
  SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32 In1;// '<S6>/In1'
};

// Block states (default storage) for system '<Root>'
struct DW_jetRacer_ROS_Wrap_T {
  ros_slroscpp_internal_block_P_T obj; // '<S4>/SinkBlock'
  ros_slroscpp_internal_block_P_T obj_j;// '<S3>/SinkBlock'
  ros_slroscpp_internal_block_S_T obj_m;// '<S5>/SourceBlock'
  real32_T PrevY;                      // '<Root>/Rate Limiter'
};

// Real-time Model Data Structure
struct tag_RTM_jetRacer_ROS_Wrap_T {
  const char_T * volatile errorStatus;
  const char_T* getErrorStatus() const;
  void setErrorStatus(const char_T* const volatile aErrorStatus);
};

// Class declaration for model jetRacer_ROS_Wrap
class jetRacer_ROS_Wrap
{
  // public data and function members
 public:
  // Real-Time Model get method
  RT_MODEL_jetRacer_ROS_Wrap_T * getRTM();

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  jetRacer_ROS_Wrap();

  // Destructor
  ~jetRacer_ROS_Wrap();

  // private data and function members
 private:
  // Block signals
  B_jetRacer_ROS_Wrap_T jetRacer_ROS_Wrap_B;

  // Block states
  DW_jetRacer_ROS_Wrap_T jetRacer_ROS_Wrap_DW;

  // Real-Time Model
  RT_MODEL_jetRacer_ROS_Wrap_T jetRacer_ROS_Wrap_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'jetRacer_ROS_Wrap'
//  '<S1>'   : 'jetRacer_ROS_Wrap/Blank Message'
//  '<S2>'   : 'jetRacer_ROS_Wrap/Blank Message1'
//  '<S3>'   : 'jetRacer_ROS_Wrap/Publish'
//  '<S4>'   : 'jetRacer_ROS_Wrap/Publish1'
//  '<S5>'   : 'jetRacer_ROS_Wrap/Subscribe'
//  '<S6>'   : 'jetRacer_ROS_Wrap/Subscribe/Enabled Subsystem'

#endif                                 // JETRACER_ROS_WRAP_H_

//
// File trailer for generated code.
//
// [EOF]
//

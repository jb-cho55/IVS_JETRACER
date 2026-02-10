//
// Prerelease License - for engineering feedback and testing purposes
// only. Not for sale.
//
// File: jetRacer_ROS_Wrap.cpp
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
#include "jetRacer_ROS_Wrap.h"
#include "slros_initialize.h"
#include "rtwtypes.h"
#include "jetRacer_ROS_Wrap_types.h"

// Model step function
void jetRacer_ROS_Wrap::step()
{
  SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32 rtb_BusAssignment;
  SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32 rtb_BusAssignment1;
  SL_Bus_jetRacer_ROS_Wrap_std_msgs_Float32 rtb_SourceBlock_o2_0;
  real32_T rateLimiterRate;
  real32_T rtb_Saturation2;
  boolean_T b_varargout_1;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // MATLABSystem: '<S5>/SourceBlock'
  b_varargout_1 = Sub_jetRacer_ROS_Wrap_4.getLatestMessage(&rtb_SourceBlock_o2_0);

  // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S6>/Enable'

  // Start for MATLABSystem: '<S5>/SourceBlock'
  if (b_varargout_1) {
    // SignalConversion generated from: '<S6>/In1'
    jetRacer_ROS_Wrap_B.In1 = rtb_SourceBlock_o2_0;
  }

  // End of Start for MATLABSystem: '<S5>/SourceBlock'
  // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // Gain: '<Root>/Gain'
  rtb_Saturation2 = 0.04F * jetRacer_ROS_Wrap_B.In1.Data;

  // Saturate: '<Root>/Saturation'
  if (rtb_Saturation2 > 1.0F) {
    rtb_Saturation2 = 1.0F;
  } else if (rtb_Saturation2 < -1.0F) {
    rtb_Saturation2 = -1.0F;
  }

  // Gain: '<Root>/Gain1' incorporates:
  //   Saturate: '<Root>/Saturation'

  rtb_Saturation2 *= 0.7F;

  // Saturate: '<Root>/Saturation1'
  if (rtb_Saturation2 > 0.6F) {
    rtb_Saturation2 = 0.6F;
  } else if (rtb_Saturation2 < -0.6F) {
    rtb_Saturation2 = -0.6F;
  }

  // End of Saturate: '<Root>/Saturation1'

  // RateLimiter: '<Root>/Rate Limiter'
  rateLimiterRate = rtb_Saturation2 - jetRacer_ROS_Wrap_DW.PrevY;
  if (rateLimiterRate > 0.016F) {
    rtb_Saturation2 = jetRacer_ROS_Wrap_DW.PrevY + 0.016F;
  } else if (rateLimiterRate < -0.016F) {
    rtb_Saturation2 = jetRacer_ROS_Wrap_DW.PrevY - 0.016F;
  }

  jetRacer_ROS_Wrap_DW.PrevY = rtb_Saturation2;

  // End of RateLimiter: '<Root>/Rate Limiter'

  // Saturate: '<Root>/Saturation2' incorporates:
  //   Sum: '<Root>/Add'

  if (rtb_Saturation2 - 0.3F > 1.0F) {
    // BusAssignment: '<Root>/Bus Assignment'
    rtb_BusAssignment.Data = 1.0F;
  } else if (rtb_Saturation2 - 0.3F < -1.0F) {
    // BusAssignment: '<Root>/Bus Assignment'
    rtb_BusAssignment.Data = -1.0F;
  } else {
    // BusAssignment: '<Root>/Bus Assignment'
    rtb_BusAssignment.Data = rtb_Saturation2 - 0.3F;
  }

  // End of Saturate: '<Root>/Saturation2'

  // Outputs for Atomic SubSystem: '<Root>/Publish'
  // MATLABSystem: '<S3>/SinkBlock'
  Pub_jetRacer_ROS_Wrap_2.publish(&rtb_BusAssignment);

  // End of Outputs for SubSystem: '<Root>/Publish'

  // BusAssignment: '<Root>/Bus Assignment1' incorporates:
  //   Constant: '<Root>/Constant'

  rtb_BusAssignment1.Data = 0.2F;

  // Outputs for Atomic SubSystem: '<Root>/Publish1'
  // MATLABSystem: '<S4>/SinkBlock'
  Pub_jetRacer_ROS_Wrap_14.publish(&rtb_BusAssignment1);

  // End of Outputs for SubSystem: '<Root>/Publish1'
}

// Model initialize function
void jetRacer_ROS_Wrap::initialize()
{
  {
    int32_T i;
    char_T b_zeroDelimTopic[14];
    static const char_T b_zeroDelimTopic_0[14] = "/steering_raw";
    static const char_T b_zeroDelimTopic_1[14] = "/steering_cmd";
    static const char_T b_zeroDelimTopic_2[14] = "/throttle_cmd";

    // SystemInitialize for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S5>/SourceBlock'
    jetRacer_ROS_Wrap_DW.obj_m.matlabCodegenIsDeleted = false;
    jetRacer_ROS_Wrap_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
    }

    Sub_jetRacer_ROS_Wrap_4.createSubscriber(&b_zeroDelimTopic[0], 1);
    jetRacer_ROS_Wrap_DW.obj_m.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SourceBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Subscribe'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish'
    // Start for MATLABSystem: '<S3>/SinkBlock'
    jetRacer_ROS_Wrap_DW.obj_j.matlabCodegenIsDeleted = false;
    jetRacer_ROS_Wrap_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_1[i];
    }

    Pub_jetRacer_ROS_Wrap_2.createPublisher(&b_zeroDelimTopic[0], 1);
    jetRacer_ROS_Wrap_DW.obj_j.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S3>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish'

    // SystemInitialize for Atomic SubSystem: '<Root>/Publish1'
    // Start for MATLABSystem: '<S4>/SinkBlock'
    jetRacer_ROS_Wrap_DW.obj.matlabCodegenIsDeleted = false;
    jetRacer_ROS_Wrap_DW.obj.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      b_zeroDelimTopic[i] = b_zeroDelimTopic_2[i];
    }

    Pub_jetRacer_ROS_Wrap_14.createPublisher(&b_zeroDelimTopic[0], 1);
    jetRacer_ROS_Wrap_DW.obj.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S4>/SinkBlock'
    // End of SystemInitialize for SubSystem: '<Root>/Publish1'
  }
}

// Model terminate function
void jetRacer_ROS_Wrap::terminate()
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  if (!jetRacer_ROS_Wrap_DW.obj_m.matlabCodegenIsDeleted) {
    jetRacer_ROS_Wrap_DW.obj_m.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S5>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Atomic SubSystem: '<Root>/Publish'
  // Terminate for MATLABSystem: '<S3>/SinkBlock'
  if (!jetRacer_ROS_Wrap_DW.obj_j.matlabCodegenIsDeleted) {
    jetRacer_ROS_Wrap_DW.obj_j.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S3>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish'

  // Terminate for Atomic SubSystem: '<Root>/Publish1'
  // Terminate for MATLABSystem: '<S4>/SinkBlock'
  if (!jetRacer_ROS_Wrap_DW.obj.matlabCodegenIsDeleted) {
    jetRacer_ROS_Wrap_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S4>/SinkBlock'
  // End of Terminate for SubSystem: '<Root>/Publish1'
}

// Constructor
jetRacer_ROS_Wrap::jetRacer_ROS_Wrap() :
  jetRacer_ROS_Wrap_B(),
  jetRacer_ROS_Wrap_DW(),
  jetRacer_ROS_Wrap_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
jetRacer_ROS_Wrap::~jetRacer_ROS_Wrap()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_jetRacer_ROS_Wrap_T * jetRacer_ROS_Wrap::getRTM()
{
  return (&jetRacer_ROS_Wrap_M);
}

const char_T* RT_MODEL_jetRacer_ROS_Wrap_T::getErrorStatus() const
{
  return (errorStatus);
}

void RT_MODEL_jetRacer_ROS_Wrap_T::setErrorStatus(const char_T* const volatile
  aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

//
// File trailer for generated code.
//
// [EOF]
//

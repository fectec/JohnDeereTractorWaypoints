/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: robocar_c.h
 *
 * Code generated for Simulink model 'robocar_c'.
 *
 * Model version                  : 2.43
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Nov 29 12:10:28 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef robocar_c_h_
#define robocar_c_h_
#ifndef robocar_c_COMMON_INCLUDES_
#define robocar_c_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* robocar_c_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap
#define typedef_cell_wrap

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap;

#endif                                 /* typedef_cell_wrap */

#ifndef struct_tag_DwkamkaJN4l1qSsZxTESCB
#define struct_tag_DwkamkaJN4l1qSsZxTESCB

struct tag_DwkamkaJN4l1qSsZxTESCB
{
  int32_T isInitialized;
  cell_wrap inputVarSize[4];
  real_T MaxAngularVelocity;
  real_T LookaheadDistance;
  real_T DesiredLinearVelocity;
  real_T ProjectionPoint[2];
  real_T ProjectionLineIndex;
  real_T LookaheadPoint[2];
  real_T LastPose[3];
  real_T WaypointsInternal[8];
};

#endif                                 /* struct_tag_DwkamkaJN4l1qSsZxTESCB */

#ifndef typedef_nav_slalgs_internal_PurePursuit
#define typedef_nav_slalgs_internal_PurePursuit

typedef struct tag_DwkamkaJN4l1qSsZxTESCB nav_slalgs_internal_PurePursuit;

#endif                             /* typedef_nav_slalgs_internal_PurePursuit */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  nav_slalgs_internal_PurePursuit obj; /* '<Root>/Pure Pursuit' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Input[3];                     /* '<Root>/Input' */
  real_T Input1;                       /* '<Root>/Input1' */
  real_T Input2;                       /* '<Root>/Input2' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
  real_T Out2;                         /* '<Root>/Out2' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void robocar_c_initialize(void);
extern void robocar_c_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'robocar_c'
 * '<S1>'   : 'robocar_c/MATLAB Function2'
 */
#endif                                 /* robocar_c_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

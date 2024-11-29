/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: robocar_c.c
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

#include "robocar_c.h"
#include "rtwtypes.h"
#include <math.h>
#include "math.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_atan2d_snf(real_T u0, real_T u1);

/* Forward declaration for local functions */
static real_T norm(const real_T x[2]);
static real_T closestPointOnLine(const real_T pt1[2], real_T pt2[2], const
  real_T refPt[2]);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      /* do nothing */
#else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
real_T rtNaN = -(real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = -(real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

/* Return rtNaN needed by the generated code. */
static real_T rtGetNaN(void)
{
  return rtNaN;
}

/* Return rtNaNF needed by the generated code. */
static real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

static real_T norm(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

static real_T closestPointOnLine(const real_T pt1[2], real_T pt2[2], const
  real_T refPt[2])
{
  real_T refPt_0[2];
  real_T alpha;
  real_T distance;
  real_T v12;
  real_T v12_0;
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(pt1[b_k] == pt2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (p) {
    pt2[0] = pt1[0];
    refPt_0[0] = refPt[0] - pt1[0];
    pt2[1] = pt1[1];
    refPt_0[1] = refPt[1] - pt1[1];
    distance = norm(refPt_0);
  } else {
    alpha = pt2[0] - pt1[0];
    v12 = (pt2[0] - refPt[0]) * alpha;
    v12_0 = alpha * alpha;
    alpha = pt2[1] - pt1[1];
    alpha = ((pt2[1] - refPt[1]) * alpha + v12) / (alpha * alpha + v12_0);
    if (alpha > 1.0) {
      pt2[0] = pt1[0];
      pt2[1] = pt1[1];
    } else if (!(alpha < 0.0)) {
      pt2[0] = (1.0 - alpha) * pt2[0] + alpha * pt1[0];
      pt2[1] = (1.0 - alpha) * pt2[1] + alpha * pt1[1];
    }

    refPt_0[0] = refPt[0] - pt2[0];
    refPt_0[1] = refPt[1] - pt2[1];
    distance = norm(refPt_0);
  }

  /* End of Start for MATLABSystem: '<Root>/Pure Pursuit' */
  return distance;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void robocar_c_step(void)
{
  real_T lookaheadStartPt[2];
  real_T rtb_path_0[2];
  real_T rtb_path_1[2];
  real_T dist;
  real_T lookaheadEndPt_idx_0;
  real_T lookaheadEndPt_idx_1;
  real_T lookaheadIdx;
  real_T minDistance;
  int32_T b_k;
  int32_T rtb_path_tmp_tmp;
  int32_T rtb_path_tmp_tmp_0;
  int32_T trueCount;
  int8_T rtb_path[8];
  int8_T tmp_data[4];
  int8_T rtb_path_tmp;
  int8_T rtb_path_tmp_0;
  boolean_T p;
  boolean_T searchFlag;
  static const int8_T tmp[8] = { 4, 8, 12, 16, 3, -3, 3, -3 };

  int32_T tmp_size_idx_0;
  boolean_T exitg1;

  /* MATLAB Function: '<Root>/MATLAB Function2' */
  for (b_k = 0; b_k < 8; b_k++) {
    rtb_path[b_k] = tmp[b_k];
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function2' */

  /* MATLABSystem: '<Root>/Pure Pursuit' incorporates:
   *  Inport: '<Root>/Input1'
   *  Inport: '<Root>/Input2'
   */
  if (rtDW.obj.MaxAngularVelocity != 1.0) {
    rtDW.obj.MaxAngularVelocity = 1.0;
  }

  if ((rtDW.obj.DesiredLinearVelocity == rtU.Input1) || (rtIsNaN
       (rtDW.obj.DesiredLinearVelocity) && rtIsNaN(rtU.Input1))) {
  } else {
    rtDW.obj.DesiredLinearVelocity = rtU.Input1;
  }

  if ((rtDW.obj.LookaheadDistance == rtU.Input2) || (rtIsNaN
       (rtDW.obj.LookaheadDistance) && rtIsNaN(rtU.Input2))) {
  } else {
    rtDW.obj.LookaheadDistance = rtU.Input2;
  }

  searchFlag = false;
  p = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 8)) {
    if (rtDW.obj.WaypointsInternal[b_k] == rtb_path[b_k]) {
      b_k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }

  if (p) {
    searchFlag = true;
  }

  if (!searchFlag) {
    for (b_k = 0; b_k < 8; b_k++) {
      rtDW.obj.WaypointsInternal[b_k] = rtb_path[b_k];
    }

    rtDW.obj.ProjectionLineIndex = 0.0;
  }

  trueCount = 0;
  for (b_k = 0; b_k < 4; b_k++) {
    trueCount++;
  }

  tmp_size_idx_0 = trueCount;
  trueCount = 0;
  for (b_k = 0; b_k < 4; b_k++) {
    /* MATLABSystem: '<Root>/Pure Pursuit' */
    tmp_data[trueCount] = (int8_T)b_k;
    trueCount++;
  }

  /* MATLABSystem: '<Root>/Pure Pursuit' incorporates:
   *  Inport: '<Root>/Input'
   * */
  if (tmp_size_idx_0 == 0) {
    /* Outport: '<Root>/Out1' */
    rtY.Out1 = 0.0;
    minDistance = 0.0;
  } else {
    searchFlag = false;
    if (rtDW.obj.ProjectionLineIndex == 0.0) {
      searchFlag = true;
      rtDW.obj.ProjectionPoint[0] = rtb_path[tmp_data[0]];
      rtDW.obj.ProjectionPoint[1] = rtb_path[tmp_data[0] + 4];
      rtDW.obj.ProjectionLineIndex = 1.0;
    }

    if (tmp_size_idx_0 == 1) {
      lookaheadEndPt_idx_0 = rtb_path[tmp_data[0]];
      rtDW.obj.ProjectionPoint[0] = lookaheadEndPt_idx_0;
      lookaheadEndPt_idx_1 = rtb_path[tmp_data[0] + 4];
      rtDW.obj.ProjectionPoint[1] = lookaheadEndPt_idx_1;
    } else {
      b_k = tmp_data[(int32_T)(rtDW.obj.ProjectionLineIndex + 1.0) - 1];
      lookaheadStartPt[0] = rtb_path[b_k];
      lookaheadStartPt[1] = rtb_path[b_k + 4];
      minDistance = closestPointOnLine(rtDW.obj.ProjectionPoint,
        lookaheadStartPt, &rtU.Input[0]);
      rtDW.obj.ProjectionPoint[0] = lookaheadStartPt[0];
      rtb_path_0[0] = lookaheadStartPt[0] - (real_T)rtb_path[tmp_data[(int32_T)
        (rtDW.obj.ProjectionLineIndex + 1.0) - 1]];
      rtDW.obj.ProjectionPoint[1] = lookaheadStartPt[1];
      rtb_path_0[1] = lookaheadStartPt[1] - (real_T)rtb_path[tmp_data[(int32_T)
        (rtDW.obj.ProjectionLineIndex + 1.0) - 1] + 4];
      dist = norm(rtb_path_0);
      lookaheadIdx = rtDW.obj.ProjectionLineIndex + 1.0;
      b_k = (int32_T)((1.0 - (rtDW.obj.ProjectionLineIndex + 1.0)) + ((real_T)
        tmp_size_idx_0 - 1.0)) - 1;
      trueCount = 0;
      exitg1 = false;
      while ((!exitg1) && (trueCount <= b_k)) {
        lookaheadEndPt_idx_0 = lookaheadIdx + (real_T)trueCount;
        if ((!searchFlag) && (dist > rtDW.obj.LookaheadDistance)) {
          exitg1 = true;
        } else {
          rtb_path_tmp_tmp_0 = tmp_data[(int32_T)(lookaheadEndPt_idx_0 + 1.0) -
            1];
          rtb_path_tmp = rtb_path[rtb_path_tmp_tmp_0];
          rtb_path_tmp_tmp = tmp_data[(int32_T)lookaheadEndPt_idx_0 - 1];
          rtb_path_tmp_0 = rtb_path[rtb_path_tmp_tmp];
          rtb_path_0[0] = rtb_path_tmp_0 - rtb_path_tmp;
          lookaheadStartPt[0] = rtb_path_tmp;
          rtb_path_1[0] = rtb_path_tmp_0;
          rtb_path_tmp = rtb_path[rtb_path_tmp_tmp_0 + 4];
          rtb_path_tmp_0 = rtb_path[rtb_path_tmp_tmp + 4];
          rtb_path_0[1] = rtb_path_tmp_0 - rtb_path_tmp;
          lookaheadStartPt[1] = rtb_path_tmp;
          rtb_path_1[1] = rtb_path_tmp_0;
          dist += norm(rtb_path_0);
          lookaheadEndPt_idx_1 = closestPointOnLine(rtb_path_1, lookaheadStartPt,
            &rtU.Input[0]);
          if (lookaheadEndPt_idx_1 < minDistance) {
            minDistance = lookaheadEndPt_idx_1;
            rtDW.obj.ProjectionPoint[0] = lookaheadStartPt[0];
            rtDW.obj.ProjectionPoint[1] = lookaheadStartPt[1];
            rtDW.obj.ProjectionLineIndex = lookaheadEndPt_idx_0;
          }

          trueCount++;
        }
      }

      b_k = tmp_data[(int32_T)(rtDW.obj.ProjectionLineIndex + 1.0) - 1];
      lookaheadEndPt_idx_0 = rtb_path[b_k];
      rtb_path_0[0] = rtDW.obj.ProjectionPoint[0] - lookaheadEndPt_idx_0;
      lookaheadEndPt_idx_1 = rtb_path[b_k + 4];
      rtb_path_0[1] = rtDW.obj.ProjectionPoint[1] - lookaheadEndPt_idx_1;
      dist = norm(rtb_path_0);
      lookaheadStartPt[0] = rtDW.obj.ProjectionPoint[0];
      lookaheadStartPt[1] = rtDW.obj.ProjectionPoint[1];
      minDistance = dist - rtDW.obj.LookaheadDistance;
      lookaheadIdx = rtDW.obj.ProjectionLineIndex;
      while ((minDistance < 0.0) && (lookaheadIdx < (real_T)tmp_size_idx_0 - 1.0))
      {
        lookaheadIdx++;
        b_k = tmp_data[(int32_T)lookaheadIdx - 1];
        rtb_path_tmp = rtb_path[b_k];
        lookaheadStartPt[0] = rtb_path_tmp;
        trueCount = tmp_data[(int32_T)(lookaheadIdx + 1.0) - 1];
        rtb_path_tmp_0 = rtb_path[trueCount];
        lookaheadEndPt_idx_0 = rtb_path_tmp_0;
        rtb_path_0[0] = rtb_path_tmp - rtb_path_tmp_0;
        rtb_path_tmp = rtb_path[b_k + 4];
        lookaheadStartPt[1] = rtb_path_tmp;
        rtb_path_tmp_0 = rtb_path[trueCount + 4];
        lookaheadEndPt_idx_1 = rtb_path_tmp_0;
        rtb_path_0[1] = rtb_path_tmp - rtb_path_tmp_0;
        dist += norm(rtb_path_0);
        minDistance = dist - rtDW.obj.LookaheadDistance;
      }

      rtb_path_0[0] = lookaheadStartPt[0] - lookaheadEndPt_idx_0;
      rtb_path_0[1] = lookaheadStartPt[1] - lookaheadEndPt_idx_1;
      dist = minDistance / norm(rtb_path_0);
      if (dist > 0.0) {
        lookaheadEndPt_idx_0 = (1.0 - dist) * lookaheadEndPt_idx_0 + dist *
          lookaheadStartPt[0];
        lookaheadEndPt_idx_1 = (1.0 - dist) * lookaheadEndPt_idx_1 + dist *
          lookaheadStartPt[1];
      }
    }

    rtDW.obj.LookaheadPoint[0] = lookaheadEndPt_idx_0;
    rtDW.obj.LookaheadPoint[1] = lookaheadEndPt_idx_1;
    dist = rt_atan2d_snf(rtDW.obj.LookaheadPoint[1] - rtU.Input[1],
                         rtDW.obj.LookaheadPoint[0] - rtU.Input[0]) - rtU.Input
      [2];
    if (fabs(dist) > 3.1415926535897931) {
      if (rtIsNaN(dist + 3.1415926535897931) || rtIsInf(dist +
           3.1415926535897931)) {
        minDistance = (rtNaN);
      } else if (dist + 3.1415926535897931 == 0.0) {
        minDistance = 0.0;
      } else {
        minDistance = fmod(dist + 3.1415926535897931, 6.2831853071795862);
        searchFlag = (minDistance == 0.0);
        if (!searchFlag) {
          lookaheadIdx = fabs((dist + 3.1415926535897931) / 6.2831853071795862);
          searchFlag = !(fabs(lookaheadIdx - floor(lookaheadIdx + 0.5)) >
                         2.2204460492503131E-16 * lookaheadIdx);
        }

        if (searchFlag) {
          minDistance = 0.0;
        } else if (minDistance < 0.0) {
          minDistance += 6.2831853071795862;
        }
      }

      if ((minDistance == 0.0) && (dist + 3.1415926535897931 > 0.0)) {
        minDistance = 6.2831853071795862;
      }

      dist = minDistance - 3.1415926535897931;
    }

    minDistance = 2.0 * sin(dist) / rtDW.obj.LookaheadDistance;
    if (rtIsNaN(minDistance)) {
      minDistance = 0.0;
    }

    if (fabs(fabs(dist) - 3.1415926535897931) < 1.4901161193847656E-8) {
      if (rtIsNaN(minDistance)) {
        minDistance = (rtNaN);
      } else if (minDistance < 0.0) {
        minDistance = -1.0;
      } else {
        minDistance = (minDistance > 0.0);
      }
    }

    if (fabs(minDistance) > 1.0) {
      if (rtIsNaN(minDistance)) {
        minDistance = (rtNaN);
      } else if (minDistance < 0.0) {
        minDistance = -1.0;
      } else {
        minDistance = (minDistance > 0.0);
      }
    }

    /* Outport: '<Root>/Out1' incorporates:
     *  Inport: '<Root>/Input'
     * */
    rtY.Out1 = rtDW.obj.DesiredLinearVelocity;
    rtDW.obj.LastPose[0] = rtU.Input[0];
    rtDW.obj.LastPose[1] = rtU.Input[1];
    rtDW.obj.LastPose[2] = rtU.Input[2];
  }

  /* Outport: '<Root>/Out2' incorporates:
   *  MATLABSystem: '<Root>/Pure Pursuit'
   */
  rtY.Out2 = minDistance;
}

/* Model initialize function */
void robocar_c_initialize(void)
{
  {
    int32_T i;

    /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
    rtDW.obj.MaxAngularVelocity = 1.0;
    rtDW.obj.isInitialized = 1;
    rtDW.obj.DesiredLinearVelocity = 0.0;
    rtDW.obj.LookaheadDistance = 0.0;
    for (i = 0; i < 8; i++) {
      rtDW.obj.WaypointsInternal[i] = (rtNaN);
    }

    /* InitializeConditions for MATLABSystem: '<Root>/Pure Pursuit' */
    rtDW.obj.LookaheadPoint[0] = 0.0;
    rtDW.obj.LookaheadPoint[1] = 0.0;
    rtDW.obj.LastPose[0] = 0.0;
    rtDW.obj.LastPose[1] = 0.0;
    rtDW.obj.LastPose[2] = 0.0;
    rtDW.obj.ProjectionPoint[0] = (rtNaN);
    rtDW.obj.ProjectionPoint[1] = (rtNaN);
    rtDW.obj.ProjectionLineIndex = 0.0;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

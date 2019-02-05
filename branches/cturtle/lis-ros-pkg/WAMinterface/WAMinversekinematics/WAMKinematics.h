/*======================================================================*
 *  Module .............Kinematics
 *  File ...............WAMKinematics.h
 *  Author .............Manfred Huber
 *  Creation Date ......August 2008
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES: Kinematic functions and definitions for the 7 DOF WAM
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

/** \file WAMKinematics.h
   \brief Kinematic functions and definitions for the 7 DOF WAM

*/

#ifndef _WAMKINEMATICS_H
#define _WAMKINEMATICS_H

#include <math.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
 
/* Robot definitions */

#define JNTS 7         /* Number of degrees of freedom for WAM */
#define LINKS 3        /* Number of links of the WAM */
#define HJNTS 4        /* Number of degrees of freedom for Barrett Hand */
#define FINGERS 3      /* Number of fingers of Barrett Hand */
#define FJNTS 3        /* Number of joints per finger of Barrett Hand */
#define AXES 3         /* Force axes */

/* 
** link lengths and offsets (m)
*/ 
#define WAM_BASE_X          (0.22)   //Base to Arm frame
#define WAM_BASE_Y          (0.14)   //Base to Arm frame
#define WAM_BASE_Z          (0.346)  //Base to Arm frame
#define WAM_J3_OFFSET_X     (0.045)  //Barrett joint 3 X offset
#define WAM_L1              (0.55)   //Barrett first link length
#define WAM_L2              (0.3)    //Barrett second link length (first wrist part)
#define WAM_L3              (0.0609)   //Barrett third link length (second wrist part)

static double wam_theta_min[JNTS] = {-2.66, -1.942, -2.704, -0.842, -4.811, -1.633, -2.2};
static double wam_theta_max[JNTS] = { 2.66,  1.962,  2.904,  3.117,  1.261,  1.508,  2.2};

void DispRotz(double disp[3], double theta, double trans[4][4]);
void DispRoty(double disp[3], double theta, double trans[4][4]);
void HomTransMult(double trans1[4][4], double trans2[4][4], double result[4][4]);

void WAMForwardKinematics(double theta[JNTS], double x[4][4], double elbwrist[2][4][4]);

int WAMInverseKinematics(double theta2, double x[4][4], double theta_ref[JNTS], double theta[JNTS]);

#ifdef __cplusplus
}
#endif/* __cplusplus */

#endif /*_WAMKINEMATICS_H*/

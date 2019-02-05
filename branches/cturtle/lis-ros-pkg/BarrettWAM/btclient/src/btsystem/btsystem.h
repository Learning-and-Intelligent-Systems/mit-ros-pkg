/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btsystem.h
 *  Creation Date ...... 15 Feb 2003
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2003-2008 Barrett Technology, Inc. <support@barrett.com>
 *                          625 Mount Auburn St
 *                          Cambridge, MA 02138, USA
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of Barrett Technology.
 *                                                                          *
 * ======================================================================== */

/** \file btsystem.h
  \brief Access and use a collection of pucks
  
  btsystem is meant to give an api to a random set of pucks and motors on multiple busses.
  these are abstracted as "actuators"
  
  \internal !ToDo! much of the bus information should be re-written to be dynamically 
  allocated.
  
*/
 
#ifndef _BTSYSTEM_H
#define _BTSYSTEM_H

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <stdio.h>
#include <inttypes.h>
/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btcan.h"

/*==============================*
 * PRIVATE DEFINED constants    *
 *==============================*/
#define MAX_BUSES (2) 

enum{CAN,ETHERNET};

/** Information related to motor controllers (pucks).
This structure contains information that is specific to each puck. This is seperated from motor information 
for readability purposes. Refer to actuator_struct for details.
*/

typedef struct 
{
  int serial; /*!< Serial number of the puck */
  int ID; /*!< The CAN bus ID number*/
  int IoffsetB;/*!< Offset to calibrate zero for current sensors*/
  int IoffsetC; /*!< Offset to calibrate zero for current sensors*/
  int group;
  int order; /*!< Must be in the range 0..3 to be valid*/
  int index,zero; 
  int max_torque;
  long int position; /*!< The position value read from the puck (encoder counts) */
  int poffset; /*!< (poffset + position) % counter_per_rev = encoder_counts_from_index*/
  int enc_pos; /*!< number of counts between positions 0 and the index pulse */
  int remainded_angle; 
  int torque_cmd; /*!< The last torque value sent to the puck (puck units) */
}puck_struct; 


/** Information related to motors
  This structure contains information that is specific to each physical 
  motor. This is seperated from puck information
  for code readibility purposes. Refer to actuator_struct for details.
*/
typedef struct 
{
  int serial; /*!< Serial number of motor */
  int commutation_offset; /*!< The angle in encoder ticks between the index pulse and commutation zero.*/
  int counts_per_rev; /*!< Encoder counts per revolution*/
  double puckI_per_Nm; /*!< Coefficient of conversion from Newton Meters to puck torque units*/
  int *values; /*!< A pointer to a list of Torque Ripple Compensation data. */
  int num_values; /*!< The number of TRC values*/
  double ratio; /*!< The number of TRC values per motor tick*/
}motor_struct; 

/** Aggregate of information needed to control motors.

  actuator_struct is the core data structure of the btsystem module. actuator_struct maintains a database
  of the actuators (motor/amplifier pairs) connected to this PC. The static information for this database is stored in 
  several text files. 
  
  The core interaction with actuators is reading positions and setting torques. actuator_struct maintains the last position read
  in .angle and uses the value in .torque to set the torque. Additional processing to these values (for instance converting from engineering
  units to puck units) is documented in the GetPostitions and SetTorques functions.
  
  The .puck and .motor structures maintain a set of static data.
  
  None of the variables in actuator_struct should be manipulated directly except for .angle and .torque. Note that this is not thread-safe and
  should be done only with care.
*/
typedef struct
{
  int bus; /*!< The CAN bus this actuator is on. This is an index into the buses array. */
  
  puck_struct puck;
  motor_struct motor;  
  
  double angle; /*!< The angle of the actuator. This is either radians (UseEngrUnits = 1) or encoder ticks (UseEngrUnits = 0) */
  double torque; /*!< The torque of the actuator. This is either Nm (UseEngrUnits = 1) or puck torque units (UseEngrUnits = 0) */
  
  int UseEngrUnits; /*!< 0 -> Use puck units 1 -> Use engineering units */
  int UseTRC; /*!< 0 -> Do not use torque ripple cancellation 1 -> Use torque ripple cancellation if possible */
  uint64_t lastpos,lasttrq;  //timing loop performance variables
}actuator_struct;

/** (internal) For future use */
typedef struct
{
  int ID;
  int bus;
}safetymodule_struct;

/** For each group we have an index of actuators, sorted by puckID. 
  See bus_struct
  */
typedef struct
{
  int group_number; /*!< The ID number for this group.*/
  int num_pucks; /*!< The number of pucks in this group. */
  int pucks_by_order[4]; /*!< An array of puck id numbers. */
}group_struct;

/** Bus topology structure.
  This structure keeps track information on each can bus connected to the PC.
  It has two indexes into the actuator list for efficient 
  processing of broadcast commands. This structure is maintained
  internally and should not be used unless low-level CAN communications
  need to be performed. An example is a call to wakepucks() which needs a pointer to
  the CANdev_t information.
  
  \code
  wakePuck(&(buses[busidx].CANdev), act[cnt].puck.ID);
  \endcode
  
  
*/
typedef struct
{
  int type; //enum = {CAN = 0,ETHERNET}
  char device_str[100]; /*!< A text string that describes this bus for users*/
  unsigned long address; // CAN id or dotted quad of safety system
  char device_name[50];
  
  //  CANdev_t CANdev; /*!< Information on the can device used for this bus */
  int pucks_by_id[65]; /*!< Indexes of actuators sorted by puck ID */
  int num_pucks; /*!< Number of indexes contained in pucks_by_id */
  int total_pucks; /*!< Number of pucks on the bus (may be more than were specified in the data files)*/
  group_struct group[65]; /*!< Each group has an index. See group_struct */
  int num_groups; /*!< Number of groups in the group array*/
}bus_struct;





typedef struct{
  int num_actuators; 
  actuator_struct *act;
}btsystem;
/*-------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
//Top level functions-----------------------------
int DiscoverSystem(void); // Automatically discover the robotic systems connected to this computer
int ReadSystemFromConfig(char *fn, int *busCount); // Use a config file to determine the robotic systems connected to this computer
#ifdef BTOLDCONFIG
int InitializeSystem(char *actuatorfile,char *busfile,char *motorfile,char *puckfile); //Global bus variables, global actuator variable, load data from files, open can bus
#else
int InitializeSystem(void);
#endif

void CloseSystem(); //
  //private 
void DumpData2Syslog();  
//actuator_struct * GetActuators(int *Num_actuators); // returns a pointer to the actuator data
int GetActuators(int bus, actuator_struct **a, int *Num_actuators);
bus_struct * GetBuses(int *Num_buses);

//COMM functions-----------------------------
long GetProp(int actuator_id, int property);
int SetProp(int actuator_id, int property, long data);
int SetByID(int bus, int CANid, int property, long data);

int EnumerateSystem();

//control functions-----------------------------
void GetPositions(int bus); //Global broadcasts
void SetTorques(int bus);
int AllIndexed();


double TurnaroundTime(int act_idx); // Time between start of getpositions and end of set torques
int IdleActuators();
int EnergizeActuators();

//Safety module functions----------------------
// int GetPSstate(int bus);
//ApproveMBusPower()
//VetoMBusPower()
//SetSafeParameters

//Options------------------------------
void SetEngrUnits(int onoff); //use encoder counts & puck I units or radians and Nm's
void SetTRC(int onoff); //Turn torque ripple compensation on or off
int ThereIsTRC(int act_idx); //Checks to see if we have TRC data for this actuator.



//Physical Bus functions
/*
open_btcomm(btcomm &bus,
close_btcomm(btcomm &bus,

testcomm_btcomm(btcomm &bus //checks to see if the designated entity is on the bus

getproperty_btcomm(btcomm &bus,int who, int property, long *reply);
int setproperty_btcomm(btcomm &bus, int who, int property, int verify, long value);

bulkgetproperty_btcomm(btcomm &bus,int who, int property, long *reply); //used to be getpositions
int bulksetproperty_btcomm(btcomm &bus, int who, int property, int verify, long value); // used to be setTorques

int wakePuck(int bus, int who); //move this to btsystem!
//btcomm bus info
get_max_samplerate(btcomm &bus
get_latency(btcomm &bus
//btcomm terminal functions - gives user a terminal, text commands are converted to binary

*/
#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif/* _BTSYSTEM_H */

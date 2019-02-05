/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btsystem.c
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

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <malloc.h>
#include <stdio.h>
#include <math.h>
#include <syslog.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btos.h"
#include "btsystem.h"
#include "btcan.h"
#include "btparser.h"
#define Border(Value,Min,Max)  (Value<Min)?Min:((Value>Max)?Max:Value)
#define TWOPI 6.283185
//extern pthread_mutex_t commMutex;

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
actuator_struct *act; //**< Array of actuator data (dynamically allocated at program start)
struct timeval timev; /* for actuator timing */
int num_actuators;    //**< Number of actuators
int act_data_valid = 0; //**< Used to make sure programmer did things right
int extra_pucks=0; //**< If there are more pucks on the bus than we expect we may have problems getting broadcast positions.

bus_struct *buses = NULL; //**< Array of bus data
int num_buses = 0; //**< Number of buses.
int bus_data_valid = 0; //**< Used to make sure programmer did things right
extern int gimbalsInit;
int invalidPos[MAX_NODES], insanePos[MAX_NODES], firstPos[MAX_BUSES];
uint64_t CPU_cycles_per_second;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
int Load_Bus_File(bus_struct *bus, char * filename);
actuator_struct * Load_Actuator_File(int *n_actuators, char * actuator_filename, char * motor_filename, char * puck_filename);

/*==============================*
 * Functions                    *
 *==============================*/
/** Automatically discover the robotic systems connected to this computer */
int DiscoverSystem(void)
{}

/** Use a config file to determine the robotic systems connected to this computer */
int ReadSystemFromConfig(char *fn, int *busCount)
{
   int err;
   //int busCount;
   char key[256];
   char tmpstr[256];
   int i;

   // Parse config file
   err = parseFile(fn);
   if (err) {
      syslog(LOG_ERR, "ReadSystemFromConfig- Unable to parse file: %s", fn);
      return(-1);
   }
   err = parseGetVal(INT, "system.busCount", &num_buses);
   if(err) {
      syslog(LOG_ERR, "ReadSystemFromConfig- Could not find 'system.busCount' in %s", fn);
      return(-1);
   }
   if(num_buses > MAX_BUSES) {
      syslog(LOG_ERR, "ReadSystemFromConfig- Error: busCount (%d) > MAX_BUSES (%d)", num_buses, MAX_BUSES);
      return(-1);
   }
   buses = (bus_struct*)malloc(num_buses * sizeof(bus_struct));
   if(buses == NULL) {
      syslog(LOG_ERR, "ReadSystemFromConfig- Could not malloc() space for %d buses", num_buses);
      return(-1);
   }
   for(i = 0; i < num_buses; i++) {
      sprintf(key, "system.bus[%d].type", i);
      parseGetVal(STRING, key, tmpstr);
      sprintf(key, "system.bus[%d].device", i);
      parseGetVal(STRING, key, buses[i].device_str);
      sprintf(key, "system.bus[%d].name", i);
      parseGetVal(STRING, key, buses[i].device_name);
      sprintf(key, "system.bus[%d].address", i);
      if(!strcmp(tmpstr,"CAN")) {
         buses[i].type = CAN;
         parseGetVal(LONG, key, &buses[i].address);
      } else if(!strcmp(tmpstr,"Ethernet")) {
         buses[i].type = ETHERNET;
         parseGetVal(STRING, key, tmpstr);
         // Convert "192.168.0.10" into a long int
         // buses[i].address = addrtol(tmpstr)
      } else {
         syslog(LOG_ERR, "ReadSystemFromConfig- Unknown bus type (%s) for bus %d in file %s", tmpstr, i, fn);
         return(-1);
      }
   }
   *busCount = num_buses;
   //num_buses = busCount;
   return(0);
}

/** Load information from configuration files, initialize structures, and open CAN device drivers
 
 
\return - 0 Success
      - -1 
      - -2 Error parsing the config file
      
*/
int InitializeSystem(void)
{
   int err, id;
   int bus_number;
   char key[256], valStr[256];
   //int num_buses;
   int canAddr;
   char ethAddr[32];
   long status[MAX_NODES];
   long reply;
   int groupID;
   int orderIdx;

   int cnt, cnt2, cnt3, cnt4;
   int i,j,k,l;
   int min, max, min_idx, max_idx, act_cnt, grp_cnt;
   int group_cnt[65];
   //actuator_struct *act;

   /* Goals:
       Initialize and enumerate each bus (bus_struct buses should already be filled in). 
       Fill in actuator/motor/puck data structures 
   */

   // Parse config file
   //err = parseFile(fn);
   //if (err)
   //  return -2;

   // For each bus, initialize
   //err = parseGetVal(INT, "system.busCount", (void*)&num_buses);
   //syslog(LOG_ERR, "Bus count = %d", num_buses);
   cnt = 0;
   for(bus_number = 0; bus_number < num_buses; bus_number++) {
      // Initialize the bus[] structure
      buses[bus_number].num_pucks = 0;
      buses[bus_number].num_groups = 0;
      for (groupID = 0; groupID < 65; groupID++) {
         buses[bus_number].group[groupID].group_number = -1;
         for (orderIdx = 0; orderIdx < 4; orderIdx++) {
            buses[bus_number].group[groupID].pucks_by_order[orderIdx] = -1;
         }
      }
      // Get bus device string
      //sprintf(key, "system.bus[%d].device", bus_number);
      //err = parseGetVal(STRING, key, (void*)(buses[bus_number].device_str));
      // Get bus type
      //sprintf(key, "system.bus[%d].type", bus_number);
      //err = parseGetVal(STRING, key, (void*)valStr);
      // Get bus address
      //sprintf(key, "system.bus[%d].address", bus_number);
      // If this is a CANbus
      if(buses[bus_number].type == CAN) {
         //err = parseGetVal(INT, key, (void*)&canAddr);
         // Initialize the hardware channel

         if(err = initCAN(bus_number, buses[bus_number].address))
            syslog(LOG_ERR, "Could not initialize can bus %d, err = %d", bus_number, err);

         syslog(LOG_ERR, "Waking all pucks");
         wakePuck(bus_number, GROUPID(WHOLE_ARM));

         // Enumerate nodes on the bus
         err = getBusStatus(bus_number, status);
         for(id = 0; id < MAX_NODES; cnt += status[id++] >= 0)
            ; // Count responding nodes


         // If this is an Ethernet bus
      } else if(buses[bus_number].type == ETHERNET) {
         //err = parseGetVal(STRING, key, (void*)ethAddr);
         syslog(LOG_ERR, "Robot on bus %d not initialized, Ethernet not supported", bus_number);
      }
      // Else, bus type not valid
      else {
         syslog(LOG_ERR, "InitializeSystem: Invalid bus type (%d)", buses[bus_number].type);
      }
   }

   /* Allocate the actuators data structure */
   syslog(LOG_ERR, "About to allocate space for %d nodes", cnt);
   act = malloc(sizeof(actuator_struct) * (cnt + 3));
   if (act == NULL) {
      syslog(LOG_ERR, "Actuator Memory allocation failed. Tried to allocate %d actuators.", cnt);
      return -1;
   }

   num_actuators = 0;
   for(bus_number = 0; bus_number < num_buses; bus_number++) {
      err = getBusStatus(bus_number, status);
      // Query each node for ID, CPR, IPNM, PIDX, GRPx
      firstPos[bus_number] = TRUE;
      for(id = 0; id < MAX_NODES; id++) {
         invalidPos[id] = 0;
         insanePos[id] = 0;
         if(id == SAFETY_MODULE) {
            if(status[id] != STATUS_READY) {
               syslog(LOG_ERR, "The safety module on bus %d is not functioning properly", bus_number);
            }
         } else {
            switch(status[id]) {
            case STATUS_RESET: // Get the puck out of reset
               syslog(LOG_ERR, "Waking puck %d", id);
               wakePuck(bus_number, id);

            case STATUS_READY:
               // Make sure the puck is in IDLE mode
               setProperty(bus_number, id, MODE, FALSE, MODE_IDLE);

               // Assign the bus
               act[num_actuators].bus = bus_number;

               // Query for various info
               act[num_actuators].puck.ID = id;
               switch(id) {
               case -1: //case 1: case 4: // xxx Remove me
                  act[num_actuators].motor.counts_per_rev = 40960;
                  act[num_actuators].motor.puckI_per_Nm = 2755;
                  act[num_actuators].puck.order = id-1;
                  act[num_actuators].puck.group = 1;
                  break;
               default:
                  getProperty(bus_number, id, CTS, &reply);
                  act[num_actuators].motor.counts_per_rev = reply;
                  getProperty(bus_number, id, IPNM, &reply);
                  act[num_actuators].motor.puckI_per_Nm = reply;
                  getProperty(bus_number, id, PIDX, &reply);
                  act[num_actuators].puck.order = reply-1;
                  getProperty(bus_number, id, GRPB, &reply);
                  act[num_actuators].puck.group = reply;
                  break;
               }

               syslog(LOG_ERR,"Puck: ID=%d CTS=%d IPNM=%.2lf PIDX=%d GRPB=%d",
                      act[num_actuators].puck.ID,
                      act[num_actuators].motor.counts_per_rev,
                      act[num_actuators].motor.puckI_per_Nm,
                      act[num_actuators].puck.order,
                      act[num_actuators].puck.group);
               // Set MaxTorque to 3.3A
               // 2.4A = 1/4 breaking strength = 3441
               switch(id) {
               case 1:
               case 2:
               case 3:
                  setProperty(bus_number, id, MT, FALSE, 4860); // Cable limit = 4860
                  break;
               case 4:
                  setProperty(bus_number, id, MT, FALSE, 4320); // Cable limit = 4320
                  break;
               case 5:
               case 6:
                  setProperty(bus_number, id, MT, FALSE, 3900); // Cable limit = 3900
                  break;
               case 7:
                  setProperty(bus_number, id, MT, FALSE, 1370); // Max continuous stall
                  break;
               default:
                  setProperty(bus_number, id, MT, FALSE, 1370);
                  break;
               }
               ++num_actuators; // Update the number of actuators
               break;
            default:
               break;
            }
         }
      }
   }


   // Get the cycles per second on this machine
   CPU_cycles_per_second = sysconf(_SC_CLK_TCK);

   for (cnt = 0; cnt < num_actuators; cnt++) {
      act[cnt].puck.position = 0;
      act[cnt].puck.torque_cmd = 0;
      act[cnt].angle = 0;
      act[cnt].torque = 0;
      act[cnt].puck.zero = 0;
      act[cnt].puck.index = 0;
      act[cnt].UseTRC = 0;
      act[cnt].UseEngrUnits = 1;
   }

   //set up group indexes. CAVEATS: does not check for duplicate order numbers in groups

   for (i = 0; i < num_buses; i++) {
      buses[i].total_pucks=0; /** \bug This variable seems unused */
      //initialize the group counter
      for (l = 0;l < 65;l++)
         group_cnt[l] = 0;
      //get number of actuators on this bus
      act_cnt = 0;
      for (j = 0; j < num_actuators; j++) {
         if (act[j].bus == i) {
            act_cnt++;
            group_cnt[act[j].puck.group]++;
         }
      }
      buses[i].num_pucks = act_cnt;
      //figure out how many groups were found
      for (l = 0;l < 65;l++) {
         if (group_cnt[l]) {
            buses[i].num_groups++;
            buses[i].group[buses[i].num_groups - 1].group_number = l;
         }
      }
      min = 0;
      //for each actuator on this bus scan through and sort from lowest puck_id to highest
      for (j = 0;j < act_cnt; j++) {
         max = 1000;
         for (k = 0; k < num_actuators; k++) {
            if ((act[k].bus == i) && (act[k].puck.ID < max) && (act[k].puck.ID > min)) {
               max = act[k].puck.ID;
               max_idx = k;
            }
         }
         buses[i].pucks_by_id[j] = max_idx;
         min = max;
      }

      //zero out pucks_by_order in index
      for (k = 0;k < buses[i].num_groups;k++) {
         for (l = 0; l < 4;l++)
            buses[i].group[k].pucks_by_order[l] = -1;
      }
      //create index of pucks by order
      for (j = 0;j < buses[i].num_pucks;j++) {
         for (k = 0;k < buses[i].num_groups;k++) {
            if (act[buses[i].pucks_by_id[j]].puck.group == buses[i].group[k].group_number) {
               buses[i].group[k].pucks_by_order[act[buses[i].pucks_by_id[j]].puck.order % 4] = buses[i].pucks_by_id[j];
            }
         }
      }
   }

   // Success!
   act_data_valid = 1;
   DumpData2Syslog();

   return 0;
}


/** Figure out who is online and error gracefully if it's something other that what our config files tell us.
 
\todo EnumerateSystem -> btsystemSanityCheck() run if not everything is perfect
*/
int EnumerateSystem()
{
   int busidx;
   long status[MAX_NODES*2];
   int err;
   long data;
   int cnt;
   int status_cnt = 0, expected_cnt = 0, reset_cnt = 0, running_cnt = 0, zerod_cnt = 0;

   if(!act_data_valid) {
      syslog(LOG_ERR,"EnumerateSystem:Tried to do stuff with actuators before you initialized them with InitializeActuators()");
      return -4;
   }
   //for each bus
   for (busidx = 0; busidx < num_buses; busidx++) {
      //Get status of pucks on bus
      usleep(10000);
      getBusStatus(busidx, status);

      //if god offline, error: Safety module not operating. Check power, etc.
      /*th03 (this bypasses the check to see if the safety module is online and ok)
      if(status[SAFETY_MODULE] == STATUS_OFFLINE)
   {
        syslog(LOG_ERR,"Initialize_WAM_Infrastructure: Safety module not responding.");
        return -1;
   }
      getProperty(buses[busidx].CANdev,SAFETY_MODULE,BUS_STATE,&data);
      if (data == BUS_ON)
   {
        //God was not in fault mode as expected. something is wrong.
        //reset god
        setProperty(buses[busidx].CANdev,SAFETY_MODULE,BUS_STATE,BUS_OFF);
        syslog(LOG_ERR,"Initialize_WAM_Infrastructure: Safety module was running when I started.");
        return (-2);
   }
      */
      //count number of live actuators
      status_cnt = 0;
      for (cnt = 0;cnt < MAX_NODES;cnt++) {
         if (status[cnt] != -1)
            syslog(LOG_ERR, "Status %d = %d", cnt, status[cnt]);
         if ((status[cnt] != STATUS_OFFLINE) && (cnt != SAFETY_MODULE))
            status_cnt++;
      }

      //for each actuator on this bus
      //count number of actuators expected
      //count number of pucks in reset condition
      //count number of pucks in running condition
      expected_cnt = 0;
      reset_cnt = 0;
      running_cnt = 0;
      zerod_cnt = 0;
      for (cnt = 0; cnt < num_actuators; cnt++) {
         //th syslog(LOG_ERR,"Actuator %d, Bus %d",cnt,act[cnt].bus);
         if (act[cnt].bus == busidx) {
            //syslog(LOG_ERR,"Status2 %d = %d",cnt,status[act[cnt].puck.ID]);
            expected_cnt++;
            if (status[act[cnt].puck.ID] == STATUS_RESET)
               reset_cnt++;
            else if (status[act[cnt].puck.ID] != STATUS_OFFLINE) {
               running_cnt++;
            }
         }
      }

      syslog(LOG_ERR, "Puck Status: Expected %d Found %d : %d running %d reset ", expected_cnt, status_cnt, running_cnt, reset_cnt);
      //if not number of actuators expected, error
      if (status_cnt == 0) {
         syslog(LOG_ERR, "!!!!!!!!!!!!!!!!!!!TROUBLE!!!!!!!!!!!!!!!!!!!!!!!!!!");
         syslog(LOG_ERR, "The wam appears to be turned off. Please turn it on.");
         syslog(LOG_ERR, "If the wam is on, check CAN connecors for continuity.");
         return -4;
      }

      /* Since 1 puck controls 3 passive gimbals joints, reduce expected_cnt by 2 */
      if(gimbalsInit)
         expected_cnt -= 2;

      extra_pucks = status_cnt - expected_cnt;
      buses[busidx].total_pucks=status_cnt;

      //we don't care if there are extra pucks running
      if (extra_pucks < 0) {
         syslog(LOG_ERR, "!!!!!!!!!!!!!!!!!!!TROUBLE!!!!!!!!!!!!!!!!!!!!!!!!!!!");
         syslog(LOG_ERR, "Bus %d only has %d pucks. We expected %d.",  busidx, status_cnt, expected_cnt);
         syslog(LOG_ERR, "Check CAN connecors for continuity. Verify actuators.dat is correct.");
         return -4;
      }

      //if number running != expected || number reset != expected, error, dump status, mode, temp to syslog
#if 1
      if ((running_cnt != expected_cnt) && (reset_cnt != expected_cnt)) {
         syslog(LOG_ERR, "!!!!!!!!!!!!!!!!!!!TROUBLE!!!!!!!!!!!!!!!!!!!!!!!!!!!");
         syslog(LOG_ERR, "Some of the pucks must have reset unexpectedly. There were %d running and %d reset.", running_cnt, reset_cnt);
         if((running_cnt == 1) && (reset_cnt == 4)) {
            syslog(LOG_ERR, "You may be using the gimbals, so this is OKAY...");
         } else
            return -4;
      }
#endif
      //if all pucks on the bus were recently reset
      //if (reset_cnt == expected_cnt)
      //{
      for (cnt = 0; cnt < num_actuators; cnt++) {
         syslog(LOG_ERR, "Checking puck ID %d", act[cnt].puck.ID);
         if (act[cnt].bus == busidx) {
            if(status[act[cnt].puck.ID] == STATUS_RESET) {
               syslog(LOG_ERR, "Doing puck ID %d", act[cnt].puck.ID);
               wakePuck(busidx, act[cnt].puck.ID);
               //syslog(LOG_ERR,"Doing puck ID %d",act[cnt].puck.ID);
               //Wait 100ms for the pucks to come online
               usleep(100000);
               //SetProp(cnt, MOTOR_OFFSET, act[cnt].motor.commutation_offset);
               //syslog(LOG_ERR,"Doing puck ID %d",act[cnt].puck.ID);
               //SetProp(cnt, I_OFFSET_B, act[cnt].puck.IoffsetB);
               //syslog(LOG_ERR,"Doing puck ID %d",act[cnt].puck.ID);
               //SetProp(cnt, I_OFFSET_C, act[cnt].puck.IoffsetC);
               //syslog(LOG_ERR,"Doing puck ID %d",act[cnt].puck.ID);
               //SetProp(cnt, CTS_PER_REV, act[cnt].motor.counts_per_rev);
               //SetProp(cnt, MAX_TORQUE,8191);
               setProperty(busidx, act[cnt].puck.ID, PIDX, FALSE, act[cnt].puck.order+1);
               setProperty(busidx, act[cnt].puck.ID, MT, FALSE, 4731);
               //SetProp(cnt,MT,4731); // Set to 3.3A Max Torque
               //SetProp(cnt,GROUP_A,act[cnt].puck.group);
            } else {
               setProperty(busidx, act[cnt].puck.ID, MODE, FALSE, MODE_IDLE);
               //SetProp(cnt, MODE, MODE_IDLE);
            }
            syslog(LOG_ERR, "Finished");
         }
      }
      //}
   }

   return 0;
}

/** Free memory and close CAN device drivers. Must be called at the end of your program to preven memory leaks*/
void CloseSystem()
{
   int cnt;

   /* malloc()'ed memory is automatically freed upon exit */
   //for (cnt = 0; cnt < num_actuators; cnt++) {
   //   if (act[cnt].motor.values != NULL)
   //      free(act[cnt].motor.values);
   //}
   //free(act);

   for (cnt = 0; cnt < num_buses; cnt++) {
      freeCAN(cnt);
   }
}

/** Dumps partial contents of actuator data to syslog for debuging and
verification.
    */
void DumpData2Syslog()
{
   int cnt, cnt2, cnt3, idx;
   char str[50];

   syslog(LOG_ERR, "Actuator data dump:");
   for (cnt = 0; cnt < num_actuators; cnt++) {
      syslog(LOG_ERR, "[%d]:Bus-%d,ID-%d,G-%d,O-%d,M-%d,Off-%d,Enc-%d",
             cnt, act[cnt].bus, act[cnt].puck.ID, act[cnt].puck.group, act[cnt].puck.order,
             act[cnt].motor.serial, act[cnt].motor.commutation_offset, act[cnt].motor.counts_per_rev);
   }
   for (cnt = 0; cnt < num_buses; cnt++) {
      syslog(LOG_ERR, "Data for Bus %d", cnt);
      syslog(LOG_ERR, "Bus Data: There were %d Pucks sorted by ID", buses[cnt].num_pucks);
      for (cnt2 = 0; cnt2 < buses[cnt].num_pucks; cnt2++) {
         idx = buses[cnt].pucks_by_id[cnt2];
         syslog(LOG_ERR, "[%d]: Actuator %d Puck %d", cnt2, idx, act[idx].puck.ID);
      }
      syslog(LOG_ERR, "Bus Data: There were %d Groups ", buses[cnt].num_groups);
      for (cnt2 = 0; cnt2 < buses[cnt].num_groups; cnt2++) {
         idx = buses[cnt].group[cnt2].group_number;
         syslog(LOG_ERR, "Group %d: A%d P%d A%d P%d A%d P%d A%d P%d", idx,
                buses[cnt].group[cnt2].pucks_by_order[0], act[buses[cnt].group[cnt2].pucks_by_order[0]].puck.ID,
                buses[cnt].group[cnt2].pucks_by_order[1], act[buses[cnt].group[cnt2].pucks_by_order[1]].puck.ID,
                buses[cnt].group[cnt2].pucks_by_order[2], act[buses[cnt].group[cnt2].pucks_by_order[2]].puck.ID,
                buses[cnt].group[cnt2].pucks_by_order[3], act[buses[cnt].group[cnt2].pucks_by_order[3]].puck.ID);
      }
   }
}

/** Gets a pointer to the actuators data.
\param Num_actuators A pointer to the int variable you want loaded with the number of actuators.
\return Returns an acutator_struct pointer.
*/
int GetActuators(int bus, actuator_struct **a, int *Num_actuators)
{
   int i;
   //if(!act_data_valid) {
   //   syslog(LOG_ERR,"GetActuators: Tried to do stuff with actuators before you initialized them with InitializeActuators()");
   //   return NULL;
   //}
   for(i = 0; i < num_actuators; i++) {
      if(act[i].bus == bus) {
         *a = &act[i];
         *Num_actuators = buses[bus].num_pucks;
         return(0);
      }
   }

   return -1;
}

/** Gets a pointer to the buses data
\param Num_buses A pointer to the int variable you want loaded with the number of buses.
\return Returns an bus_struct pointer.
*/
bus_struct * GetBuses(int *Num_buses)
{
   if(!buses) {
      syslog(LOG_ERR,"GetBuses: buses data structure is NULL");
      return NULL;
   }
   *Num_buses = num_buses;

   return buses;
}

/** Broadcast getpositions for all actuators.
 
This function will send act->torque for each actuator in the database. 
If UseEngrUnits is true, the torque value will
be converted to puck units. If UseTRC is true, the torque ripple 
cancelation value will be added if it is available.
 
*/
void GetPositions(int bus)
{
   int cnt, cnt2, idx, err;
   double tmp;
   long int data[MAX_NODES];
   long int newPosition;

   if(!act_data_valid) {
      syslog(LOG_ERR,"GetProp: Tried to do stuff with actuators before you initialized them with InitializeActuators()");
      return;
   }

   /*
   for (cnt = 0; cnt < num_actuators; cnt++) {
      if(act[cnt].bus == bus){
         gettimeofday(&timev, 0); // if this segfaults put a timezone struct in here 
         act[cnt].lastpos = timev.tv_usec + timev.tv_sec * 1000000;
      }
}
   */
   /* Initialize the returned position array with illegal values */
   for(cnt = 0; cnt < MAX_NODES; cnt++)
      data[cnt] = 0x80000000;

   //for each bus do a broadcast get position...
   //for (cnt = 0; cnt < num_buses; cnt++) {
   err = getPositions(bus, 0, buses[bus].num_pucks, data);
   if(err) {
      // Problem reading positions
      //syslog(LOG_ERR, "getPositions err = %d", err);
   }
   //for each puck on this bus, get your position from the return array
   for (cnt2 = 0; cnt2 < buses[bus].num_pucks; cnt2++) {
      idx = buses[bus].pucks_by_id[cnt2];

      if ((idx > num_actuators)|| (idx<0)) {
         syslog(LOG_ERR, "puck %d on bus %d shows an idx %d", cnt2,bus, idx);
         idx = 0;
      }

      newPosition = data[act[idx].puck.ID];

      if(firstPos[bus]) {
         act[idx].puck.position = newPosition;
      }

      /* Check for a valid value */
      if(newPosition != 0x80000000) {
         /* Check for a sane value, 1/4 rev / cycle is the limit */
         if(labs(newPosition - act[idx].puck.position) < (act[idx].motor.counts_per_rev / 4)) {
            act[idx].puck.position = newPosition;
         } else {
            insanePos[act[idx].puck.ID]++;
            //err = canClearMsg(bus);
         }
      } else {
         invalidPos[act[idx].puck.ID]++;
         //err = canClearMsg(bus);
      }

      if (act[idx].UseEngrUnits)
         act[idx].angle = TWOPI * act[idx].puck.position / act[idx].motor.counts_per_rev;
      else
         act[idx].angle = act[idx].puck.position;

      if (act[idx].UseTRC)
         act[idx].puck.enc_pos = act[idx].puck.position + act[idx].puck.poffset;
   }

   if(firstPos[bus])
      firstPos[bus] = FALSE;
   //}
}

/** Broadcast the torques to the actuators
*/
void SetTorques(int bus)
{
   int cnt, cnt2, cnt3, idx;
   int data[MAX_NODES];
   long enc, pos, poff, TRC;
   //  uint64_t now;
   double tmp;

   if(!act_data_valid) {
      syslog(LOG_ERR,"SetTorques: Tried to do stuff with actuators before you initialized them with InitializeActuators()");
      return;
   }
   //Do engineering units conversion
   for (cnt = 0; cnt < num_actuators; cnt++) {
      if(act[cnt].bus == bus) {
         if (act[cnt].UseEngrUnits) {
            tmp = act[cnt].torque * act[cnt].motor.puckI_per_Nm;
            act[cnt].puck.torque_cmd = Border(tmp, -8191, 8191);
         } else
            act[cnt].puck.torque_cmd = Border(act[cnt].torque, -8191, 8191);

         //add torque ripple cancellation if applicable
         if ((act[cnt].UseTRC) && (act[cnt].motor.values != NULL)) {
            TRC = 0;
            tmp = floor(act[cnt].puck.enc_pos * act[cnt].motor.ratio); //puck.enc_pos calculated in GetPositions()
            enc = tmp;
            enc = enc % act[cnt].motor.num_values;
            if(enc < 0)
               enc += act[cnt].motor.num_values;
            TRC = act[cnt].motor.values[enc];
            act[cnt].puck.torque_cmd += TRC;
         }
      }
   }

   //Pack data and send it
   //for (cnt = 0; cnt < num_buses; cnt++) {
   cnt = bus;
   for (cnt2 = 0; cnt2 < buses[cnt].num_groups; cnt2++) {
      for (cnt3 = 0; cnt3 < 4; cnt3++) {
         idx = buses[cnt].group[cnt2].pucks_by_order[cnt3];
         if (idx == -1) {
            data[cnt3] = 0;
         } else if ((idx > num_actuators) || (idx<0)) {
            syslog(LOG_ERR, "SetTorque:bus %d, order idx %d pointed to act %d", cnt,cnt3,idx);
            idx = 0;
            data[cnt3] = 0;
         } else {
            data[cnt3] = act[idx].puck.torque_cmd;
         }
      }
      setTorques(cnt, buses[cnt].group[cnt2].group_number, data);
   }
   //}

   //I think this code was used for debugging in RTAI, eliminating it to debug in XENOMAI
   /*
   for (cnt = 0; cnt < num_actuators; cnt++) {
      if(act[cnt].bus == bus){
         gettimeofday(&timev, 0); // if this segfaults put a timezone struct here
         act[cnt].lasttrq = timev.tv_usec + timev.tv_sec * 1000000;
      }
}
   */
}

/** Get a property from an actuator.
 
  caveats: If called with an invalid act_idx it uses idx=0
 
  \param act_idx Index of the actuator we want to get info from [0..num_actuators]
  \param property Property we want to request. Enumerated in btcan.h.
  
  \retval value
  
  \warning No way out if the bus is dead and doesn't return a value
*/
long GetProp(int act_idx, int property)
{
   int idx;
   long int data[MAX_NODES];

   if(!act_data_valid) {
      syslog(LOG_ERR,"GetProp: Tried to do stuff with actuators before you initialized them with InitializeActuators()");
      return 0;
   }

   if ((act_idx >= num_actuators) || (act_idx < 0)) {
      syslog(LOG_ERR,"GetProp: index out of range. idx %d, max %d.",act_idx,num_actuators);
      idx = 0;
   } else
      idx = act_idx;
   //syslog(LOG_ERR, "GetProp ID %d data before %d", actuator_id, data[0]);
   getProperty(act[idx].bus, act[idx].puck.ID, property, data);
   //syslog(LOG_ERR, "GetProp ID %d data after %d", actuator_id, data[0]);

   if (property == AP) {
      act[idx].puck.position = *data;
      if (act[idx].UseEngrUnits)
         act[idx].angle = TWOPI * act[idx].puck.position / act[idx].motor.counts_per_rev;
      else
         act[idx].angle = act[idx].puck.position;
   }

   return *data;
}

/** Set the property of an actuator.
 
 \param actuator_id The id number of the actuator you are setting a property on. The id starts with 0 and counts up in the order that the actuators are listed in actuators.dat.
 \param property The property that you want to set. See the enumerations listed in btwam.h.
 \param data The data you want to send
 
 */
int SetProp(int actuator_id, int property, long data)
{
   int err, idx;

   idx = actuator_id;
   if(!act_data_valid) {
      syslog(LOG_ERR,"SetProp: Tried to do stuff with actuators before you initialized them with InitializeActuators()");
      return -1;
   }
   if((idx > num_actuators) || (idx < 0)) {
      syslog(LOG_ERR, "SetProp:actuator idx %d out of range",idx);
      return -1;
   }
   err = setProperty(act[idx].bus, act[idx].puck.ID, property, TRUE, data);

   if(property == AP) {
      act[idx].puck.poffset = GetProp(idx,MECH);
   }
   return err;
}

/** Set the property of a device.
 
 \param CANid The id number of the device you are setting a property on. 
 \param property The property that you want to set. See the enumerations listed in btwam.h.
 \param data The data you want to send
 
*/
int SetByID(int bus, int CANid, int property, long data)
{
   int err;
   //syslog(LOG_ERR, "About to setProperty, property = %d", property);
   err = setProperty(bus, CANid, property, TRUE, data);

   return(err);
}

/** The number of milliseconds it took for the last getposition -> settorques exchange
*/
double TurnaroundTime(int act_idx)
{
   uint64_t diff;
   static double tmp;

   diff = act[act_idx].lasttrq - act[act_idx].lastpos;
   if (diff < 0 || diff > 100000000)
      return tmp;

   tmp = (double)diff * 1000.0 / (double)CPU_cycles_per_second;
   return tmp;
}

/** \depreciated Check to see how many actuators don't have indexes yet.
*/
int AllIndexed() //returns number of pucks that have not gotten indexes yet.
{
   int ret, cnt;

   ret = 0;
   for (cnt = 0; cnt < num_actuators; cnt++)
   {
      if(GetProp(cnt, MECH) >= 0)
         ret++;
   }
   return num_actuators - ret;
}

/*
int GetPSstate(int bus)
{
  long data;
  getProperty(&(buses[bus].CANdev), SAFETY_MODULE, BUS_STATE, &data);
}
*/

/** Sets all actuators to mode 2 (MODE_TORQUE)
*/
int EnergizeActuators()
{
   int cnt, err = 0;

   for (cnt = 0; cnt < num_actuators; cnt++) {
      err += SetProp(cnt, MODE, MODE_TORQUE);
      if (act[cnt].puck.torque_cmd != 0)
         syslog(LOG_ERR,"!!!DANGER!!! The pucks were energized with a non-zero torque.");
   }
   return err;
}

/** Sets all actuators to mode 0 (MODE_IDLE)
*/
int IdleActuators()
{
   int cnt, err = 0;

   for (cnt = 0; cnt < num_actuators; cnt++) {
      err += SetProp(cnt, MODE, MODE_IDLE);
   }
   return err;
}

/** Turns on or off conversion to and from engineering units.
 
  defaults to on.
  
  When on, actuator angle is calculated as radians based on the number of encoder counts per revolution
  When off, actuator angle equals the motor position in encoder ticks.
  
  When on, actuator torque is converted from Newton meters based on the IperNm value in the motor file.
  When off, actuator torque is in puck units.
*/
void SetEngrUnits(int onoff)
{
   int cnt;

   for (cnt = 0; cnt < num_actuators; cnt++) {
      act[cnt].UseEngrUnits = onoff;
   }
}

/** Turns on or off Torque ripple cancelation
 
  If there is no torque ripple cancelation data available this function will do nothing.
*/
void SetTRC(int onoff)
{
   int cnt,isindexed = 0;

   for (cnt = 0; cnt < num_actuators; cnt++) {
      if (act[cnt].motor.values == NULL || onoff == 0)
         act[cnt].UseTRC = 0;
      else {
         isindexed = GetProp(cnt,ZERO);
         if (isindexed) {
            act[cnt].puck.poffset = GetProp(cnt,MECH);
            act[cnt].UseTRC = 1;
         } else
            act[cnt].UseTRC = 0;
      }
   }
}

/** Checks to see if torque ripple cancellation data is available for this actuator.
 
  \retval 0 Nope
  \retval 1 Yup
 
*/
int ThereIsTRC(int act_idx)
{
   int idx;

   idx = act_idx;
   if ((idx > num_actuators)|| (idx<0)) {
      syslog(LOG_ERR, "ThereIsTRC:actuator idx %d out of range",idx);
      return 0;
   }
   return ((act[idx].motor.values == NULL)?0:1);

}

/*============================================== private functions ========================================*/
/** (internal) Loads data from files into actuator data array
 
  \retval Pointer to actuator array that was allocated
  \retval NULL Unable to allocate actuator array or some other problem occured
 
*/
actuator_struct * Load_Actuator_File(int *n_act, char * actuator_filename, char * motor_filename, char * puck_filename)
{
   FILE *in;
   char fileName[32];
   FILE *tr;
   char magic_string[50];
   int n_actuators, num_trc;
   actuator_struct *act;
   int cntsum, asum;
   int cnt, cnt2, anum, id, ser, bus, offset, enc_counts, group, order, ioffsetb, ioffsetc;
   int idkey[20];
   int done;
   double ipernm;

   //--------------------load actuator data----------
   if ((in = fopen(actuator_filename, "r")) == NULL) {
      syslog(LOG_ERR, "Could not open actuator data file: %s", actuator_filename);
      return NULL;
   }

   fscanf(in, "%s", magic_string);

   if (strcmp(magic_string, "BarrettTechnologyActuatorFile")) {
      syslog(LOG_ERR, "Actuator data file has wrong header string: [%s] instead of [BarrettTechnologyActuatorFile]", magic_string);
      return NULL;
   }

   fscanf(in, "%d", &n_actuators);

   if ((n_actuators < 1) || (n_actuators > 100)) {
      syslog(LOG_ERR, "Actuator data file reports %d actuators. Must be between 1 and 100.", n_actuators);
      return NULL;
   }

   if ((act = malloc(sizeof(actuator_struct) * (n_actuators + 1))) == NULL) {
      syslog(LOG_ERR, "Actuator Memory allocation failed. Tried to get %d actuators.", n_actuators);
      return NULL;
   }

   for (cnt = 0;cnt < n_actuators;cnt++) {
      if (fscanf(in, "%d, %d, %d", &bus, &id, &ser) == EOF) {
         syslog(LOG_ERR, "Load actuators: There are less actuators than you say there are in this file.");
         syslog(LOG_ERR, "Load actuators: Make sure the number of actuators specified is less than or equal to the number of entries.");
         free(act);
         return NULL;
      }


      act[cnt].puck.serial = id;
      act[cnt].motor.serial = ser;
      act[cnt].bus = bus;

      //check to make sure there isn't a duplicate of this somewhere.
      for (cnt2 = 0;cnt2 < cnt;cnt2++)
         if ((act[cnt].puck.ID == act[cnt2].puck.ID) && (act[cnt].motor.serial == act[cnt2].motor.serial)) {
            syslog(LOG_ERR, "act[%d] and [%d] appear to be duplicates. Ignoring it", cnt, cnt2);
            cnt--;
         }
   }

   fclose (in); //close the actuator file

   //---------------load motor data----------
   if ((in = fopen(motor_filename, "r")) == NULL) {
      syslog(LOG_ERR, "Could not open motor data file: %s", motor_filename);
      free(act);
      return NULL;
   }

   for (cnt = 0;cnt < n_actuators;cnt++) {
      fseek( in, 0, SEEK_SET );
      done = 0;
      while (!done) {
         if (fscanf(in, "%d, %d, %d, %lf", &ser, &offset, &enc_counts, &ipernm) == EOF) {
            syslog(LOG_ERR, "Could not find a matching motor serial number in the motor data file.");
            free(act);
            return NULL;
         } else if (ser == act[cnt].motor.serial) {
            done = 1;
            act[cnt].motor.commutation_offset = offset;
            act[cnt].motor.counts_per_rev = enc_counts;
            act[cnt].motor.puckI_per_Nm = ipernm;
         }
      }
      /*! \todo need to add initialization of other data in the actuator structure here*/
   }

   fclose(in);  //close the motor file

   //------------Load puck data-------------------
   if ((in = fopen(puck_filename, "r")) == NULL) {
      syslog(LOG_ERR, "Could not open puck data file: %s", puck_filename);
      free(act);
      return NULL;
   }

   for (cnt = 0;cnt < n_actuators;cnt++) {
      fseek( in, 0, SEEK_SET );
      done = 0;
      while (!done) {
         if (fscanf(in, "%d, %d, %d, %d, %d, %d", &ser, &id, &group, &order, &ioffsetb, &ioffsetc) == EOF) {
            syslog(LOG_ERR, "Could not find a matching puck ID & bus in the puck data file.");
            free(act);
            return NULL;
         } else if (ser == act[cnt].puck.serial) {
            done = 1;
            act[cnt].puck.ID = id;
            act[cnt].puck.group = group;
            act[cnt].puck.order = order-1;
            act[cnt].puck.IoffsetB = ioffsetb;
            act[cnt].puck.IoffsetC = ioffsetc;
         }
      }
   }

   fclose(in);  //close the motor file


   //-----------------Load torque ripple data-------------------
   for (cnt = 0; cnt < n_actuators; cnt++) {
      /* Open the *.tr file */
      sprintf(fileName, "motor%d.tr", act[cnt].motor.serial);
      if ((tr = fopen(fileName, "r")) != NULL) {

         fscanf(tr, "%d", &num_trc);
         syslog(LOG_ERR,"Trying to get %d values from file: %s", num_trc, fileName);

         /* Read the tr values */
         if ((act[cnt].motor.values = (int* )malloc(num_trc * sizeof(int))) == NULL) {
            syslog(LOG_ERR, "Actuator Memory allocation failed. Tried to get %d torque ripple values.", num_trc);
         } else {
            act[cnt].motor.num_values = num_trc;
            act[cnt].motor.ratio = (double)num_trc / (double)act[cnt].motor.counts_per_rev;

            for (cnt2 = 0; cnt2 < num_trc; cnt2++) {
               if(fscanf(tr, "%d", &(act[cnt].motor.values[cnt2])) != 1) {
                  syslog(LOG_ERR, "Error reading TR data: File = %s, Iteration %d of %d", fileName, cnt2, num_trc);
               }
            }
         }
         fclose(tr);
      } else //Make sure we are not accidentally accessing "bad" memory
         act[cnt].motor.values = NULL;
   }

   *n_act = n_actuators;
   syslog(LOG_ERR,"LoadActuators complete");
   return act;
}

/** (internal) Loads information in busses.dat
        
    \retval int number of busses found
    \retval -1 fatal error
    
    \todo fscanf(bla bla , magic_string, bla bla) needs to be checked
*/
int Load_Bus_File(bus_struct *bus, char * filename)
{
   FILE *in;
   int cnt, cnt2, cnt3, anum, id, ser, offset, enc_counts;
   int idkey[20];
   char magic_string[50];
   int cntsum, asum;
   int bus_number;
   char dev_str[100];

   if ((in = fopen(filename, "r")) == NULL) {
      syslog(LOG_ERR, "Could not open bus data file: %s", filename);
      return -1;
   }

   fscanf(in, "%s", magic_string);

   if (strcmp(magic_string, "BarrettTechnologyBusFile")) {
      syslog(LOG_ERR, "Bus data file has wrong header string: [%s] instead of [BarrettTechnologyBusFile]", magic_string);
      fclose(in);
      return -1;
   }

   cnt = 0;
   while ((fscanf(in, "%d, %s%s", &bus_number, dev_str, magic_string) != EOF)) {
      if (bus_number != cnt) {
         syslog(LOG_ERR, "Illegal Bus file: bus numbers must start with 0 and count up in sequence");
         fclose(in);
         return -1;
      }
      strcpy(bus[bus_number].device_str , dev_str);
      //th debug: syslog(LOG_ERR,"Read %s :: %s :: Got %s",dev_str,magic_string,bus[bus_number].device_str);
      cnt++;
      if (cnt >= MAX_BUSES) {
         syslog(LOG_ERR, "Illegal Bus file: Number of buses exceeds %d.", MAX_BUSES);
         fclose(in);
         return -1;
      }

      //zero out actuator index structures.
      /** \todo need to allocate this dynamically one day.*/
      bus[bus_number].num_pucks = 0;
      bus[bus_number].num_groups = 0;
      for (cnt2 = 0;cnt2 < 65;cnt2++) {
         bus[bus_number].group[cnt2].group_number = -1;
         for (cnt3 = 0; cnt3 < 4; cnt3++) {
            bus[bus_number].group[cnt2].pucks_by_order[cnt2] = -1;
         }
      }
   }
   if (cnt == 0) {
      syslog(LOG_ERR, "Bus data file has no information");
      fclose(in);
      return -1;
   }
   fclose(in);

   return cnt;
}

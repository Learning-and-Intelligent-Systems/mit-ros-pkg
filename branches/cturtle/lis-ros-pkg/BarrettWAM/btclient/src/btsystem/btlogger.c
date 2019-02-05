/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btlogger.c
 *  Creation Date ...... 03 Apr 2003
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Note: Uses some code & concepts developed at Nortwestern University 
 *        by Traveler Hauptman 
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

#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>

#include "btmath.h"
#include "btlogger.h"

/*==============================*
 * Internal use Functions       *
 *==============================*/
void InitDataFileDL(btlogger *db);
int DataSizeDL(btlogger *db);
void UpdateDL(btlogger *db);
void flushDL(btlogger *db);
void DLwrite(btlogger *db);

/*==============================*
 * Functions                    *
 *==============================*/
/*****************************************************************************/
/** Allocate a specified number of fields.
 
You can add fewer fields than specified by PrepDL() but you cannot AddDataDL() 
after calling InitDL().
 
Typical usage:
\code
  PrepDL(&log,3);
  AddDataDL(&log,&a,sizeof(double),BTLOG_DOUBLE,"Arclength");
  //...more AddDataDL()
  //initialize buffers
  InitDL(&log,1000,"logfile.dat");
\endcode
 
\param db Pointer to the btlogger structure
\param fields The number of fields you want to record
 
\internal chk'd TH 051101
*/
int PrepDL(btlogger *db, unsigned int fields)
{
   int cnt;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(db,"PrepDL"))
      return -2;
#endif

   db->data = btmalloc(fields * sizeof(btdata_info));

   db->fields = 0;
   db->maxfields = fields;
   for(cnt = 0; cnt < db->maxfields;cnt++)
      db->data[cnt].size = 0;

   return 0;
}

/** Adds a field to the list of data to log.
 
You must call PrepDL() first. Fields are
added in the order that this function is called.
 
\param size Size of data = Array_length * sizeof(type)
\param data Pointer to data you want to log.
\param type See #btlog_enum
\param name Data name to be printed in the column heading
\retval 0 Success
\retval -1 No more fields available
\retval -2 db was invalid 
 
\internal chk'd TH 051101
\bug Check to see if we have already initialized with InitDL()
*/
int AddDataDL(btlogger *db,void *data, int size, int type,char *name)
{
   int idx,ret;

#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(db,"AddDataDL"))
      return -2;
#endif
   //Sanity check
#if (BTDEBUG & BTDEBUG_RANGE) > 3

   if (db->maxfields < 0) {
      syslog(LOG_ERR,"AddDataDL: db->maxfields = %d. Something in wrong...",db->maxfields);
      return -3;
   }
#endif

   idx = db->fields;
   if (idx <= db->maxfields) {
      db->data[idx].size = size;
      db->data[idx].data = data;
      db->data[idx].type = type;
      strcpy(db->data[idx].name,name);
      db->fields++;
      ret = 0;
   } else {
#if (BTDEBUG & BTDEBUG_RANGE) > 10
      syslog(LOG_ERR,"AddDataDL: No more field slots left. Use a higher value in PrepDL()");
#endif

      ret = -1;
   }
   return ret;
}

/** \internal (internal use) Counts up how much data is being stored
\exception db not valid
\internal chk'd TH 051101
*/
int DataSizeDL(btlogger *db)
{
   int total,cnt;
   
   total = 0;
   for(cnt = 0; cnt < db->maxfields;cnt++)
      total += db->data[cnt].size;
   return total;
}

/** Initialize the buffers and prepare for logging.
 
 Don't forget to call PrepDL() 
and AddDataDL() first to define what fields will be recorded.
 
\param size The number of records the buffer will hold
\param filename The path and filename where you want to write to
\retval 0 Success
\retval -1 Could not open file
\internal chk'd TH 051101
*/

int InitDL(btlogger *db,int size, char *filename)
{
   int datasize;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(db,"InitDL"))
      return -2;
#endif

   db->Log_Data = 0;
   db->DLwritten = 0;
   db->Log_File_Open = 0;
   db->DLctr = 0;
   db->DLidx = 0;
   if ((db->DLfile = fopen(filename,"w"))==NULL) {
      syslog(LOG_ERR,"InitDL:Could not open datalogger file %s",filename);
      return -1;
   }
   db->Log_File_Open = 1;
   db->buffersize = size;
   datasize = DataSizeDL(db);
   db->data_size = datasize;
   db->DLbuffer1 = btmalloc(size * datasize);
   db->DLbuffer2 = btmalloc(size * datasize);

   InitDataFileDL(db);
   db->DL = db->DLbuffer1;
   return 0;
}

/** Turn the data logger on.
The data logger will not record data until after DLon() is called
\internal chk'd TH 051101
*/
void DLon(btlogger *db)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(db,"DLon"))
      return;
#endif

   db->Log_Data = 1;
}

/** Turn the data logger off.
When the datalogger is turned off with DLoff(), data logging is paused until
logging is turned back on by DLon()
\internal chk'd TH 051101
*/
void DLoff(btlogger *db)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(db,"DLoff"))
      return;
#endif

   db->Log_Data = 0;
}

/** Close the logging file and free buffer memory
\internal chk'd TH 051101
*/
void CloseDL(btlogger *db)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(db,"CloseDL"))
      return;
#endif

   flushDL(db);
   free(db->data);
   free(db->DLbuffer1);
   free(db->DLbuffer2);
   fclose(db->DLfile);
}

/** Copy all the data pointed to by the btdata_info array into the present buffer.
 
 This function should be called every time you want to save a record to the buffer.
 Typically this is called in a high priority thread with a short loop period.
 
 \internal chk'd TH 051101
*/
void TriggerDL(btlogger *db)
{
   int tmp,cnt;
   void *start,*run;
#ifdef BT_NULL_PTR_GUARD

   if (!btptr_ok(db,"TriggerDL"))
      return;
#endif
   //If data logging is turned on
   if (db->Log_Data) {
      start = db->DL + db->DLidx * db->data_size; //point to the present location in the buffer

      for(cnt = 0;cnt < db->fields; cnt++) {
         memcpy(start,db->data[cnt].data,db->data[cnt].size); //copy user data to buffer
         start += db->data[cnt].size;
      }
      UpdateDL(db);
   }
}

/**\internal  (internal)Writes the header of a binary data file.
\internal chk'd TH 051101
*/
void InitDataFileDL(btlogger *db)
{
   FILE *datfile;
   int tmp,cnt;
   long len;

   datfile = db->DLfile;

   fwrite(&(db->fields),sizeof(int),1,datfile);   //6 fields

   for(cnt = 0;cnt < db->fields; cnt++) {
      fwrite(&(db->data[cnt].type),sizeof(int),1,datfile);
      fwrite(&(db->data[cnt].size),sizeof(int),1,datfile); //block size = arraysize + sizeof(variable)
      fwrite(db->data[cnt].name,sizeof(char),50,datfile);
   }
}

/**\internal  (internal)UpdateDL increments the DL index and switches the pointer to the other buffer if necessary
\internal chk'd TH 051101
*/
void UpdateDL(btlogger *db)
{
   db->DLidx++;                      //Increment current array index

   //If our index exceeds our array size
   if (db->DLidx >= db->buffersize) {
      db->DLidx = 0;                  //reset our index to zero

      //If we are currently pointed to buffer 1
      if (db->DL == db->DLbuffer1) {
         db->DL = db->DLbuffer2;           //Point to buffer 2
         db->DLwritten = 1;            //Indicate that buffer 1 is full
      } else {
         db->DL = db->DLbuffer1;           //Point to buffer 1
         db->DLwritten = 2;            //Indicate that buffer 2 is full
      }
   }
}

/**\internal  (internal)DLwrite checks to see if one of the buffers got filled and if so, writes it to a file
\internal chk'd TH 051101
*/
void DLwrite(btlogger *db)
{
   void  *DLout;
   int Ridx;
   long Rlength;

   //If data logging is turned on
   if (db->Log_Data) {
      //If any buffer is full
      if (db->DLwritten) {
         //If our data logging files are open and everything is peachy
         if (db->Log_File_Open) {

            if (db->DLwritten == 1)       //If buffer1 is full
               DLout = db->DLbuffer1;      //Set DLout to point the same place as DLbuffer1
            else if (db->DLwritten == 2)  //Else If buffer2 is full
               DLout = db->DLbuffer2;      //Set DLout to point the same place as DLbuffer1

            Ridx = db->DLctr;                           //Record index = full buffer counter
            fwrite(&Ridx,sizeof(int),1,db->DLfile);     //Write Record index as a binary integer
            Rlength = db->buffersize*db->data_size;         //Calculate the Record length
            fwrite(&Rlength,sizeof(long),1,db->DLfile);     //Write the Record length in bytes

            fwrite(DLout,db->data_size,db->buffersize,db->DLfile);  //Write all the data in binary form
            db->DLctr++;                                        //increment the record index
         }
         db->DLwritten=0;                                      //Reset full buffer indicator to zero
      }
   }
}

/**\internal  (internal use) Write what is remaining in our buffers before closing
\internal chk'd TH 051101
*/
void flushDL(btlogger *db)
{
   void  *DLout;
   int Ridx;
   long Rlength;


   if (db->Log_File_Open) {
      //If our data logging files are open and everything is peachy
      DLout = db->DL;
      Ridx = db->DLctr;                           //Record index = full buffer counter
      fwrite(&Ridx,sizeof(int),1,db->DLfile);     //Write Record index as a binary integer
      Rlength = db->DLidx*db->data_size;         //Calculate the Record length
      fwrite(&Rlength,sizeof(long),1,db->DLfile);     //Write the Record length in bytes

      fwrite(DLout,db->data_size,db->DLidx,db->DLfile);  //Write all the data in binary form
      db->DLctr++;                                        //increment the record index
   }
}

/** Checks the buffers and writes them if one is full.
 
This must be cyclically called (from an event loop perhaps)
with a period that is shorter than the time it takes to fill the buffer.
 
\warning If you do not call this often enough, you will have buffer sized gaps
in your data file.
 
\internal chk'd TH 051101
*/
void evalDL(btlogger *db)
{
#ifdef BT_NULL_PTR_GUARD
   if (!btptr_ok(db,"evalDL"))
      return;
#endif

   if (db->DLwritten)
      DLwrite(db);
}

/*************************** binary file to text file converter ***********/
/** Decode a binary file created by btlogger.
 
 
\param header If header !0 then the first line will be column names
 
\internal chk'd TH 051101
*/
int DecodeDL(char *infile, char *outfile, int header)
{
   btlogger db; //use this just because it is convinient
   int numfields=0,raysize;
   FILE *inf,*outf;
   int fieldcnt;
   int current_index;
   long length;
   void *data,*dataidx;
   long cnt,idx,ridx;
   int array_len;

   int *intdata;
   double *doubledata;
   long *longdata;
   long long *exlongdata;

   syslog(LOG_ERR,"DecodeDL: Starting logfile decode from %s -> %s",infile,outfile);

   //open input file
   if ((inf = fopen(infile,"rb"))==NULL) {
      syslog(LOG_ERR,"DecodeDL:Unable to open input file: %s",infile);
      return -1;
   }

   //open output file
   if ((outf = fopen(outfile,"w"))==NULL) {
      syslog(LOG_ERR,"DecodeDL:Unable to open output file: %s\n",outfile);
      return -2;
   }

   //figure out fields
   //print fields
   fread(&fieldcnt,sizeof(int),1,inf);
   syslog(LOG_ERR,"DecodeDL:Fields %d",fieldcnt);
   PrepDL(&db,fieldcnt); //allocate memory for our field info

   // Read header info, write text header
   for (cnt = 0;cnt < fieldcnt;cnt++) {
      fread(&(db.data[cnt].type),sizeof(int),1,inf);
      fread(&(db.data[cnt].size),sizeof(int),1,inf);
      fread(db.data[cnt].name,sizeof(char),50,inf);

      switch (db.data[cnt].type) {
      case 0://integer
         array_len = db.data[cnt].size / sizeof(int);
         break;
      case 1://long
         array_len = db.data[cnt].size / sizeof(long);
         break;
      case 2://double
         array_len = db.data[cnt].size / sizeof(double);
         break;
      case 3://long long
         array_len = db.data[cnt].size / sizeof(long long);
         break;
      case 4://btreal
         array_len = db.data[cnt].size / sizeof(btreal);
         break;
      }

      if (header) {
         if (array_len > 1) {
            for (ridx = 0; ridx < array_len; ridx++) {
               fprintf(outf,"%s[%d]",db.data[cnt].name,ridx);
               if ((ridx < array_len - 1) || (cnt < fieldcnt - 1))
                  fprintf(outf,",");
            }
         } else {
            fprintf(outf,"%s",db.data[cnt].name);
            if (cnt < fieldcnt - 1)
               fprintf(outf,",");
         }
      }

      //syslog(LOG_ERR,"DecodeDL:Field %d - type: %d size: %d, name: %s",cnt,db.data[cnt].type,db.data[cnt].size,db.data[cnt].name);
   }
   if (header)
      fprintf(outf,"\n");

   while(!feof(inf)) {
      if(fread(&current_index,sizeof(int),1,inf)==0)
         break;
      fread(&length,sizeof(long),1,inf);
      //syslog(LOG_ERR,"DecodeDL:record index: %d length: %ld",current_index,length);
      data = btmalloc(length);

      fread(data,sizeof(char),length,inf);

      dataidx = data;
      cnt = 0;
      while(cnt<length) {

         for (idx = 0;idx < fieldcnt;idx++) {
            switch (db.data[idx].type) {
            case 0://integer
               array_len = db.data[idx].size / sizeof(int);
               if ((db.data[idx].size % sizeof(int)) != 0)
                  syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
               for (ridx = 0; ridx < array_len; ridx++) {
                  intdata = (int *)dataidx;
                  fprintf(outf," %d ",*intdata);
                  if (ridx < array_len - 1)
                     fprintf(outf,",");
                  dataidx +=  sizeof(int);
               }
               cnt+=db.data[idx].size;
               break;
            case 1://long
               array_len = db.data[idx].size / sizeof(long);
               if ((db.data[idx].size % sizeof(long)) != 0)
                  syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
               for (ridx = 0; ridx < array_len; ridx++) {
                  longdata = (long *)dataidx;
                  fprintf(outf," %ld ",*longdata);
                  if (ridx < array_len - 1)
                     fprintf(outf,",");
                  dataidx +=  sizeof(long);
               }
               cnt+=db.data[idx].size;
               break;
            case 2://double
               array_len = db.data[idx].size / sizeof(double);
               if ((db.data[idx].size % sizeof(double)) != 0)
                  syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
               for (ridx = 0; ridx < array_len; ridx++) {
                  doubledata = (double *)dataidx;
                  fprintf(outf," %f ",*doubledata);
                  if (ridx < array_len - 1)
                     fprintf(outf,",");
                  dataidx +=  sizeof(double);
               }
               cnt+=db.data[idx].size;
               break;
            case 3://long long
               array_len = db.data[idx].size / sizeof(long long);
               if ((db.data[idx].size % sizeof(long long)) != 0)
                  syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
               for (ridx = 0; ridx < array_len; ridx++) {
                  exlongdata = (long long *)dataidx;
                  fprintf(outf," %lld ",*exlongdata);
                  if (ridx < array_len - 1)
                     fprintf(outf,",");
                  dataidx +=  sizeof(long long);
               }
               cnt+=db.data[idx].size;
               break;
            case 4://btreal
               array_len = db.data[idx].size / sizeof(btreal);
               if ((db.data[idx].size % sizeof(btreal)) != 0)
                  syslog(LOG_ERR,"DecodeDL: This block is not an even multiple of our datatype (int)");
               for (ridx = 0; ridx < array_len; ridx++) {
                  doubledata = (btreal *)dataidx;
                  fprintf(outf," %f ",*doubledata);
                  if (ridx < array_len - 1)
                     fprintf(outf,",");
                  dataidx +=  sizeof(btreal);
               }
               cnt+=db.data[idx].size;
               break;
            }
            if (idx < fieldcnt-1)
               fprintf(outf,",");

         }
         fprintf(outf,"\n");
      }

      free(data);
   }
   free(db.data);
   fclose(inf);
   fclose(outf);
}

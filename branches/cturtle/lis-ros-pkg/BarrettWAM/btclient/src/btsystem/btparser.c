/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btparser.c
 *  Creation Date ...... 15 Feb 2005
 *  Author ............. Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2005-2008 Barrett Technology, Inc. <support@barrett.com>
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
#include "btmath.h"
#include "btparser.h"

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
char  hdr[255];
FILE *outFile;
btparser btparser_default = BTPARSE_INIT;
int multi = 0;

/*==============================*
 * Defines                      *
 *==============================*/

/*==============================*
 * Functions                    *
 *==============================*/
/* Strip everything after a hash mark, unless hash is preceeded by an escape char ('\') */
void stripComments(char *str)
{
   char *c;

   if((c = strchr(str, '#')) != NULL) {
      if(c != str && *(c-1) == '\\') // Special case
         stripComments(c+1);
      else
         *c = '\0';
   }
}

/* Get the nested key value and update the header */
void nestLine(char *line)
{
   char *h;

   h = hdr + strlen(hdr);

   while(*line == ' ' || *line == '\t')
      ++line; // Skip initial whitespace
   while(*line != ' ' && *line != '\t' && *line != '{')
      *h++ = *line++; // Copy the key value

   *h++ = '.'; // Add a dot to the hdr
   *h = '\0'; // Terminate the hdr
}

/* Extract the key */
void getKey(char *key, char *line)
{
   char *k = key;

   while(*line == ' ' || *line == '\t')
      ++line; // Skip initial whitespace
   while(*line != ' ' && *line != '\t' && *line != '=')
      *k++ = *line++; // Copy the key value
   *k = '\0'; // Terminate the key
}

/* Write out the assignment statement */
void assignLine(char *line)
{
   char key[255];
   char *val;
   char *k;

   if(multi){
	   //syslog(LOG_ERR, "line=[%s]", line);
      if((val = strchr(line, '<')) != NULL){ // Vector, Matrix
         fprintf(outFile, "%s", val);
         // Check for matrix ending on this line
         if((val = strrchr(line, '>')) != NULL){ 
            if(*(val-1) == '>'){
               multi = 0; // Found the end of the matrix
               fprintf(outFile, "\n"); //Finish the line
            }
         }
         else{
            syslog(LOG_ERR, "Invalid matrix structure in parsed file");
         }
      }
      else{
         syslog(LOG_ERR, "Invalid matrix structure in parsed file");
      }
      return;
   }
   
      getKey(key, line);
      line = strchr(line, '=') + 1; // Get on the right side of =
	   //syslog(LOG_ERR, "line=[%s]", line);
   
      if((val = strchr(line, 0x22)) != NULL){ // String
         fprintf(outFile, "%s%s = %s\n", hdr, key, val);
      }else if((val = strchr(line, '<')) != NULL){ // Vector, Matrix
         if(*(val+1) == '<'){ // Matrix
            multi = 1; // Multiple lines are possible
            fprintf(outFile, "%s%s = %s", hdr, key, val);
            // Check for matrix ending on this line
            if((val = strrchr(line, '>')) != NULL){ 
               if(*(val-1) == '>'){
                  multi = 0; // Found the end of the matrix
                  fprintf(outFile, "\n"); //Finish the line
               }
            }
            else{
               syslog(LOG_ERR, "Invalid matrix structure in parsed file");
	       multi = 0;
            }
         }
         else{
            fprintf(outFile, "%s%s = %s\n", hdr, key, val); // Output the Vector
         }
      }else {
         // Array
         do {
            val = line;
            k = strchr(line, ',');
            if(k != NULL) {
               *k = '\0';
               line = k+1;
            }
            fprintf(outFile, "%s%s = %s\n", hdr, key, val);
         } while(k);
      }
}

/* Update the header when exiting a nested statement */
void killLine(char *line)
{
   char *h;
   
   h = hdr + strlen(hdr) - 2;
   while(h > hdr && *(h-1) != '.')
      --h; // Go back to the prev dot or start
   *h = '\0'; // Terminate the hdr
}

/** Create a value-lookup file from a structured configuration file
 
*/
int parseFile(char *fn)
{
   return btParseFile(&btparser_default,fn);
}

/** Initialize the btparser object and scan the file to be parsed.
 
Creates a value-lookup file from a structured configuration file.
Use btParseClose() to delete these files.
 
*/
int btParseFile(btparser *parse_obj,char *filename)
{
   FILE    *inFile;
   char    line[1024], srch[1024], key1[255], key2[255];
   int  addBrace, index;
   long stepPos, srchPos;

   strcpy(parse_obj->tempfile,parse_obj->filename);
   strcat(parse_obj->tempfile,".tmp");
   if((outFile=fopen(parse_obj->tempfile,"w"))==NULL) {
      return(1);
   }

   if((inFile=fopen(filename,"r"))==NULL) {
      fprintf(outFile, "Cannot open %s for reading\n", filename);
      fclose(outFile);
      return(1);
   }

   // Pass 1 = Structure -> Flat File
   hdr[0] = '\0';
   while(1) {
      if(fgets(line, 255, inFile) == NULL)
         break;
      line[strlen(line)-1] = '\0';  // Overwrite newline with termination
      stripComments(line);    // Strip the comments
      //fprintf(outFile, "%s\n", line);
      if(strchr(line, '{') != NULL){
         nestLine(line); // NEST
      }else if(strchr(line, '=') != NULL){
         do{
	    assignLine(line); // ASGN
            if(multi){
	       fgets(line, 255, inFile);
               line[strlen(line)-1] = '\0';  // Overwrite newline with termination
               stripComments(line);    // Strip the comments
	    }
         }while(multi);
      }else if(strchr(line, '}') != NULL){
         killLine(line); // KILL
      }
   }
   fclose(outFile);
   fclose(inFile);

   // Pass 2 = Add braces to non-brace duplicates, apply indices
   inFile = fopen(parse_obj->tempfile, "r+");
   strcpy(parse_obj->flatfile,parse_obj->filename);
   strcat(parse_obj->flatfile,".flt");
   outFile = fopen(parse_obj->flatfile,"w");
   while(1) {
      index = 0;
      if(fgets(line, 1024, inFile) == NULL)
         break;
      line[strlen(line)-1] = '\0';  // Overwrite newline with termination
      stepPos = ftell(inFile);
      getKey(key1, line);
      if(key1[0] == '.')
         continue; // Already processed this line
      addBrace = (strchr(key1, '[') == NULL); // If dup found, add brace?
      while(1) {
         srchPos = ftell(inFile);
         if(fgets(srch, 1024, inFile) == NULL)
            break;
         srch[strlen(srch)-1] = '\0';  // Overwrite newline with termination
         getKey(key2, srch);
         // If the keys are identical
         if(!strcmp(key1,key2)) {
            // First match?
            if(!index) {
               if(addBrace)
                  // Append brace to end of key
                  fprintf(outFile, "%s[%d] = %s\n", key2, index, strchr(line,'=')+1);
               else {
                  // Use existing brace in key
                  *(strchr(key2,']')) = '\0'; // Overwrite brace
                  fprintf(outFile, "%s%d]%s = %s\n", key2, index, key2+strlen(key2)+1, strchr(line,'=')+1);
                  key2[strlen(key2)] = ']'; // Replace brace
               }
               ++index;
            }
            if(addBrace)
               // Append brace to end of key
               fprintf(outFile, "%s[%d] = %s\n", key2, index, strchr(srch,'=')+1);
            else {
               // Use existing brace in key
               *(strchr(key2,']')) = '\0'; // Overwrite brace
               fprintf(outFile, "%s%d]%s = %s\n", key2, index, key2+strlen(key2)+1, strchr(srch,'=')+1);
            }
            ++index;

            // Mark the line as processed
            fseek(inFile, srchPos, SEEK_SET);
            fputc('.', inFile);
            fseek(inFile, srchPos + strlen(srch) + 1, SEEK_SET);
         }
      }
      if(!index)
         fprintf(outFile, "%s\n", line);  // No dups found, output line
      fseek(inFile, stepPos, SEEK_SET);    // Return to present step location
   }
   fclose(outFile);
   fclose(inFile);

   return(0);
}

/** Look up the value of a configuration key.
 
parseFile() must be called prior to this function. 
If it is not the default btparse_obj is used. ("wam.conf")
 
see btParseGetVal()
*/
int parseGetVal(int type, char *find, void *loc)
{
   return btParseGetVal(&btparser_default,type,find,loc);
}

/** Look up the value of a configuration key.
 
parse_obj->filename must have the filename. btparsefile() will be called
automatically if it has not yet been called.
 
\param parse_obj Pointer to a btparser object.
\param type Variable type. See #parsetypes.
\param str The key found in the config file.
\param loc Void pointer where you want the value to be stuffed.
 
\retval 0 Success.
\retval -1 Key not found.
\retval 1 specified type is unknown.
*/
int btParseGetVal(btparser *parse_obj,int type, char *find, void *loc)
{
   int intVal;
   long longVal;
   double doubleVal;
   char key[255];
   char str[1024];
   char buf[1024];
   char *val = NULL, *s;
   FILE *inFile;

   if (parse_obj->filename[0] && !parse_obj->flatfile[0])
      btParseFile(parse_obj,parse_obj->filename);

   inFile = fopen(parse_obj->flatfile,"r");
   while(1) {
      if(fgets(str, 1024, inFile) == NULL)
         break;
      getKey(key, str);
      if(!strcmp(key,find)) {
         // Keys match
         val = strchr(str,'=') + 1; // Get right of =
         break;
      }
   }
   fclose(inFile);
   if(!val) {
      syslog(LOG_ERR, "parseGetVal: key not found [%s]",find);
      return -1; // If key not found, return err
   }
   switch(type) {
   case INT:
      intVal = atoi(val); // Convert
      *(int*)loc = intVal; // Assign
#if BTDEBUG & BTDEBUG_PARSER

      syslog(LOG_ERR, "btParseGetVal:Key[%s]=%d",find,*(int*)loc);
#endif

      break;
   case LONG:
      longVal = atol(val); // Convert
      *(long*)loc = longVal; // Assign
#if BTDEBUG & BTDEBUG_PARSER

      syslog(LOG_ERR, "btParseGetVal:Key[%s]=%d",find,*(long*)loc);
#endif

      break;
   case DOUBLE:
      doubleVal = strtod(val,NULL); // Convert
      *(double*)loc = doubleVal; // Assign
#if BTDEBUG & BTDEBUG_PARSER

      syslog(LOG_ERR, "btParseGetVal:Key[%s]=%f",find,*(double*)loc);
#endif

      break;
   case STRING:
      val = strchr(val,0x22)+1; // Move beyond first dbl quote
      s = (char*)loc;
      intVal = 0;
      while(1) {
         if(*val == '\\') // If escape, copy next char
            val++;
         else if(*val == 0x22) // If dbl quote, quit
            break;

         *s++ = *val++; // Copy the string
      }
      *s = '\0'; // Terminate the string
#if BTDEBUG & BTDEBUG_PARSER

      syslog(LOG_ERR, "btParseGetVal:Key[%s]=%s",find,(char*)loc);
#endif

      break;
   case VECTOR:
      strto_vn((vect_n *)loc, val, "<>");
#if BTDEBUG & BTDEBUG_PARSER

      syslog(LOG_ERR, "btParseGetVal:Key[%s]=%s",find,sprint_vn(buf,(vect_n*)loc));
#endif

      break;
   case MATRIX:
      strto_mn((matr_mn *)loc, val);
#if BTDEBUG & BTDEBUG_PARSER

      syslog(LOG_ERR,
             "btParseGetVal:Key[%s]=%s",find,sprint_mn(buf,(matr_mn*)loc));
#endif

   default:
      return(1);
      break;
   }
   return(0);
}

/** Delete temporary files and reset parse_obj object */
void btParseClose(btparser *parse_obj)
{
   if (parse_obj->flatfile[0]) {
      remove(parse_obj->flatfile);
      parse_obj->flatfile[0] = 0;
   }
   if (parse_obj->tempfile[0]) {
      remove(parse_obj->flatfile);
      parse_obj->tempfile[0] = 0;
   }
}

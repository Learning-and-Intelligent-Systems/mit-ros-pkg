/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btparser.h
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

/** \file btparser.h
    \brief Config file parsing

The functions in btparser.c allow an application to read a structured
    configuration file and extract values from it.

\e myconfigfile 
\verbatim
# Example config file. Comments start with an octothorpe.
system{
    busCount = 1
    bus[]{
        type = "CAN"                # Type of bus
        address = 0                 # Address of robot
    }
    bus[]{
        world = <<1,2,3,3>,<1,2,3,3>,<1,2,3,3>> # World -> Base frame transform
        N[] = 35.87, 28.21, 28.21, 17.77, 10.27, 10.27, 14.93
    }
}
\endverbatim

The values in the above config file could be accessed by:
\code
{
  btparser parse;
  char bus_type[50];
  char buff[100],buff2[20];
  int address,Nvals[15],buscnt,cnt;
  matr_n werld;
  
  werld = new_mn(4,4);
  
  btParseFile(&parse,"myconfigfile");
  btParseGetVal(&parse,INT,"system.busCount",(void*)&buscnt);
  btParseGetVal(&parse,INT,"system.bus[0].address",(void*)&address);
  btParseGetVal(&parse,STRING,"system.bus[0].type",(void*)bus_type);
  for (cnt = 0;cnt < 7;cnt ++){
    sprintf(buff,"system.bus[1].N[%d]",cnt);
    btParseGetVal(&parse,INT,buff,(void*)Nvals);
  }
  btParseGetVal(&parse,INT,"system.bus[1].world",(void*)werld);
}
\endcode

Setting the BTDEBUG_PARSER bit of the BTDEBUG define will dump additional debug
info to syslog.

\todo Matrix Parsing

*/

#ifndef _PARSER_H
#define _PARSER_H

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */

typedef struct {
  char filename[255];
  char tempfile[255];
  char flatfile[255];
}btparser;

#define BTPARSE_INIT {"wam.conf","",""}

int btParseFile(btparser *parse_obj,char *filename);
int btParseGetVal(btparser *parse_obj,int type, char *str, void *loc);
void btParseClose(btparser *parse_obj);

int parseFile(char *fn);
int parseGetVal(int type, char *str, void *loc);
enum parsetypes {INT = 0,LONG,DOUBLE,STRING,VECTOR,MATRIX};

#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /* _PARSER_H */

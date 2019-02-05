/* ======================================================================== *
 *  Module ............. experimental
 *  File ............... thfaststack.h
 *  Creation Date ...... 2005
 *  Author ............. Traveler Hauptman
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

#define STK_SIZE 20
typedef struct {
  int prev_idx; //index of the object that allocated this one
  int idx; //index of this object
  void *obj;
}stkkey;

typedef struct stk_struct{
  struct stk_struct *next;

  size_t sizeofdata; //number of bytes in the stored object 
  size_t offsetofidx;
  size_t offsetofstk;
 // void *objs; //pointer to array of objects
  int max_objs;
  stkkey *keys; //array of free indexes (size: max_objs)
  int keys_end; //Number of indexes available
  int min_idx; //Tracks the amount we have depleted the stack.
}stk;

/**For a given array size, get a pointer to its stack
  if a stack for this size doesn't exist yet, create it*/
stk* get_stk_ptr(int objsize,int offset,int stkoffset); 
void* new_from_stk(stk* istk,int idx); //get a new vect_n from the stack
void release_from_stk(stk* istk,int idx);


typedef struct tst_struct
{
  int some;
  double dat;
  stk* stack;
  int stkidx;
  char name[32];
  struct tst_struct *parent,*next,**freelist;
}tst;

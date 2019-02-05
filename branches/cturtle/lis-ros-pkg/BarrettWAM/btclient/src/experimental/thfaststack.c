/* ======================================================================== *
 *  Module ............. experimental
 *  File ............... thfaststack.c
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

/**
  The goal of this module is to provide for fast temporary memory allocation
  for data.
  
  Speed of allocation is treated as most important. deallocation does the bulk of
  the work.
  
  This will be used for an efficient vector math library.
  
*/
#include <stdlib.h>
#include "thfaststack.h"
stk* stkhead = NULL;
/** Return pointer to stack object for a given size.
For a given array size, get a pointer to its stack
if a stack for this size doesn't exist yet, create it*/
stk* get_stk_ptr(int objsize,int offset,int stkoffset)
{
  stk* sp,spend;
  void* vp;
  stk** vp2;
  int cnt;
  
  sp = stkhead;
  if (sp != NULL)
    while (sp != NULL && sp->sizeofdata != objsize)
      sp = sp->next;
  
  if (sp != NULL)
  {
    return sp;
  }
  else
  {
    sp = (stk*)malloc(sizeof(stk));
    sp->next = stkhead;
    stkhead = sp;
    
    vp = (void*)malloc(STK_SIZE * (sizeof(stkkey) + (size_t)objsize));
    sp->keys = (stkkey*)vp;
    sp->max_objs = STK_SIZE;
    sp->keys_end = STK_SIZE;
    sp->offsetofidx = offset;
     sp->offsetofstk = stkoffset;
    sp->min_idx = STK_SIZE;
    vp = vp + sizeof(stkkey)*STK_SIZE;
    for (cnt = 0; cnt < STK_SIZE;cnt++){
      sp->keys[cnt].obj = vp;
      sp->keys[cnt].idx = cnt;
      vp2 = (stk**)(vp + sp->offsetofstk);
      *vp2 = sp;
      vp = vp + objsize;
    }
    return sp;
    //new stack of size "size"
  }
}

/** Return a pointer to memory allocated for your object
*/
void* new_from_stk(stk* istk,int idx) //get a new vect_n from the stack
{
  int end,obj_idx,*idxp;
  void* vp;
  void* vp2;
  end = --istk->keys_end;
  obj_idx = istk->keys[end].idx;
  istk->keys[obj_idx].prev_idx = idx;
  vp = istk->keys[obj_idx].obj;
  idxp = (int*)(vp + istk->offsetofidx);
  *(idxp) = obj_idx;
  return vp;
}

/** Release memory that you were previously using */

void release_from_stk(stk* istk,int idx)
{
  int this_idx,end;
  
  this_idx = idx;
  while (this_idx != -1)
  {
    end = istk->keys_end;
    if (end < istk->min_idx) istk->min_idx = end;
    istk->keys[end].idx = this_idx;
    this_idx = istk->keys[this_idx].prev_idx;
    istk->keys_end ++;
  }
}
tst* freelist = NULL;




tst* new_tst()
{
  tst* p;
  p = malloc(sizeof(tst));
  p->stack = get_stk_ptr(sizeof(tst),sizeof(int)+sizeof(double)+sizeof(stk*),sizeof(int)+sizeof(double));
  p->stkidx = -1;
  p->parent = NULL;
  p->freelist = &freelist;
  return p;
}

tst* new_from_list(tst* inp)
{
  tst* ret;
  if(*inp->freelist != NULL){
    
    ret = *inp->freelist;
    ret->parent = inp;
    *inp->freelist = (*inp->freelist)->next;
  }
  else {
    ret = new_tst();
  }
  return ret;
}

void release_to_list(tst* inp)
{
  tst* p,p2;
  p = inp;
  while(p != NULL){
    p->next = *p->freelist;
    *p->freelist = p;
    p = p->parent;
  }  
}


int main()
{
  tst *t1,*t2,*t3,*t4,*t5,*t6,*t7,*t8,*t9;
  tst* tray[10];
  int cnt;
  
  t1 = new_tst();
  t1->some = 1;
  t3 = (tst*)new_from_stk(t1->stack,t1->stkidx);
  t3->some = 2;
  t4 = (tst*)new_from_stk(t3->stack,t3->stkidx);
  t4->some = 3;
  t2 = new_tst();
  t2->some = 1;
  t5 = (tst*)new_from_stk(t2->stack,t2->stkidx);
  t5->some = 4;
  t6 = (tst*)new_from_stk(t5->stack,t5->stkidx);
  t6->some = 5;
  release_from_stk(t4->stack,t4->stkidx);
  t7 = (tst*)new_from_stk(t1->stack,t1->stkidx);
  t7->some = 6;
  t8 = (tst*)new_from_stk(t7->stack,t7->stkidx);
  t8->some = 7;
  t9 = (tst*)new_from_stk(t8->stack,t8->stkidx);
  t9->some = 8;
  release_from_stk(t4->stack,t4->stkidx);
  printf("\n%d %d %d %d %d %d %d %d %d %d\n",t1->some,t2->some,t3->some,t4->some,t5->some,t6->some,t7->some,t8->some,t9->some,t1->stack->min_idx);
  
  for (cnt = 0;cnt < 10;cnt ++){
    tray[cnt] = new_tst();
    tray[cnt]->some = cnt;
  }
  printf("\nReleasing\n");
  for (cnt = 0;cnt < 10;cnt ++){
    release_to_list(tray[cnt]);
    printf("%d ",freelist->some);
  }
  tray[0] = new_tst();
  printf("\ngettin new ones\n");
  printf("%d ",tray[0]->some);
  for (cnt = 1;cnt < 10;cnt ++){
    tray[cnt] = new_from_list(tray[cnt-1]);
    printf("%d ",tray[cnt]->some);
  }
  printf("\nPrint freelist\n");
    t1 = freelist;
  while(t1 != NULL){
    printf("%d ",t1->some);
    t1 = t1->next;
  }
  printf("\nReleasing from 7\n");
  release_to_list(tray[6]);
  t1 = freelist;
  while(t1 != NULL){
    printf("%d ",t1->some);
    t1 = t1->next;
}
  printf("\n");
  return 0;
}

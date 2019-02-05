/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btmath.c
 *  Creation Date ...... 18 Feb 2005
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
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

#ifdef S_SPLINT_S
#include <err.h>
#endif
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>
#include <regex.h>
#include "btmath.h"

#define MAX_LINE_LENGTH     255
#define MAX_BTPTRS 1000
#define MAX_VECTOR_SIZE 1000

//#DEFINE VECT_N_DEBUG
//#DEFINE VECT_N_BOUNDS  //perform bounds checking
//#DEFINE MATR_N_DEBUG
//#define BT_NULL_PTR_GUARD
//#define VECT_SIZE_CHK

int num_btptrs = 0;
void *btptrs[MAX_BTPTRS];

/** (internal) Adds a pointer to the garbage collection list
 
 
*/
int addbtptr(void *ptr)
{
   if (num_btptrs < MAX_BTPTRS) {
      num_btptrs++;
      btptrs[num_btptrs] = ptr;
      return num_btptrs;
   } else {
      syslog(LOG_ERR,"btptr: cannot free this ptr %p",ptr);
      return 0;
   }
}

int rembtptr(int idx)
{
   if (idx >= 0 || idx <= MAX_BTPTRS) {
      free(btptrs[idx]);
      btptrs[idx] = NULL;
   }
}

void freebtptr()
{
   int cnt;
   syslog(LOG_ERR,"btptr: Releasing %d pointers",num_btptrs);
   for(cnt = 0;cnt < num_btptrs; cnt++) {
      if (btptrs[cnt] != NULL)
         free(btptrs[cnt]);
   }
}

/** Sets each element of a vector to the specified value
*/
BTINLINE void fill_vn(vect_n* dest, btreal val)
{
   int cnt;

   BTPTR_OK(dest,"fill_vn");

   for (cnt=0;cnt < dest->n;cnt++)
      dest->q[cnt] = val;
}

/** Allocate memory for a vect_n object
 
  \todo Add file/string buffer read and write functions
  
  new_vn() must be called for each vect_n object you wish to use. It allocates 
  memory and sets each element of the vector to 0.0.
  
  For temporary (local scope) variables use init_local_vn().
  
  \param size Valid values >= 1
  \return A pointer to the newly created vect_n object
  
  \code
  Diagram of memory allocated by new_vn(4);
         
           +-----+
           | ret |--+
           |-----|  |
           |  n  |  |
           |-----|  |
   +-------|  q  |  |
   |       |-----|  |
   | +-> +-| ret |<-+
   | +---+ |-----|
   |       |  n  |
   |       |-----|
   |       |  q  |--+
   |       |-----|  |
   +------>| val |  |
           | val |  |
           | val |  |
           | val |  |
           |-----|  |
           | val |<-+
           | val |
           | val |
           | val |
           +-----+
  \endcode
  
  \dot
  digraph vect_n {
      node [shape=record, fontname=Helvetica, fontsize=10];
      vn [label="{<p1> *ret|<p2>n|<p3>*q|<p4> *ret|<p5>n|<p6>*q|<p7>data\n[...]|<p8>data\n[...]}"];
      vn:p1:e -> vn:p4:e
      vn:p4:e -> vn:p4:e
      vn:p3:e -> vn:p7:e
      vn:p6:e -> vn:p8:e
  }
  \enddot
  \internal
  \bug depending on how data is allocated, this library will behave differently. I am 
  starting with lots of data allocation, however, *r of each vector can probably 
  point to the same data structure. *u is not necessarily needed 
*/
vect_n * new_vn(int size) //allocate an n-vector
{
   void* vmem;
   vect_n *n;

#ifdef BT_ARRAY_BOUNDS_CHECK

   idx_bounds_ok(size,MAX_VECTOR_SIZE,"new_vn");
#endif

   //allocate mem for vector,return vector, and return structure
   vmem = btmalloc(2*size*sizeof(btreal)+2*sizeof(vect_n));

   //addbtptr(vmem);
   n = init_vn((vect_n*)vmem, size);
   fill_vn(n,0.0);
   fill_vn(n->ret,0.0);
   return n;
}

/** Takes a vect_n* to a block of memory and initializes the
vect_n data structures.
 
Values are not initialized
 
note: a vect_n* instead of a void* is used for convinience when
linking up stack allocated vect_n's.
 
*/
vect_n* init_vn(vect_n* v, int size)
{
   void* vmem;
   vect_n* n;

   vmem = (void*)v;
   n = (vect_n*)vmem;
   n->n = size;
   n->ret = (vect_n*)(vmem + sizeof(vect_n));
   n->q = (btreal*)(vmem+ 2*sizeof(vect_n));
   n->ret->n = size;
   n->ret->q = (btreal*)(vmem + 2*sizeof(vect_n) + size*sizeof(btreal));
   n->ret->ret = n->ret;
   return n;
}

/** Free the memory of a vect_n object
  This function frees the memory previously allocated. Instead of calling destroy_vn()
  it is easier to call freebtptr() at the end of your program to free all the btmath
  objects.
  
*/
void destroy_vn(vect_n **p)
{
   btfree((void**)p);
}

/** Create a local vect_n
 
\param header Pointer to array of size 2 of vect_n allocated on the local stack
\param data Pointer to array of size 2*len of btreal on the local stack
 
\code Usage
void some_function(vect_n* dest, vect_n* src)
{
    vect_n tmp_vn[2],tmp2_vn[2];
    btreal tmp_btreal[8],tmp_btreal2[100];
    vect_n *tmp;
    
    tmp = init_local_vn(tmp_vn,tmp_btreal,4); //size 4, clear way
    init_local_vn(tmp2_vn,tmp_btreal,6); //size 6 tricky way
    
    set_vn(tmp_vn,scale_vn(2.0,src));
    set_vn(tmp2_vn,src);
}
\endcode
*/
vect_n* init_local_vn(vect_n* header,btreal* data, int size)
{
   vect_n *n;

   n = header;
   n->n = size;
   n->ret = header + 1;
   n->q = data;
   n->ret->n = size;
   n->ret->q = data + size;
   n->ret->ret = n->ret;
#ifdef BT_ARRAY_BOUNDS_CHECK

   idx_bounds_ok(size,MAX_VECTOR_SIZE,"init_local_vn");
#endif

   fill_vn(n,0.0);
   //fill_vn(n->ret,0.0);
   return n;
}

/** Returns the number of elements in src
*/
int len_vn(vect_n *src)
{
   //number of elements in the vector
   return src->n;
}

/** Returns the memory footprint of vect_n src
*/
int sizeof_vn(vect_n *src)
{
   //return sizeof info for whole vector
   return 2*src->n*sizeof(btreal)+2*sizeof(vect_n);
}

/** Allocate a group of pointers
\todo This should just allocate one chunk of memory instead of calling new_vn 
multiple times
*/
int new_vn_group(int num, int size, ...)
{
   //allocate a group of n-vectors
   int cnt;
   vect_n** vp;
   va_list ap;
   va_start(ap,size);
   //th Add vector size checking code here
   for (cnt=0;cnt < num;cnt++) {
      vp = va_arg(ap, vect_n**);
      *vp = new_vn(size);
   }
   va_end(ap);

}

/** Copy src vect_n to dest vect_n
 
  src must have equal or fewer elements than dest. 
  
*/
BTINLINE void set_vn(vect_n* dest, vect_n* src)
{
   //assignment, copy
   int cnt,max;

   BTPTR_OK(dest,"set_vn dest")
   BTPTR_OK(src,"set_vn src")

#ifdef BT_ARRAY_BOUNDS_CHECK
   if(!idx_bounds_ok(dest->n,MAX_VECTOR_SIZE,"set_vn dest")) {
      syslog(LOG_ERR,"btmath:set_vn:dest:pointer:%x",dest);
      return;
   }
   if(!idx_bounds_ok(src->n,MAX_VECTOR_SIZE,"set_vn src")) {
      syslog(LOG_ERR,"btmath:set_vn:src:pointer:%x:dest:pointer:%x",src,dest);
      return;
   }
#endif

   if (dest->n < src->n)
      max = dest->n;
   else
      max = src->n;
   for (cnt=0;cnt < max;cnt++)
      dest->q[cnt] = src->q[cnt];
}

/** Copy a block of elements from src vect_n to dest vect_n
*/
void setrange_vn(vect_n* dest, vect_n* src, int dest_start, int src_start, int num)
{
   int cnt,max;
   BTPTR_OK(dest,"setrange_vn dest")
   BTPTR_OK(src,"setrange_vn src")

#ifdef BT_ARRAY_BOUNDS_CHECK
   if(!idx_bounds_ok(dest_start,dest->n-1,"setrange_vn dest")) {
      syslog(LOG_ERR,"btmath:set_vn:dest:pointer:%x",dest);
      return;
   }
   if(!idx_bounds_ok(src_start,src->n-1,"setrange_vn src")) {
      syslog(LOG_ERR,"btmath:set_vn:src:pointer:%x:dest:pointer:%x",src,dest);
      return;
   }
#endif


   for (cnt=0;cnt < num;cnt++)
      dest->q[dest_start + cnt] = src->q[src_start + cnt];

}

/** Copy elements of vect_n src to a btreal array
 
*/
BTINLINE void extract_vn(btreal* dest, vect_n* src) //copy vector to a btreal array
{
   int cnt;
   //th Add vector size checking code here
   for (cnt=0;cnt < src->n;cnt++)
      dest[cnt] = src->q[cnt];

}

/** Return a pointer to the btreal array used by this vect_n object
*/
BTINLINE btreal* valptr_vn(vect_n* src)
{
   return src->q;
}

/** Copy elements of a btreal array to vect_n src
 
*/
BTINLINE void inject_vn(vect_n* dest, btreal* src)
{
   //copy btreal array to vector
   int cnt;
   //th Add vector size checking code here
   for (cnt=0;cnt < dest->n;cnt++)
      dest->q[cnt] = src[cnt];
}

/** Set the value of a single element in vect_n dest
 
\param dest Target vect_n object
\param idx Index of desired element of dest. The first element is idx=0.
*/
BTINLINE void setval_vn(vect_n* dest, int idx, btreal val)
{

   BTPTR_OK(dest,"setval_vn")

#ifdef BT_ARRAY_BOUNDS_CHECK
   if(!idx_bounds_ok(idx,dest->n,"setval_vn"))
      syslog(LOG_ERR,"btmath ERROR:pointer is %p",dest);
#endif

   dest->q[idx] = val;
}

/** Returns the value of a single element in vect_n dest
 
\param dest Target vect_n object
\param idx Index of desired element of dest. The first element is idx=0.
*/
BTINLINE btreal getval_vn(vect_n* dest, int idx)
{
   BTPTR_OK(dest,"getval_vn")

#ifdef BT_ARRAY_BOUNDS_CHECK
   if(!idx_bounds_ok(idx,dest->n,"getval_vn"))
      syslog(LOG_ERR,"btmath ERROR:pointer is %p",dest);
#endif

   return dest->q[idx];
}

/** Set the values of a vect_n object
\bug If the function arguments are constant integers (instead of constant btreals)
you will get bogus results and segfaults. ex: const_vn(&t1,2.0,3.0,3.0) is OK, const_vn(&t1,2,3,3) is NOT
*/
vect_n* const_vn(vect_n* a, ...)
{
   //set vector to a constant array of btreals
   int cnt;
   btreal val;
   va_list ap;
   va_start(ap,a);
   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++) {
      a->q[cnt] = va_arg(ap, btreal);
   }
   va_end(ap);
   return a;
}

/** Initialize a single element of dest to 1.0
\warning No bounds checking for i
*/
BTINLINE void einit_vn(vect_n* dest,int i)
{
   // einit_vn(&a,3) = <0,0,0,1,0>
   fill_vn(dest,0.0);
   dest->q[i] = 1.0;
   //setval_vn(dest,1.0);
}

vect_n* subset_vn(vect_n* src,int start,int end)
{
   int cnt;
   BTPTR_OK(src,"subset_vn")
#ifdef BT_ARRAY_BOUNDS_CHECK
   if((end-start) < 0 || (end-start)>src->n)
      syslog(LOG_ERR,"btmath ERROR:subset_vn tried to use: start:%d end:%d",start,end);
#endif

   src->ret->n = end - start;
   for (cnt=0;cnt < src->ret->n;cnt++)
      src->ret->q[cnt] = src->q[start + cnt];
   return src->ret;
}

void reset_vn(vect_n* src)
{
   src->ret->n = src->n;
}

/** Vector Negate
 
a[i] = -1.0 * a[i]
*/
BTINLINE vect_n* neg_vn(vect_n* a)
{
   //negate a vector
   int cnt;
   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      a->ret->q[cnt] = -1.0*a->q[cnt];
   return a->ret;
}

/** Vector Scalar Multiply
 
a[i] = x * a[i]
*/
BTINLINE vect_n* scale_vn(btreal x, vect_n* a)
{
   int cnt;
   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      a->ret->q[cnt] = x*a->q[cnt];
   return a->ret;
}

/** Vector Addition
 
ret[i] = a[i] + b[i]
*/
BTINLINE vect_n* add_vn(vect_n* a, vect_n* b)
{
   int cnt;
   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      a->ret->q[cnt] = a->q[cnt] + b->q[cnt];
   return a->ret;
}

/** Vector Subtraction
 
ret[i] = a[i] - b[i]
*/
BTINLINE vect_n* sub_vn(vect_n* a, vect_n* b)
{
   int cnt;
   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      a->ret->q[cnt] = a->q[cnt] - b->q[cnt];
   return a->ret;
}

/*
BTINLINE vect_n* wedge_vn(vect_n* a, vect_n*b) // need to figure this out
{
}*/

/** Vector Dot Product
 
ret = a[0] * b[0] + ... + a[i] * b[i]
*/
BTINLINE btreal dot_vn(vect_n* a, vect_n* b)
{
   int cnt;
   btreal res = 0.0;
   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      res += a->q[cnt] * b->q[cnt];
   return res;
}

/** Vector Norm
 
ret = sqrt(dot_vn(a,b));
*/
BTINLINE btreal  norm_vn(vect_n* a)
{
   //euclidian norm
   btreal res = 0.0;
   int cnt;
   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      res += a->q[cnt] * a->q[cnt];
   return sqrt(res);
}

/** Angle between vectors
 
Cos(theta) = a dot b / (norm(a)*norm(b)
*/
BTINLINE btreal  angle_vn(vect_n* a,vect_n* b)
{
   btreal den;

   den = norm_vn(a) * norm_vn(b);
   if (den > PRACTICALLY_ZERO) {
      return acos(dot_vn(a,b)/den);
   } else
      return acos(0.0);
}

/** Unit Vector
 
ret = a/norm_vn(a);
*/
BTINLINE vect_n* unit_vn(vect_n* a)
{
   //unit vector
   btreal div;
   int cnt;

   //th Add vector size checking code here
   div = norm_vn(a);
   if (div > PRACTICALLY_ZERO) {
      for (cnt=0;cnt < a->n;cnt++)
         a->ret->q[cnt] = a->q[cnt]/div;
   } else {
      for (cnt=0;cnt < a->n;cnt++)
         a->ret->q[cnt] = 0.0;
   }
   return a->ret;
}

/** Interpolate between two vectors
 
Returns a point that is distance s along the line from a to b. The direction
towards b is considered positive. s is 0 to dist(a,b);
 
\param s Distance along line from a to b
\param a Starting point
\param b End point
\return add_vn(a,scale_vn(s,unit_vn(sub_vn(b,a))));
*/
vect_n* unit_interp_vn(vect_n* a, vect_n* b,btreal s)
{
   return add_vn(a,scale_vn(s,unit_vn(sub_vn(b,a))));
}

/** Interpolate between two vectors with another paramater
 
The vectors are combined with another dimension t.
For instance, if you have two vectors with an externally stored time, you
can interpolate between them in time.
\code
(v2 - v1) / (t2-t1) = (vx - v1) / (tx - t1)
dt = t2 - t1
t = tx - t1
(v2 - v1) / dt = (vx - v1) / t
v1 + (v2 - v1) t/dt = vx
\endcode
 
\param dt Difference between end point and starting point
\param t Distance in dimension t you want to interpolate for.
\param b End point
\return add_vn(a,scale_vn(s,unit_vn(sub_vn(b,a))));
*/
vect_n* interp_vn(vect_n* a, vect_n* b,btreal dt,btreal t)
{
   if(dt == 0.0){
      return(a);
   }else{
      return add_vn(a,scale_vn(t/dt,sub_vn(b,a)));
   }
}

/** Force a vect_n to be in the range min value to max value (inclusive)
 
  If the value of any element in the vector is out of range, it is set to the 
  closest boundary value.
 
*/
BTINLINE vect_n* bound_vn(vect_n* a, btreal min, btreal max)
{
   int cnt;
   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++) {
      if (a->q[cnt] > max)
         a->ret->q[cnt] = max;
      else if (a->q[cnt] < min)
         a->ret->q[cnt] = min;
   }
   return a->ret;

}

/** Per Element Multiply
 
ret[i] = a[i] * b[i]
*/
BTINLINE vect_n* e_mul_vn(vect_n* a, vect_n* b)
{
   // Per Element multiply
   int cnt;

   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      a->ret->q[cnt] = a->q[cnt] * b->q[cnt];
   return a->ret;
}

/** Per Element Divide
 
ret[i] = a[i] / b[i]
*/
BTINLINE vect_n* e_div_vn(vect_n* a, vect_n* b)
{
   // Per Element multiply
   int cnt;

   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
   {
      // Check for divide by zero ...
      if (b->q[cnt] == 0.0)
      {
         syslog(LOG_ERR,"Attempt to divide by 0 in e_div_vn!");
         return a;
      }
      a->ret->q[cnt] = a->q[cnt] / b->q[cnt];
   }
   return a->ret;
}

/** Per element power
  ret[i] = a[i]^b
*/
BTINLINE vect_n* e_pow_vn(vect_n* a, btreal b)
{
   int cnt;

   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      a->ret->q[cnt] = pow(a->q[cnt], b);
   return a->ret;
}

/** Per element square root
 ret[i] = sqrt(a[i])
*/
BTINLINE vect_n* e_sqrt_vn(vect_n* a)
{
   int cnt;

   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      a->ret->q[cnt] = sqrt(a->q[cnt]);
   return a->ret;
}

/** Per element square
 ret[i] = a[i]^2
*/
BTINLINE vect_n* e_sqr_vn(vect_n* a)
{
   int cnt;

   //th Add vector size checking code here
   for (cnt=0;cnt < a->n;cnt++)
      a->ret->q[cnt] =  a->q[cnt] * a->q[cnt];
   return a->ret;
}

/** Use printf to print values of a vect_n
 This function prints three lines for each vector. Consider using sprint_vn instead.
*/
void print_vn(vect_n* src)
{
   int i,j;
   if (src != NULL) {
      printf("<");
      for(j = 0;j<src->n;j++)
         printf("%g,",src->q[j]);
      printf(">");
   }
}

/** Print vect_n to a string buffer suitable for display or text file
 
\code
< 1.2, 2.4, 5.3>
\endcode
 
\code
void main()
{
  char buff1[100],buff2[100];
  vect_n x,v;
  
  v = new_vn(5);
  x = new_vn(5);
  fill_vn(v,3.0);
  set_vn(x,scale_vn(4.2,v));
  printf("x: %s \nv: %s",sprint_vn(buff1,v),sprint_vn(buff2,v));
}
\endcode
*/
char* sprint_vn(char *dest,vect_n* src)
{
   int i,j;

   if (btptr_ok(src,"sprint_vn")) {
      dest[0] = '<';
      dest[1] = 0;
      for(j = 0;j<src->n;j++)
         sprintf(dest+strlen(dest),"%8.4f,",src->q[j]);
      dest[strlen(dest)-1] = 0;
      strcat(dest,">");
   }
   return dest;
}

/**Print a vector out to syslog*/
void syslog_vn(char* header,vect_n* src)
{
   char buffer[100];
   syslog(LOG_ERR,"%s %s",header,sprint_vn(buffer,src));
}

/** Print vect_n to a string buffer suitable for insertion into a comma delimited
file.
 
\code
 1.2, 2.4, 5.3
\endcode
 
see: sprint_vn()
*/
char* sprint_csv_vn(char *dest,vect_n* src)
{
   int i,j;
   if (btptr_ok(src,"sprint_cvs_vn")) {
      dest[0] = 0;
      for(j = 0;j<src->n;j++)
         sprintf(dest+strlen(dest)," %f,",src->q[j]);
      dest[strlen(dest)-1] = 0;
   }
   return dest;
}

/** Print vect_n to a string buffer suitable for insertion into gnu plot
 
\code
 1.2 2.4 5.3
\endcode
 
see: sprint_vn()
*/
char* sprint_plt_vn(char *dest,vect_n* src)
{
   int i,j;
   if (btptr_ok(src,"sprint_cvs_vn")) {
      dest[0] = 0;
      for(j = 0;j<src->n;j++)
         sprintf(dest+strlen(dest)," %8.4f",src->q[j]);
      dest[strlen(dest)-1] = 0;
   }
   return dest;
}

/** Counts number of double values in string to be parsed.
The string must be in comma delimited format. It ends with the
end-vector delimiter provided. ' ' for the start delimiter is considered to be
NO delimiter. 
Use this for text vectors: strcount_vn(&string, "<>") for "< 1.2, 2.3>"
Use this for CSV vectors: strcount_vn(&string, " \n\r") for "1.3, 54.3\r"
 
\param src Pointer to a pointer to the string. It is updated to point to the first 
delimiter character
\param delimiter Constant string with 3 characters. 1 Vector opening character and 2 vector closing characters.
\return src_string is stuffed with the address of the first character past the start character
 - -1 = No starting delimiter found
 - -2 = Too many starting delimiters found
 - -3 = No ending delimiter found
 - -4 = Invalid characters found in string
\warning If there are no valid values ("e,e+-" for example) this returns a  1, which 
is false.
*/
int strcount_vn(char **src_string,char *delimiter)
{
   char valid[25] = "1234567890.,eE-+ \t   ";
   char *start1,*second,*scan,*src;
   double tmp;
   vect_n *dest;
   int cnt = 0,done = 0,num = 0,tst;

   //check validity "1234567890.,eE-+ \t"
   valid[18] = delimiter[0]; //opening character
   valid[19] = delimiter[1]; //closing character
   valid[20] = delimiter[2]; //alternate closing character

   src = *src_string;
   if( strcspn(src,valid))
      return -4;
   start1 = src;
   //find first '<'
   if (delimiter[0] != 0 && delimiter[0] != ' ') {
      start1 = strchr(src,delimiter[0]);
      if (start1 == NULL) {
         syslog(LOG_ERR,"sscan_vn: No starting <");
         return -1;
      } //couldn't find start1ing '<'
      //look for another '<', abort if it's there
      start1 += 1; //point to character after delimiter
      second = strchr(start1,delimiter[0]);
      if (second != NULL) {
         syslog(LOG_ERR,"sscan_vn: Too many <");
         return -2;
      } //found a second '<', this is probably a matrix
   }

   //count number of values

   scan = start1;
   while(!done) {
      tmp = strtod(scan,&second);

      //printf("\n%s :: %s :: %d :: %d",scan,second,num,*second);
      while (*second == ' ' || *second == '\t')
         second++; //skip to the end of any whitespace
      if (*second == ',') {
         num++;
         scan = second + 1;
      } else if (*second == delimiter[1] || *second == delimiter[2]) {
         num++;
         done = 1;
      } else {
         done = 1;
      }
   }
   if (num == 0) {
      syslog(LOG_ERR,"sscan_vn: No values");
      return 0;
   } //there were apparently no values
   //create vector with size num_values
   if ((*second != delimiter[1]) && (*second != delimiter[2])) {
      syslog(LOG_ERR,"sscan_vn: No ending >, %c, %d, %d, %d",*second,*second,delimiter[1],delimiter[2]);
      return -3;
   } //expected a terminating '>' but did not find one

   *src_string = start1; //return the address of the first delimiter
   return num;
}

/** Convert a string to a vect_n
  
\param src "< -1, 3.4e3, ,+.9>" - Empty values interpreted as 0.0
 
*/
vect_n * strto_vn(vect_n *dest,char *src,char *delimiters)
{
   char *start1,*second,*scan;
   int cnt = 0,num = 0;

   start1 = src;
   num = strcount_vn(&start1,delimiters); // validate string and count the number of values

   scan = start1;

   //while !done, vect.q[cnt] = strtod()
   for (cnt = 0; cnt < num; cnt++) {
      if (cnt > dest->n) {
         syslog(LOG_ERR,"strto_vn: cnt (%d) > dest->n (%d), num=%d, dest=%x",cnt, dest->n, num, dest);
         break;
      }
      setval_vn(dest,cnt,strtod(scan,&second));
      scan = strpbrk(second,",");
      if (scan != NULL)
         scan = scan + 1; //skip over the comma
   }

   return dest;
}

/** Create a new vect_n and convert a string to a vect_n
 
  A new vect_n of the appropriate size is allocated during the course of this function
  
\param src "< -1, 3.4e3, ,+.9>" - Empty values interpreted as 0.0
 
*/
vect_n * sscan_vn(char *src)
{
   char *start1,*second,*scan;
   vect_n *dest;
   int cnt = 0,num = 0;

   start1 = src;
   num = strcount_vn(&start1,"<>"); // validate string and count the number of values
   if (num <= 0) {
      return NULL;
   } else {
      dest = new_vn(num);

      scan = start1;

      //while !done, vect.q[cnt] = strtod()
      for (cnt = 0; cnt < num; cnt++) {
         setval_vn(dest,cnt,strtod(scan,&second));
         scan = strpbrk(second,",");
         if (scan != NULL)
            scan = scan + 1; //skip over the comma
      }

      return dest;
   }
}

/** Convert a CSV string to a vect_n
  
\param src " -1, 3.4e3, ,+.9" - Empty values interpreted as 0.0
\param dest vect_n
 
 
*/
vect_n * csvto_vn(vect_n* dest, char *src)
{
   char *start1,*second,*scan;
   int cnt = 0,num = 0;

   start1 = src;
   num = strcount_vn(&start1," \n\r"); // validate string and count the number of values
   if (num < 0)
      syslog(LOG_ERR,"strto_vn strcount_vn error %d",num);
   scan = start1;

   //while !done, vect.q[cnt] = strtod()
   for (cnt = 0; cnt < num; cnt++) {
      setval_vn(dest,cnt,strtod(scan,&second));
      scan = strpbrk(second,",");
      if (scan != NULL)
         scan = scan + 1; //skip over the comma
   }

   return dest;
}

/** \todo unfinished
*/
int test_vn(btreal error)
{
   vect_n *c1,*c2,*c3,*c4;
   vect_n *b1,*b2;
   btreal a1,a2,a3;

   c1 = new_vn(4);
   c2 = new_vn(4);
   c3 = new_vn(4);


   printf("\nThe following should be <5,4,3,2>");
   const_vn(c1,5.0,4.0,3.0,2.0);
   printf("\n<%g,%g,%g,%g>\n\n",getval_vn(c1,0),getval_vn(c1,1),getval_vn(c1,2),getval_vn(c1,3));

   printf("\nSet: <5,4,3,2>");
   set_vn(c2,c1);
   print_vn(c2);

   printf("\nSub,Add: <5,4,3,2>");
   set_vn(c3,add_vn(c1,sub_vn(c1,c2)));
   print_vn(c3);

   printf("\nNeg,Scale: <-15,-12,-9,-6>");
   set_vn(c3,scale_vn(3,neg_vn(c1)));
   print_vn(c3);

   printf("\nUnit: <0,0,0,0>");
   set_vn(c3,unit_vn(c1));
   print_vn(c3);

   printf("\nDot: %g | %g",5.0*5.0+4.0*4.0+3.0*3.0+2.0*2.0,dot_vn(c1,c2));
   printf("\nNorm: %g | %g",sqrt(5*5+4*4+3*3+2*2),norm_vn(c1));

   printf("\nSum abuse: 5*<5,4,3,2>");
   set_vn(c3,add_vn(c1,add_vn(c1,add_vn(c1,add_vn(c1,c1)))));
   print_vn(c3);

   printf("\nsscan1");
   c4 = sscan_vn(" < 4.3, 5 , 89, 98, 100 >");
   if (c4 != NULL)
      print_vn(c4);

   printf("\nsscan2");
   c4 = sscan_vn(" < 4.3 5, 89, 98, 100 > ");
   if (c4 != NULL)
      print_vn(c4);

   printf("\nsscan3");
   c4 = sscan_vn(" < 4.3,u 5, 89, 98, 100 >");
   if (c4 != NULL)
      print_vn(c4);

   printf("\nsscan4");
   c4 = sscan_vn(" < 4.3,, 89, 98, 100 >");
   if (c4 != NULL)
      print_vn(c4);

   printf("\nsscan4");
   c4 = sscan_vn(" < <4.3,, 89, 98, 100 >");
   if (c4 != NULL)
      print_vn(c4);



}


/**************************************** End vect_n functions ****************/

/**************************************** vectray functions ****************/
/** Allocate an array of vect_n data in one block
  vect_ray has buffer overrun data so that idx_vr(ray,-1) and idx_vr(ray,max_rows + 1) are valid
*/
vectray * new_vr(int vect_size,int max_rows)
{
   void* vmem;
   vectray *n;
   int cnt;
   //allocate mem for vector,return vector, and return structure
   vmem = btmalloc(vect_size*(max_rows + 2)*sizeof(btreal)+sizeof(vectray));

   n = (vectray*)vmem;
   n->n = vect_size;
   n->max_rows = max_rows;
   n->num_rows = 0; //the number of rows that are filled in the array
   n->rayvect = new_vn(vect_size);
   n->lval = new_vn(vect_size);
   n->rval = new_vn(vect_size);
   n->eval = new_vn(vect_size);
   n->stride = vect_size;
   n->edit_point = 0;
   n->data = (btreal*)(vmem + sizeof(vectray)+ sizeof(btreal)*vect_size);

   for (cnt = 0; cnt < max_rows; cnt ++)
      fill_vn(idx_vr(n,cnt),0.0);
   return n;
}

void destroy_vr(vectray **vr)
{
   btfree((void**)vr);
}

/** Set internal vect_n proxy to the specified index.
 
You probably want to use getvn_vr instead!
 
sets the internal vector structure to point to the indexed data and returns the
  pointer. CAUTION! You cannot use this in _vn math functions. only use this with 
  set_vn()
  
  
*/
BTINLINE vect_n * idx_vr(vectray *ray,int idx)
{
   ray->rayvect->q = &(ray->data[ray->stride * idx]);
   return ray->rayvect;
}

vect_n * lval_vr(vectray *ray,int idx)
{
   ray->lval->q = &(ray->data[ray->stride * idx]);
   return ray->lval;
}

vect_n * rval_vr(vectray *ray,int idx)
{
   ray->rval->q = &(ray->data[ray->stride * idx]);
   return ray->rval;
}

vect_n * edit_vr(vectray *ray)
{
   ray->rval->q = &(ray->data[ray->stride * ray->edit_point]);
   return ray->rval;
}

/** Reroute a vect_n data pointer to a block of data in a vector array
 
This is a dangerous function to use. Read the code for more info.
*/
/* Use this when you need to do multi vector operations on elements of a
vectray but you don't want to copy the data each time. You are essentially using
an externally allocated vect_n for scratch space. Do a search of the code for 
examples of use.
*/
BTINLINE vect_n * mapdat_vr(vect_n *dest, vectray *ray, int idx)
{
   dest->q = &(ray->data[ray->stride * idx]);
   return dest;
}

/** Copy vect_n elements of a vectray element to another vect_n
 
 
*/
BTINLINE vect_n * getvn_vr(vect_n *dest,vectray *ray, int idx)
{
   ray->rayvect->q = ray->data + ray->stride * idx;
   set_vn(dest,ray->rayvect);
   return dest;
}

/** Add a vect_n to the array of vect_n's
  This is the fastest way to copy data from a vector to
  an array.
 
 \retval 0 = Append was successful
         -1 = The array is full.
*/
BTINLINE int append_vr(vectray *ray, vect_n* v)
{

   if (ray->num_rows < ray->max_rows) {
      set_vn(idx_vr(ray,ray->num_rows),v);
      ray->num_rows++;
      return 0;
   } else {
      return -1;
   }
}

/** Increment vector array edit point
    see edit_point_vr()
*/
void next_vr(vectray *ray)
{
   if(ray->edit_point < ray->num_rows)
      ray->edit_point++;
}

/** Decrement vector array edit point
    see edit_point_vr()
*/
void prev_vr(vectray *ray)
{
   if(ray->edit_point > 0)
      ray->edit_point--;
}

/** Set edit point to beginning of array
    see edit_point_vr()
*/
void start_vr(vectray *ray)
{
   ray->edit_point = 0;
}

/** Set edit point to very end of a vector array.
 
  Note that this moves the edit point beyond the
  last row such that an insert_vr() will add a row to
  the end.
    see edit_point_vr()
*/
void end_vr(vectray *ray)
{
   ray->edit_point = ray->num_rows;
}

/** Set the edit point to the index provided
 see edit_point_vr()
 
 \retval 0 = set edit point was successful
         -1 = the provided index was out of bounds.
*/
int edit_at_vr(vectray *ray,int idx)
{
   if ((idx >= 0) && (idx <= ray->num_rows)) {
      ray->edit_point = idx;
      return 0;
   } else
      return -1;
}

/** Get the present edit point for a vector array.
 
The vectray object maintains an internal index for editing called
the edit point. \warning not yet threadsafe
 
Moving the edit point is accomplished using the following functions:
 - next_vr()
 - prev_vr()
 - start_vr()
 - end_vr()
 - edit_at_vr()
 
Editing at the edit point is accomplished with:
 - insert_vr()
 - delete_vr()
 
 
 
*/
int edit_point_vr(vectray *ray)
{
   return ray->edit_point;

}

/**
 Insert a point to the vectray if room is available.
 
 \todo The array does NOT automatically grow!
 
 blank spot is inserted
 num_rows is incremented
 edit_point is incremented
 \retval 0 = Insert was successful
         -1 = The array is full.
*/
BTINLINE int insert_vr(vectray *ray,vect_n *src)
{
   int cnt;
   if (ray->num_rows < ray->max_rows) {
      for(cnt = ray->num_rows;cnt > ray->edit_point;cnt--) {
         set_vn(lval_vr(ray,cnt),rval_vr(ray,cnt-1));
      }
      set_vn(idx_vr(ray,ray->edit_point),src);
      ray->num_rows++;
      ray->edit_point++;
      return 0;
   } else {
      return -1;
   }
}

/** Remove a vector from the vector array and copy all subsequent
vectors to fill the gap.
 
If the edit point is at or beyond the end of the array (one past the last row) then
the last row will be deleted.
 
It is safe to call delete_vr() on an empty array.
 
\retval 0 = Delete was successful
         1 = The array is empty.
*/
int delete_vr(vectray *ray)
{
   int cnt;
   for(cnt = ray->edit_point;cnt < ray->num_rows - 1;cnt++) {
      set_vn(lval_vr(ray,cnt),rval_vr(ray,cnt+1));
   }

   ray->num_rows--;
   if (ray->num_rows < 0)
      ray->num_rows = 0; //bound to zero

   //Move edit point if we deleted from end of array
   if (ray->edit_point > ray->num_rows)
      ray->edit_point = ray->num_rows;
   if (ray->num_rows > 0)
      return 0;
   else
      return 1;
}

/** Returns the index of the last element in ray */
BTINLINE int numrows_vr(vectray *ray)
{
   return ray->num_rows;
}

/** Returns the number of element in ray */
BTINLINE int numelements_vr(vectray *ray)
{
   return ray->n;
}

/** Returns the maximum number of elements in ray */
BTINLINE int maxrows_vr(vectray *ray)
{
   return (ray->max_rows );
}

/** Copy a subarray from one vectray to another
 
*/
void copy_sub_vr(vectray *dest, vectray *src, int src_r, int dest_r, int rows, int src_c, int dest_c, int columns)
{
   int i,j;
   btreal *dest_start,*src_start;

   dest_start = dest->data + dest->stride * dest_r + dest_c;
   src_start = src->data + src->stride * src_r + src_c;
   for (i = 0; i < rows;i++) {
      for (j = 0; j < columns;j++) {
         *dest_start = *src_start;
         dest_start++;
         src_start++;
      }
      dest_start += dest->stride - columns;
      src_start += src->stride - columns;
   }
}

vectray * resize_vr(vectray **vr,int max_rows)
{
   vectray *newvr,*tvr;
   tvr = *vr;
   newvr = new_vr(tvr->n,max_rows);
   newvr->num_rows = tvr->num_rows;
   newvr->edit_point = tvr->edit_point;
   copy_sub_vr(newvr,tvr,0,0,tvr->max_rows,0,0,tvr->n);
   destroy_vr(vr);
   return newvr;
}

/** Erases all the elements in ray */
BTINLINE void clear_vr(vectray *ray)
{
   ray->num_rows = 0;
}

/** Calculate the lenth of all the points if you
connected the dots.
 
Used by dist_adjust_vta()
*/
btreal arclength_vr(vectray *ray)
{
   btreal len = 0.0;
   int cnt;
   for(cnt = 1;cnt < ray->num_rows;cnt++) {
      len += norm_vn(sub_vn(lval_vr(ray,cnt-1),rval_vr(ray,cnt)));
   }
   return len;
}

/** Read a csv file and create a new vectray for the values.
*/
int read_csv_file_vr(char *fileName, vectray **vr)
{
   char        line[MAX_LINE_LENGTH],buff[250];
   int         charCt, i, row, columnCt;
   double      a[8];
   FILE        *inFile;
   int rows,columns;
   int mincols = 50000,maxcols = 0;
   char *linedummy;
   vect_n* tmp_v;

   //Open the file
   if((inFile = fopen(fileName, "r")) == NULL) {
      syslog(LOG_ERR, "Trajectory file '%s' not found!", fileName);
      return(1);
   }
   syslog(LOG_ERR, "Opened the trajectory file");

   //Scan the file
   rows = 0;
   while(1) {
      if(fgets(line, MAX_LINE_LENGTH, inFile) == NULL)
         break; //Reached EOF
      if(line[0] == '#') {
         syslog(LOG_ERR, "Found a comment: [%s]", line);
         continue; //Skip comments
      }
      linedummy = line;
      columns = strcount_vn(&linedummy," \n\r");

      if(columns > 1) {
         //syslog(LOG_ERR, "Read row %d with %d columns", rows, columns);
         if (mincols > columns)
            mincols = columns;
         if (maxcols < columns)
            maxcols = columns;
         columnCt = columns;
         ++rows;
      }
   }
   if (mincols != maxcols) {
      syslog(LOG_ERR, "Different rows have different columns. Min: %d Max: %d", mincols,maxcols);
      return(2);
   }
   //Allocate memory for the file
   *vr = new_vr(maxcols,rows);
   syslog(LOG_ERR, "New vr. Col: %d Row: %d", maxcols,rows);
   tmp_v = new_vn(maxcols); // BUG: This becomes a dangling pointer (leak)
   //Load the file into memory
   rewind(inFile);
   row = 0;
   while(1) {
      if(fgets(line, MAX_LINE_LENGTH, inFile) == NULL)
         break; //Reached EOF
      if(line[0] == '#') {
         syslog(LOG_ERR, "Found a comment: [%s]", line);
         continue; //Skip comments
      }

      fill_vn(tmp_v,0.0);
      csvto_vn(tmp_v,line);
      append_vr(*vr,tmp_v);
      //syslog(LOG_ERR, "Creating Vector: [%s]", sprint_vn(buff,idx_vr(*vr,row)));
      ++row;

   }
   syslog(LOG_ERR, "Loaded the data into the arrays");
   //Close the file
   fclose(inFile);

   columns = columnCt;

   return(0);
}

int write_csv_file_vr(char *filename, vectray *vr)
{
   int cnt;
   FILE        *outFile;
   char buff[250];

   //Open the file
   if((outFile = fopen(filename, "w")) == NULL) {
      syslog(LOG_ERR, "Trajectory file '%s' not found!", filename);
      return(1);
   }

   syslog(LOG_ERR, "write_csv_file_vr: %d",numrows_vr(vr));
   for (cnt = 0; cnt < numrows_vr(vr); cnt ++)
      fprintf(outFile,"%s\n",sprint_csv_vn(buff,idx_vr(vr,cnt)));

   fclose(outFile);
}

int test_vr(btreal error)
{
   vectray *vr,*vr2;
   int cnt,idx;

   vr = new_vr(3,10);
   for (cnt = 0; cnt < 10; cnt++) {
      fill_vn(idx_vr(vr,cnt),cnt);
      set_vn(idx_vr(vr,cnt),scale_vn(2.0,idx_vr(vr,cnt)));
      print_vn(idx_vr(vr,cnt));
   }

   read_csv_file_vr("test.csv",&vr2);
   for (cnt = 0; cnt < numrows_vr(vr2); cnt++) {
      print_vn(idx_vr(vr2,cnt));
   }
}

/**************************************** End vectray functions ****************/
/********************** vect_3 functions **************************************/


/** Optimized for 3 element vectors.  See fill_vn()
*/
BTINLINE void fill_v3(vect_3* dest, btreal val)
{
   dest->q[0] = val;
   dest->q[1] = val;
   dest->q[2] = val;
}

/** Optimized for 3 element vectors.  See new_vn()
*/
vect_3 * new_v3() //allocate an n-vector
{
   void* vmem;
   vect_3 *n;

   //allocate mem for vector,return vector, and return structure
   vmem = btmalloc(2*sizeof(vect_3));

   //addbtptr(vmem);
   n = (vect_3*)vmem;
   n->n = 3;
   n->ret = (vect_3*)(vmem + sizeof(vect_3));
   n->q = n->data;
   n->ret->n = 3;
   n->ret->q = n->ret->data;
   n->ret->ret = n->ret;
   fill_v3(n,0.0);

   return n;
}

vect_3 * init_staticv3(staticv3 *sv3)
{
   vect_3 *n;

   n = &sv3->main;
   n->n = 3;
   n->ret = &sv3->scratch;
   n->q = n->data;
   n->ret->n = 3;
   n->ret->q = n->ret->data;
   n->ret->ret = n->ret;
   fill_v3(n,0.0);

   return n;
}


/** See set_vn()
*/
BTINLINE vect_3* set_v3(vect_3* dest, vect_3* src)
{
   dest->q[0] = src->q[0];
   dest->q[1] = src->q[1];
   dest->q[2] = src->q[2];
   return dest;
}

/** Optimized for 3 element vectors. See const_vn()
*/
BTINLINE vect_3* const_v3(vect_3* dest, btreal v1, btreal v2, btreal v3)
{
   dest->q[0] = v1;
   dest->q[1] = v2;
   dest->q[2] = v3;
   return dest;
}

/**
  returns a pointer to a vect_3 filled by constant values
  \warning This is NOT theadsafe. Maybe allocate memory each time
*/
BTINLINE vect_3* C_v3(btreal v1, btreal v2, btreal v3)
{
   static vect_3 scratch,scratch2;
   scratch.q = scratch.data;
   scratch2.q = scratch2.data;
   scratch.ret = &scratch2;
   scratch2.ret = &scratch2;
   scratch.q[0] = v1;
   scratch.q[1] = v2;
   scratch.q[2] = v3;
   return &scratch;
}

/** Optimized for 3 element vectors. See setval_vn()
*/
BTINLINE void setval_v3(vect_3* dest, int idx, btreal val)
{
   dest->q[idx] = val;
}

/** Optimized for 3 element vectors. See getval_vn()
*/
BTINLINE btreal getval_v3(vect_3* dest, int idx)
{
   return dest->q[idx];
}

/** Optimized for 3 element vectors. See neg_vn()
*/
BTINLINE vect_3* neg_v3(vect_3* a)
{
   a->ret->q[0] = -1.0 * a->q[0];
   a->ret->q[1] = -1.0 * a->q[1];
   a->ret->q[2] = -1.0 * a->q[2];
   return a->ret;
}

/** Optimized for 3 element vectors. See scale_vn()
*/
BTINLINE vect_3* scale_v3(btreal x, vect_3* a)
{
   a->ret->q[0] = x * a->q[0];
   a->ret->q[1] = x * a->q[1];
   a->ret->q[2] = x * a->q[2];
   return a->ret;
}

/** Optimized for 3 element vectors. See add_vn()
*/
BTINLINE vect_3* add_v3(vect_3* a, vect_3* b)
{
   a->ret->q[0] = a->q[0] + b->q[0];
   a->ret->q[1] = a->q[1] + b->q[1];
   a->ret->q[2] = a->q[2] + b->q[2];
   return a->ret;
}

/** Optimized for 3 element vectors. See sub_vn()
*/
BTINLINE vect_3* sub_v3(vect_3* a, vect_3* b)
{
   a->ret->q[0] = a->q[0] - b->q[0];
   a->ret->q[1] = a->q[1] - b->q[1];
   a->ret->q[2] = a->q[2] - b->q[2];
   return a->ret;
}

/** Optimized for 3 element vectors. See cross_vn()
*/
BTINLINE vect_3* cross_v3(vect_3* a, vect_3*b)
{
   btreal q[3];
   q[0] = a->q[1] * b->q[2] - a->q[2] * b->q[1];
   q[1] =  a->q[2] * b->q[0] - a->q[0] * b->q[2];
   q[2] = a->q[0] * b->q[1] - a->q[1] * b->q[0];
   a->ret->q[0] = q[0];
   a->ret->q[1] = q[1];
   a->ret->q[2] = q[2];
   return a->ret;
}

/** Optimized for 3 element vectors. See dot_vn()
*/
BTINLINE btreal  dot_v3(vect_3* a, vect_3* b)
{
   return (a->q[0]*b->q[0] + a->q[1]*b->q[1] + a->q[2]*b->q[2]);
}

/** Optimized for 3 element vectors. See norm_vn()
*/
BTINLINE btreal  norm_v3(vect_3* a)
{
   return sqrt(a->q[0]*a->q[0] + a->q[1]*a->q[1] + a->q[2]*a->q[2]);
}

/** Optimized for 3 element vectors. See unit_vn()
*/
BTINLINE vect_3* unit_v3(vect_3* a)
{
   btreal div;
   div = sqrt(a->q[0]*a->q[0] + a->q[1]*a->q[1] + a->q[2]*a->q[2]);
   if (div > PRACTICALLY_ZERO) {
      a->ret->q[0] = a->q[0]/div;
      a->ret->q[1] = a->q[1]/div;
      a->ret->q[2] = a->q[2]/div;
   } else {
      a->ret->q[0] = 0.0;
      a->ret->q[1] = 0.0;
      a->ret->q[2] = 0.0;
   }
   return a->ret;
}

/** Optimized for 3 element vectors. See print_vn()
*/
void print_v3(vect_3* src)
{
   int i,j;

   printf("<");
   for(j = 0;j<src->n;j++)
      printf("%g,",src->q[j]);
   printf(">");

}

/** Optimized for 3 element vectors. See sprint_vn()
*/
char* sprint_v3(char *dest,vect_3* src)
{
   int i,j;
   if (src != NULL) {
      dest[0] = '<';
      dest[1] = 0;
      for(j = 0;j<src->n;j++)
         sprintf(dest+strlen(dest)," %8.4f,",src->q[j]);
      dest[strlen(dest)-1] = 0;
      strcat(dest," >");
   }
}

/** \todo unfinished
*/
int test_v3(btreal error)
{
   vect_3 *c1,*c2,*c3;
   vect_3 *b1,*b2;
   btreal a1,a2,a3;

   c1 = new_v3();
   c2 = new_v3();
   c3 = new_v3();


   printf("\nThe following should be <5,4,3,2>");
   const_v3(c1,5.0,4.0,3.0);
   printf("\n<%g,%g,%g,%g>\n\n",getval_v3(c1,0),getval_v3(c1,1),getval_v3(c1,2));

   printf("\nSet: <5,4,3,2>");
   set_v3(c2,c1);
   print_v3(c2);

   printf("\nSub,Add: <5,4,3,2>");
   set_v3(c3,add_v3(c1,sub_v3(c1,c2)));
   print_v3(c3);

   printf("\nNeg,Scale: <-15,-12,-9,-6>");
   set_v3(c3,scale_v3(3,neg_v3(c1)));
   print_v3(c3);

   printf("\nUnit: <0,0,0,0>");
   set_v3(c3,unit_v3(c1));
   print_v3(c3);

   printf("\nDot: %g | %g",5.0*5.0+4.0*4.0+3.0*3.0+2.0*2.0,dot_v3(c1,c2));
   printf("\nNorm: %g | %g",sqrt(5*5+4*4+3*3+2*2),norm_v3(c1));

   printf("\nSum abuse: 5*<5,4,3,2>");
   set_v3(c3,add_v3(c1,add_v3(c1,add_v3(c1,add_v3(c1,c1)))));
   print_v3(c3);

   printf("\nCross abuse: 5*<5,4,3,2>");
   set_v3(c3,cross_v3(c1,scale_v3(getval_v3(c2,2),c2)));
   print_v3(c3);

   const_v3(c1,0.1,-0.4,5.0);
   const_v3(c2,0.8,3.2,-1.4);
   printf("\nCross <0.1,-0.4,5.0> X <0.8,3.2,-1.4> = <-15.44,4.14,.64>:");
   print_v3(cross_v3(c1,c2));

}

/**************************************** End vect_3 functions ****************/
/********************** quat functions **************************************/

BTINLINE void fill_q(quat* dest, btreal val)
{
   dest->q[0] = val;
   dest->q[1] = val;
   dest->q[2] = val;
   dest->q[3] = val;
}

quat * new_q() //allocate an n-vector
{
   void* vmem;
   quat *n;
   //allocate mem for vector,return vector, and return structure
   vmem = btmalloc(2*sizeof(quat));

   //addbtptr(vmem);
   n = (quat*)vmem;
   n->n = 4;
   n->ret = (quat*)(vmem + sizeof(quat));
   n->q = n->data;
   n->ret->n = 4;
   n->ret->q = n->ret->data;
   n->ret->ret = n->ret;
   fill_q(n,0.0);

   return n;
}

BTINLINE quat* set_q(quat* dest, quat* src)
{
   dest->q[0] = src->q[0];
   dest->q[1] = src->q[1];
   dest->q[2] = src->q[2];
   dest->q[3] = src->q[3];
   return dest;
}

BTINLINE quat* const_q(quat* dest, btreal v1, btreal v2, btreal v3, btreal v4)
{
   dest->q[0] = v1;
   dest->q[1] = v2;
   dest->q[2] = v3;
   dest->q[3] = v4;
   return dest;
}

/**
  returns a pointer to a quat filled by constant values
  \warning This is NOT theadsafe. Maybe allocate memory each time
*/
BTINLINE quat* C_q(btreal v1, btreal v2, btreal v3, btreal v4)
{
   static quat scratch,scratch2;
   scratch.q = scratch.data;
   scratch2.q = scratch2.data;
   scratch.ret = &scratch2;
   scratch2.ret = &scratch2;
   scratch.q[0] = v1;
   scratch.q[1] = v2;
   scratch.q[2] = v3;
   scratch.q[3] = v3;
   return &scratch;
}

BTINLINE void setval_q(quat* dest, int idx, btreal val)
{
   dest->q[idx] = val;
}

BTINLINE btreal getval_q(quat* dest, int idx)
{
   return dest->q[idx];
}

BTINLINE void extract_q(btreal* dest, quat* src)
{
   //copy vector to a btreal array
   int cnt;

   //th Add vector size checking code here
   for (cnt=0;cnt < src->n;cnt++)
      dest[cnt] = src->q[cnt];

}

BTINLINE void inject_q(quat* dest, btreal* src) //copy btreal array to vector
{
   int cnt;

   //th Add vector size checking code here
   for (cnt=0;cnt < dest->n;cnt++)
      dest->q[cnt] = src[cnt];
}

BTINLINE quat* conj_q(quat* a)
{
   a->ret->q[0] = a->q[0];
   a->ret->q[1] = -1.0 * a->q[1];
   a->ret->q[2] = -1.0 * a->q[2];
   a->ret->q[3] = -1.0 * a->q[3];
   return a->ret;
}

BTINLINE quat* add_q(quat* a, quat* b)
{
   a->ret->q[0] = a->q[0] + b->q[0];
   a->ret->q[1] = a->q[1] + b->q[1];
   a->ret->q[2] = a->q[2] + b->q[2];
   a->ret->q[3] = a->q[3] + b->q[3];
   return a->ret;
}

BTINLINE quat* sub_q(quat* a, quat* b)
{
   a->ret->q[0] = a->q[0] - b->q[0];
   a->ret->q[1] = a->q[1] - b->q[1];
   a->ret->q[2] = a->q[2] - b->q[2];
   a->ret->q[3] = a->q[3] - b->q[3];
   return a->ret;
}

BTINLINE quat* scale_q(btreal x, quat* a)
{
   a->ret->q[0] = x * a->q[0];
   a->ret->q[1] = x * a->q[1];
   a->ret->q[2] = x * a->q[2];
   a->ret->q[3] = x * a->q[3];
   return a->ret;
}

BTINLINE btreal  dot_q(quat* a, quat* b)
{
   return (a->q[0]*b->q[0] + a->q[1]*b->q[1] + a->q[2]*b->q[2]+ a->q[3]*b->q[3]);
}

BTINLINE btreal  norm_q(quat* a)
{
   return sqrt(a->q[0]*a->q[0] + a->q[1]*a->q[1] + a->q[2]*a->q[2] + a->q[3]*a->q[3]);
}

BTINLINE quat* unit_q(quat* a)
{
   btreal div;
   div = norm_q(a);
   if (div > PRACTICALLY_ZERO) {
      a->ret->q[0] = a->q[0]/div;
      a->ret->q[1] = a->q[1]/div;
      a->ret->q[2] = a->q[2]/div;
      a->ret->q[3] = a->q[3]/div;
   } else {
      a->ret->q[0] = 0.0;
      a->ret->q[1] = 0.0;
      a->ret->q[2] = 0.0;
      a->ret->q[3] = 0.0;
   }
   return a->ret;
}

quat* inv_q(quat* a)
{
   btreal div;
   div = norm_q(a);
   if (div > PRACTICALLY_ZERO) {
      a->ret->q[0] = a->q[0]/div;
      a->ret->q[1] = -1.0*a->q[1]/div;
      a->ret->q[2] = -1.0*-a->q[2]/div;
      a->ret->q[3] = -1.0*-a->q[3]/div;
   } else {
      a->ret->q[0] = 0.0;
      a->ret->q[1] = 0.0;
      a->ret->q[2] = 0.0;
      a->ret->q[3] = 0.0;
   }
   return a->ret;

}

quat* pow_q(quat* a, btreal b)
{
   return exp_q(scale_q(b,log_q(a)));

}

BTINLINE quat* mul_q(quat* a, quat*b)
{
   btreal q[4];

   q[0] = a->q[0] * b->q[0] - (a->q[1]*b->q[1] + a->q[2]*b->q[2]+ a->q[3]*b->q[3]);
   q[1] = a->q[0] * b->q[1] + b->q[0] * a->q[1] + a->q[2] * b->q[3] - a->q[3] * b->q[2];
   q[2] = a->q[0] * b->q[2] + b->q[0] * a->q[2] + a->q[3] * b->q[1] - a->q[1] * b->q[3];
   q[3] = a->q[0] * b->q[3] + b->q[0] * a->q[3] + a->q[1] * b->q[2] - a->q[2] * b->q[1];

   a->ret->q[0] = q[0];
   a->ret->q[1] = q[1];
   a->ret->q[2] = q[2];
   a->ret->q[3] = q[3];
   return a->ret;
}

quat* exp_q(quat* a)
{
   btreal theta,st;

   theta = sqrt(a->q[1]*a->q[1] + a->q[2]*a->q[2]+ a->q[3]*a->q[3]);
   st = sin(theta);

   a->ret->q[0] = cos(theta);

   if (fabs(st) > PRACTICALLY_ZERO) {
      a->ret->q[1] = a->q[1]*st/theta;
      a->ret->q[2] = a->q[2]*st/theta;
      a->ret->q[3] = a->q[3]*st/theta;
   } else {
      a->ret->q[1] = a->q[1];
      a->ret->q[2] = a->q[2];
      a->ret->q[3] = a->q[3];

   }

   return a->ret;
}

quat* log_q(quat* a)
{
   btreal theta,st;

   theta = acos(a->q[0]);
   st = sin(theta);

   a->ret->q[0] = 0.0;

   if (fabs(st) > PRACTICALLY_ZERO) {
      a->ret->q[1] = a->q[1]/st*theta;
      a->ret->q[2] = a->q[2]/st*theta;
      a->ret->q[3] = a->q[3]/st*theta;
   } else {
      a->ret->q[1] = a->q[1];
      a->ret->q[2] = a->q[2];
      a->ret->q[3] = a->q[3];

   }

   return a->ret;
}

/** Convert Rotation matrix to a quaternion
 
*/
quat* R_to_q(quat* dest, matr_3* src)
{
   dest->q[0] = 0.5*sqrt(fabs(src->q[0] + src->q[5] + src->q[10] + 1));
   dest->q[1] = (src->q[9] - src->q[6])*0.25/dest->q[0];
   dest->q[2] = (src->q[2] - src->q[8])*0.25/dest->q[0];
   dest->q[3] = (src->q[4] - src->q[1])*0.25/dest->q[0];
   return dest;
}

/** Convert quaternion to rotation matrix
*/
matr_3* q_to_R(matr_3* dest, quat* src)
{
   btreal q00 = src->q[0]*src->q[0];
   btreal q01 = src->q[0]*src->q[1];
   btreal q02 = src->q[0]*src->q[2];
   btreal q03 = src->q[0]*src->q[3];
   btreal q11 = src->q[1]*src->q[1];
   btreal q12 = src->q[1]*src->q[2];
   btreal q13 = src->q[1]*src->q[3];
   btreal q22 = src->q[2]*src->q[2];
   btreal q23 = src->q[2]*src->q[3];
   btreal q33 = src->q[3]*src->q[3];
   
   setrow_m3(dest,0,
             q00 + q11 - q22 - q33,
             2.0*(q12 - q03),
             2.0*(q13 + q02)); // BZ: Changed to +

   setrow_m3(dest,1,
             2.0*(q12 + q03),
             q00 - q11 + q22 - q33,
             2.0*(q23 - q01));

   setrow_m3(dest,2,
             2.0*(q13 - q02),
             2.0*(q23 + q01),
             q00 - q11 - q22 + q33);
   /*
   setrow_m3(dest,0,
             src->q[0]*src->q[0] + src->q[1]*src->q[1] - src->q[2]*src->q[2] - src->q[3]*src->q[3],
             2.0*(src->q[1]*src->q[2] - src->q[0]*src->q[3]),
             2.0*(src->q[1]*src->q[3] + src->q[0]*src->q[2])); // BZ: Changed to +

   setrow_m3(dest,1,
             2.0*(src->q[1]*src->q[2] + src->q[0]*src->q[3]),
             src->q[0]*src->q[0] - src->q[1]*src->q[1] + src->q[2]*src->q[2] - src->q[3]*src->q[3],
             2.0*(src->q[2]*src->q[3] - src->q[0]*src->q[1]));

   setrow_m3(dest,2,
             2.0*(src->q[1]*src->q[3] - src->q[0]*src->q[2]),
             2.0*(src->q[2]*src->q[3] + src->q[0]*src->q[1]),
             src->q[0]*src->q[0] - src->q[1]*src->q[1] - src->q[2]*src->q[2] + src->q[3]*src->q[3]);
   */
   return dest;
}

/** Great circle distance between two quaternions
Returns the great circle distance between two unit quaternions. If the
distance is greater than pi it will return the dual dist/vector that is
less than pi
*/
btreal GCdist_q(quat* start, quat* end)
{
   btreal q[4];
   //  q[0] = a->q[0] * b->q[0] + a->q[1]*b->q[1] + a->q[2]*b->q[2] + a->q[3]*b->q[3]; //optimized but untested
   //  return acos(q[0])*2.0;                                                          //optimized but untested
   return angle_q(force_closest_q(mul_q(end,conj_q(start)))); // inv_q = conj_q for quaternions of unit length
}

/** Great circle distance between two quaternions
Returns the great circle distance between two unit quaternions.
*/
vect_3* GCaxis_q(vect_3* dest, quat* start, quat* end)
{
   btreal q[4];
   // inv_q = conj_q for quaternions of unit length
   return axis_q(dest,force_closest_q(mul_q(end,conj_q(start))));
}

quat * force_closest_q(quat* src)
{
   //const btreal pi = 3.14159265359;
   btreal flip = 1.0;

   src->ret->q[0] = src->q[0];

   if (src->q[0] < 0.0) {
      flip = -1.0;
      src->ret->q[0] = cos(pi - acos(src->q[0]));
   }

   src->ret->q[1] = flip * src->q[1];
   src->ret->q[2] = flip * src->q[2];
   src->ret->q[3] = flip * src->q[3];

   return src->ret;
}

/** Return the angle of rotation represented by a unit quaternion.
\param src A unit quaternion
\bug We constrian q[0] to be positive
*/
btreal angle_q(quat* src)
{
   if (src->q[0] > 1.0)
      src->q[0] = 1.0;
   if (src->q[0] < -1.0)
      src->q[0] = -1.0;
   return acos(src->q[0])*2.0;
}

/** Set vector dest to the axis of rotation defined by quaternion src
\param src A quaternion
Givin a unit quaternion q (used to rotate vector v) it can be decomposed
into subcomponents (a, b*u) where a and b are scalars and u is a unit vector.
u is the axis of the rotation given by q and theta is the angle of rotation.
a = cos(theta/2) ; b = sin(theta/2) 
 
\bug We insure that the axis is given relative to a positive angle 0 - pi
*/
vect_3* axis_q(vect_3* dest, quat* src)
{
   dest->q[0] = src->q[1];
   dest->q[1] = src->q[2];
   dest->q[2] = src->q[3];

   set_v3(dest,unit_v3(dest));
   return dest;
}

/********************** quat functions **************************************/

matr_mn * new_mn(int r,int c) //allocate an n-vector
{
   void *ptr;
   matr_mn *n;
   //allocate mem for return structure
   ptr = btmalloc(2*sizeof(matr_mn) + 2*r*c*sizeof(btreal));

   n = (matr_mn*)ptr;
   n->ret = (matr_mn*)(ptr + sizeof(matr_mn));
   n->ret->ret = n->ret;
   n->q = (btreal*)(ptr + 2*sizeof(matr_mn));
   n->ret->q = (btreal*)(ptr + 2*sizeof(matr_mn) + r*c*sizeof(btreal));
   n->m = r;
   n->n = c;
   n->ret->m = r;
   n->ret->n = c;
   n->s = r * c;
   n->ret->s = r * c;
   
   ident_mn(n);

   return n;
}

matr_mn * new_mn_ptr(matr_mn *a, int r, int c, int offset)
{
   void *ptr;
   matr_mn *n;
   //allocate mem for return structure
   ptr = btmalloc(2*sizeof(matr_mn));

   n = (matr_mn*)ptr;
   n->ret = (matr_mn*)(ptr + sizeof(matr_mn));
   n->ret->ret = n->ret;
   n->q = a->q + offset;
   n->ret->q = a->ret->q + offset;
   n->m = r;
   n->n = c;
   n->ret->m = r;
   n->ret->n = c;
   
   return n;
}

void destroy_mn(matr_mn **src)
{
   btfree((void**)src);
}

BTINLINE void set_mn(matr_mn* dest, matr_mn* src)
{
   int i,j,imax,jmax;
   unsigned int ds,ss;

   imax = (dest->m < src->m)?dest->m:src->m;
   jmax = (dest->n < src->n)?dest->n:src->n;
   ds = dest->n; //destination stride
   ss = src->n; //source stride
   for (i = 0; i < imax; i++)
      for (j = 0; j < jmax; j++)
         dest->q[i*ds + j] = src->q[i*ss + j];
}

matr_mn* ident_mn(matr_mn* dest)
{
   int i,j,imax,jmax;
   unsigned int ds;

   imax = dest->m;
   jmax = dest->n;

   ds = dest->n; //destination stride
   for (i = 0; i < imax; i++)
      for (j = 0; j < jmax; j++)
         if(i == j)
            dest->q[i*ds + j] = 1.0;
         else
            dest->q[i*ds + j] = 0.0;

   return(dest);
}

void zero_mn(matr_mn* dest)
{
   int i, elements;
   
   elements = dest->m * dest->n;

   for (i = 0; i < elements; i++)
            dest->q[i] = 0.0;
}

void setrow_mn(matr_mn* dest, vect_n* src,int row)
{
   int i,j,imax,jmax;
   unsigned int ds,ss;


   if (row <= dest->m || row > 0)
      imax = row;
   else
      return;

   jmax = (dest->n < src->n)?dest->n:src->n;
   ds = dest->n; //destination stride

   for (j = 0; j < jmax; j++)
      dest->q[imax*ds + j] = src->q[j];
}

void setcol_mn(matr_mn* dest, vect_n* src,int col)
{
   int i,j,imax,jmax;
   unsigned int ds,ss;


   if (col <= dest->n || col > 0)
      jmax = col;
   else
      return;

   imax = (dest->m < src->n)?dest->m:src->n;
   ds = dest->n; //destination stride

   for (i = 0; i < imax; i++)
      dest->q[i*ds + jmax] = src->q[i];
}

void getcol_mn(vect_n* dest, matr_mn* src, int col)
{
   int i,j,imax,jmax;
   unsigned int ds,ss;


   if (col <= src->n || col > 0)
      jmax = col;
   else
      return;

   imax = (src->m < dest->n)?src->m:dest->n;
   ds = src->n; //destination stride

   for (i = 0; i < imax; i++)
      dest->q[i] = src->q[i*ds + jmax];
}

btreal getval_mn(matr_mn* src, int row, int col)
{
   return src->q[src->n * row + col];
}

void setval_mn(matr_mn* src, int row, int col, btreal val)
{
   src->q[src->n * row + col] = val;

}
BTINLINE matr_mn* scale_mn(btreal x, matr_mn* a)
{
   int i, elements;
   
   elements = a->m * a->n;
   
   for(i = 0; i < elements; i++)
      a->ret->q[i] = a->q[i] * x;
   
   return(a->ret);
}

/** Multiply two matrices of type matr_mn
   \note Result matrix "r" must not point to the same location as input matrix "a"
   */
BTINLINE matr_mn* mul_mn(matr_mn* r, matr_mn* a, matr_mn* b)
{
   unsigned int i, j, k, aCols, bCols;
   
   if((r->m * r->n) != (a->m * b->n)){
      syslog(LOG_ERR, "mul_mn(): incorect size for return matrix: a=%dx%d, b=%dx%d, r=%dx%d, need r=%dx%d", 
         a->m, a->n, b->m, b->n, r->m, r->n, a->m, b->n);
      return r;
   }
   
   aCols = a->n;
   bCols = b->n;

   r->m = a->m;
   r->n = b->n;  
   
   zero_mn(r);
   
   // Rij += Aik + Bkj
   for(i = 0; i < a->m; i++)
      for(j = 0; j < bCols; j++)
         for(k = 0; k < aCols; k++)
            ELEM(r,i,j) += ELEM(a,i,k) * ELEM(b,k,j);
            //r->q[i*bCols+j] += a->q[i*aCols+k] + b->q[k*bCols+j];
   
   return r;
}

BTINLINE matr_mn* T_mn(matr_mn* a){
   unsigned int i, j;
   
   a->ret->m = a->n;
   a->ret->n = a->m;
   
   for(i = 0; i < a->n; i++)
      for(j = 0; j < a->m; j++)
         ELEM(a->ret,i,j) = ELEM(a,j,i);
         //setval_mn(a->ret, i, j, getval_mn(a, j, i));
         //a->ret->q[i*a->m+j] = a->q[j*a->n+i];
   
   return a->ret;
}

BTINLINE matr_mn* add_mn(matr_mn* r, matr_mn* a, matr_mn* b){
   unsigned int i, elements;
   
   if((r->m != a->m) || (r->n != a->n))
      syslog(LOG_ERR, "add_mn: Matrices not same size. r=%dx%d, a=%dx%d", r->m, r->n, a->m, a->n);

   r->m = a->m;
   r->n = a->n;
   
   elements = a->m * a->n;
   //syslog(LOG_ERR, "r->q[0] = %f", r->q[0]);
   for(i = 0; i < elements; i++)
         r->q[i] = a->q[i] + b->q[i];
   
   return r;
}

/** Multiply a matrix by a vector.
 
b must have at least a->n elements.
ret must have at least a->m elements
 
*/
vect_n* matXvec_mn(matr_mn* a, vect_n* b,vect_n* ret)
{
   int i,j,imax,jmax;
   unsigned int ds;

   imax = a->m;
   jmax = a->n;

   for (i = 0; i < imax; i++) {
      ret->q[i] = 0.0;
      ds = a->n * i;
      for (j = 0; j < jmax; j++)
         ret->q[i] += a->q[ds + j] * b->q[j];
   }
   return ret;


}

/** Print a matrix to a string
Uses barretts <<>> delimited format.
*/
char* sprint_mn(char *dest,matr_mn* src)
{
   int i,j;

   if (btptr_ok(src,"sprint_vn")) {
      dest[0] = '<';
      dest[1] = 0;
      for (i = 0;i<src->m;i++) {
         sprintf(dest+strlen(dest),"<");
         for(j = 0;j<src->n;j++) {
            sprintf(dest+strlen(dest),"%8.4f,",src->q[i*src->n+j]);
         }
         sprintf(dest+strlen(dest),">,\n");
      }
      dest[strlen(dest)-1] = 0;
      strcat(dest,">");
   }
   return dest;
}

/** Print a matrix to a string
Uses barretts <<>> delimited format.
*/
char* sprint_mh(char *dest,matr_h* src)
{
   int i,j;

   if (btptr_ok(src,"sprint_vn")) {
      dest[0] = '<';
      dest[1] = 0;
      for (i = 0;i<src->m;i++) {
         sprintf(dest+strlen(dest),"<");
         for(j = 0;j<src->n;j++) {
            sprintf(dest+strlen(dest),"%8.4f,",src->q[i*src->n+j]);
         }
         sprintf(dest+strlen(dest),">,");
      }
      dest[strlen(dest)-1] = 0;
      strcat(dest,">");
   }
   return dest;
}

/** Fill an existing matrix with values from a string.
 
This function takes a string that has values in the format of:
\code
"< < 123, 321>, <123>,  <234, 324, 344>>"
\endcode
to represent:
\code
123 321 (0)
123 (0) (0)
234 324 344
\endcode
 
If the string is invalid, nothing is done.
 
Behaves nicely if the string matrix is a different size than the destination 
matrix. 
*/
void strto_mn(matr_mn* dest,char *str)
{
   regex_t tra[3];
   int row = 0;
   char *sub;
   char str1[400];
   char str2[400];
   unsigned int len;
   regmatch_t matches[50];
   vect_n *vect;
   local_vn(tmpvn,100);

   init_vn(tmpvn,100);
   regcomp(&tra[0],"<[[:space:]]*(<[[:digit:][:space:],.+eE-]+>[[:space:],]*)+[[:space:]]*>[[:space:]]*",REG_EXTENDED );
   regcomp(&tra[1],"<[[:space:]]*<.*>[[:space:]]*>[[:space:]]*",REG_EXTENDED );
   regcomp(&tra[2],"<[[:digit:][:space:],.+eE-]+>",REG_EXTENDED );
   //printf("%s ...\n",str);
   if (!regexec(&tra[0],str,0,NULL,0) && !regexec(&tra[1],str,0,NULL,0)) {
      sub = str;
      while (!regexec(&tra[2],sub,3,matches,0)) {
         if (row > dest->m)
            break;

         len = matches[0].rm_eo-matches[0].rm_so;
         strncpy(str1,sub + matches[0].rm_so,len+1);
         str1[len] = 0;
         sub = sub + matches[0].rm_eo;
         vect = sscan_vn(str1);
         setrow_mn(dest,vect,row);
         //print_vn(vect);

         destroy_vn(&vect);
         row++;
         //printf("\n:%s\n",str1);
      }
   }

}

/********************** matr_h functions **************************************/


/** Allocates memory and calculation swap space for a new homogeneous matrix.
 
  All matrices need to be initialized with new. It is preferable to declare 
  matrices as pointers. If you declare a non-pointer matrix then the memory set 
  aside for it will be ignored and new memory will be allocated.
  
  Not initializing a matrix with new_mh() will result in a seg_fault.
  
  ex:
  \code
  //YES
  matr_h *a,*b;  
  new_mh(a); new_mh(b); //allocate matrix and calculation swap memory
  set_mh(a,b);
  //NO
  matr_h a,b;
  set_mh(&a,&b);
  //NO
  matr_h a,b;
  new_mh(&a); new_mh(&b); 
  set_mh(&a,&b);
  \endcode
  
*/
/* I allocate an extra chunk of memory with each matrix so that I have a place to
return calculation results without dirtying up the original matrix. This could
probably be a single piece of memory somewhere but I am worried about thread 
safety. th050222
 
*/
matr_h * new_mh()
{
   //allocate an n-vector
   void *ptr;
   matr_h *n;

   //allocate mem for return structure
   ptr = btmalloc(2*sizeof(matr_h));

   //addbtptr(ptr);
   n = (matr_h*)ptr;
   n->ret = (matr_h*)(ptr + sizeof(matr_h));
   n->ret->ret = n->ret;
   n->q = n->data;
   n->ret->q = n->ret->data;
   n->m = 4;
   n->n = 4;
   n->s = 16;
   n->ret->m = 4;
   n->ret->n = 4;
   n->ret->s = 16;
   
   ident_mh(n);
   
   return n;
}

/** Copy src matrix to dest matrix
*/
BTINLINE void set_mh(matr_h* dest, matr_h* src)
{
   dest->q[0] = src->q[0];
   dest->q[1] = src->q[1];
   dest->q[2] = src->q[2];
   dest->q[3] = src->q[3];
   dest->q[4] = src->q[4];
   dest->q[5] = src->q[5];
   dest->q[6] = src->q[6];
   dest->q[7] = src->q[7];
   dest->q[8] = src->q[8];
   dest->q[9] = src->q[9];
   dest->q[10] = src->q[10];
   dest->q[11] = src->q[11];

}

BTINLINE void setrow_mh(matr_h* dest,int row, btreal s1, btreal s2, btreal s3, btreal s4)
{
   btreal *q;
   q = dest->q + 4*row;
   q[0] = s1;
   q[1] = s2;
   q[2] = s3;
   q[3] = s4;

}

BTINLINE void getcol_mh(vect_3* dest, matr_h* src, int n)
{
   btreal *q;
   q = src->q + n;
   dest->q[0] = *q;
   q += 4;
   dest->q[1] = *q;
   q += 4;
   dest->q[2] = *q;
}

BTINLINE void ident_mh(matr_h* dest)
{
   dest->q[0] = 1.0;
   dest->q[1] = 0.0;
   dest->q[2] = 0.0;
   dest->q[3] = 0.0;
   dest->q[4] = 0.0;
   dest->q[5] = 1.0;
   dest->q[6] = 0.0;
   dest->q[7] = 0.0;
   dest->q[8] = 0.0;
   dest->q[9] = 0.0;
   dest->q[10] = 1.0;
   dest->q[11] = 0.0;
   dest->q[12] = 0.0;
   dest->q[13] = 0.0;
   dest->q[14] = 0.0;
   dest->q[15] = 1.0;
}

BTINLINE matr_h* mul_mh(matr_h* a,matr_h* b)
{
   btreal ret[12];
   //col 1
   ret[0] = a->q[0]*b->q[0] + a->q[1]*b->q[4] + a->q[2]*b->q[8];
   ret[4] = a->q[4]*b->q[0] + a->q[5]*b->q[4] + a->q[6]*b->q[8];
   ret[8] = a->q[8]*b->q[0] + a->q[9]*b->q[4] + a->q[10]*b->q[8];

   //col 2
   ret[1] = a->q[0]*b->q[1] + a->q[1]*b->q[5] + a->q[2]*b->q[9];
   ret[5] = a->q[4]*b->q[1] + a->q[5]*b->q[5] + a->q[6]*b->q[9];
   ret[9] = a->q[8]*b->q[1] + a->q[9]*b->q[5] + a->q[10]*b->q[9];

   //col 3
   ret[2] = a->q[0]*b->q[2] + a->q[1]*b->q[6] + a->q[2]*b->q[10];
   ret[6] = a->q[4]*b->q[2] + a->q[5]*b->q[6] + a->q[6]*b->q[10];
   ret[10] = a->q[8]*b->q[2] + a->q[9]*b->q[6] + a->q[10]*b->q[10];

   //col 4
   ret[3] = a->q[0]*b->q[3] + a->q[1]*b->q[7] + a->q[2]*b->q[11] + a->q[3];
   ret[7] = a->q[4]*b->q[3] + a->q[5]*b->q[7] + a->q[6]*b->q[11] + a->q[7];
   ret[11] = a->q[8]*b->q[3] + a->q[9]*b->q[7] + a->q[10]*b->q[11] + a->q[11];

   a->ret->q[0] = ret[0];
   a->ret->q[1] = ret[1];
   a->ret->q[2] = ret[2];
   a->ret->q[3] = ret[3];
   a->ret->q[4] = ret[4];
   a->ret->q[5] = ret[5];
   a->ret->q[6] = ret[6];
   a->ret->q[7] = ret[7];
   a->ret->q[8] = ret[8];
   a->ret->q[9] = ret[9];
   a->ret->q[10] = ret[10];
   a->ret->q[11] = ret[11];

   return a->ret;
}

BTINLINE void setmul_mh(matr_h* a,matr_h* b)
{
   btreal ret[12];
   //col 1
   ret[0] = a->q[0]*b->q[0] + a->q[1]*b->q[4] + a->q[2]*b->q[8];
   ret[4] = a->q[4]*b->q[0] + a->q[5]*b->q[4] + a->q[6]*b->q[8];
   ret[8] = a->q[8]*b->q[0] + a->q[9]*b->q[4] + a->q[10]*b->q[8];

   //col 2
   ret[1] = a->q[0]*b->q[1] + a->q[1]*b->q[5] + a->q[2]*b->q[9];
   ret[5] = a->q[4]*b->q[1] + a->q[5]*b->q[5] + a->q[6]*b->q[9];
   ret[9] = a->q[8]*b->q[1] + a->q[9]*b->q[5] + a->q[10]*b->q[9];

   //col 3
   ret[2] = a->q[0]*b->q[2] + a->q[1]*b->q[6] + a->q[2]*b->q[10];
   ret[6] = a->q[4]*b->q[2] + a->q[5]*b->q[6] + a->q[6]*b->q[10];
   ret[10] = a->q[8]*b->q[2] + a->q[9]*b->q[6] + a->q[10]*b->q[10];

   //col 1
   ret[3] = a->q[0]*b->q[3] + a->q[1]*b->q[7] + a->q[2]*b->q[11] + a->q[3];
   ret[7] = a->q[4]*b->q[3] + a->q[5]*b->q[7] + a->q[6]*b->q[11] + a->q[7];
   ret[11] = a->q[8]*b->q[3] + a->q[9]*b->q[7] + a->q[10]*b->q[11] + a->q[11];

   a->q[0] = ret[0];
   a->q[1] = ret[1];
   a->q[2] = ret[2];
   a->q[3] = ret[3];
   a->q[4] = ret[4];
   a->q[5] = ret[5];
   a->q[6] = ret[6];
   a->q[7] = ret[7];
   a->q[8] = ret[8];
   a->q[9] = ret[9];
   a->q[10] = ret[10];
   a->q[11] = ret[11];

}

BTINLINE vect_3* matXvec_mh(matr_h* a, vect_3* b)
{
   btreal ret[3];

   ret[0] = a->q[0]*b->q[0] + a->q[1]*b->q[1] + a->q[2]*b->q[2] + a->q[3];
   ret[1] = a->q[4]*b->q[0] + a->q[5]*b->q[1] + a->q[6]*b->q[2] + a->q[7];
   ret[2] = a->q[8]*b->q[0] + a->q[9]*b->q[1] + a->q[10]*b->q[2] + a->q[11];

   b->ret->q[0] = ret[0];
   b->ret->q[1] = ret[1];
   b->ret->q[2] = ret[2];

   return b->ret;
}

btreal getval_mh(matr_h* src, int row, int col)
{
   return src->q[row*4 + col];
}

void print_mh(matr_h* src)
{
   int i,j;

   for(i = 0;i<3;i++) {
      printf("\n| ");
      for(j = 0;j<4;j++)
         printf("% 6.3f ",src->q[4*i + j]);
      printf("|");
   }
   printf("\n");
}

int test_mh(btreal error)
{
   int cnt;
   matr_h* t1,*t2,*t3;

   printf("\nNew t1,t2:");

   t1 = new_mh();
   t2 = new_mh();
   t3 = new_mh();
   print_mh(t1);

   printf("\nSet Col t1:");
   setrow_mh(t1,1,1.0,2.0,3.0,4.0);
   print_mh(t1);

   printf("\nSet Col t2:");
   setrow_mh(t2,2,2.0,2.0,2.0,2.0);
   print_mh(t2);

   printf("\nMul t2 = t2*t1:");
   set_mh(t3,mul_mh(t1,t2));
   print_mh(t3);

   printf("\nMul t2 = t2*t1:");
   set_mh(t2,mul_mh(t1,t2));
   print_mh(t2);

}

int bench_mh()
{
   int cnt;
   matr_h* t1,*t2,*t3;

   printf("\nNew t1,t2:");
   t1 = new_mh();
   t2 = new_mh();
   t3 = new_mh();
   print_mh(t1);


   for (cnt = 0;cnt < 10000;cnt++) {

   }

}

/**************************************** End matr_h functions ****************/
/********************** matr_3 functions **************************************/

/** Allocate and initialize a new matr_3 object. See new_mh() */
matr_3 * new_m3() //allocates memory and sets to identity
{
   return(new_mh());
}

/** Copy src matrix to dest matrix
*/
BTINLINE void set_m3(matr_3* dest, matr_3* src)
{
   dest->q[0] = src->q[0];
   dest->q[1] = src->q[1];
   dest->q[2] = src->q[2];
   dest->q[3] = src->q[3];
   dest->q[4] = src->q[4];
   dest->q[5] = src->q[5];
   dest->q[6] = src->q[6];
   dest->q[7] = src->q[7];
   dest->q[8] = src->q[8];
   dest->q[9] = src->q[9];
   dest->q[10] = src->q[10];
   dest->q[11] = src->q[11];
}

BTINLINE void setrow_m3(matr_3* dest,int row, btreal s1, btreal s2, btreal s3)
{
   btreal *q;
   q = dest->q + 4*row;
   q[0] = s1;
   q[1] = s2;
   q[2] = s3;

}

BTINLINE void ident_m3(matr_3* dest)
{
   dest->q[0] = 1.0;
   dest->q[1] = 0.0;
   dest->q[2] = 0.0;
   dest->q[3] = 0.0;
   dest->q[4] = 0.0;
   dest->q[5] = 1.0;
   dest->q[6] = 0.0;
   dest->q[7] = 0.0;
   dest->q[8] = 0.0;
   dest->q[9] = 0.0;
   dest->q[10] = 1.0;
   dest->q[11] = 0.0;
}

BTINLINE void getcol_m3(vect_3* dest, matr_3* src, int n)
{
   btreal *q;
   q = src->q + n;
   dest->q[0] = *q;
   q += 4;
   dest->q[1] = *q;
   q += 4;
   dest->q[2] = *q;
}

BTINLINE btreal getval_m3(matr_3* src, int row, int col)
{
   return src->q[row*4 + col];
}

/** Multiply two matr_3 matrices
\bug This can be further optimized by assuming an orthonormal matrix and calculating
the third column as a cross product of the first two
*/
BTINLINE matr_h* mul_m3(matr_3* a,matr_3* b)
{
   btreal ret[12];
   //col 1
   ret[0] = a->q[0]*b->q[0] + a->q[1]*b->q[4] + a->q[2]*b->q[8];
   ret[4] = a->q[4]*b->q[0] + a->q[5]*b->q[4] + a->q[6]*b->q[8];
   ret[8] = a->q[8]*b->q[0] + a->q[9]*b->q[4] + a->q[10]*b->q[8];

   //col 2
   ret[1] = a->q[0]*b->q[1] + a->q[1]*b->q[5] + a->q[2]*b->q[9];
   ret[5] = a->q[4]*b->q[1] + a->q[5]*b->q[5] + a->q[6]*b->q[9];
   ret[9] = a->q[8]*b->q[1] + a->q[9]*b->q[5] + a->q[10]*b->q[9];

   //col 3
   ret[2] = a->q[0]*b->q[2] + a->q[1]*b->q[6] + a->q[2]*b->q[10];
   ret[6] = a->q[4]*b->q[2] + a->q[5]*b->q[6] + a->q[6]*b->q[10];
   ret[10] = a->q[8]*b->q[2] + a->q[9]*b->q[6] + a->q[10]*b->q[10];

   //col 4
   ret[3] = 0.0;
   ret[7] = 0.0;
   ret[11] = 0.0;

   a->ret->q[0] = ret[0];
   a->ret->q[1] = ret[1];
   a->ret->q[2] = ret[2];
   a->ret->q[3] = ret[3];
   a->ret->q[4] = ret[4];
   a->ret->q[5] = ret[5];
   a->ret->q[6] = ret[6];
   a->ret->q[7] = ret[7];
   a->ret->q[8] = ret[8];
   a->ret->q[9] = ret[9];
   a->ret->q[10] = ret[10];
   a->ret->q[11] = ret[11];

   return a->ret;
}

BTINLINE void setmul_m3(matr_3* a,matr_3* b)
{
   btreal ret[12];
   //col 1
   ret[0] = a->q[0]*b->q[0] + a->q[1]*b->q[4] + a->q[2]*b->q[8];
   ret[4] = a->q[4]*b->q[0] + a->q[5]*b->q[4] + a->q[6]*b->q[8];
   ret[8] = a->q[8]*b->q[0] + a->q[9]*b->q[4] + a->q[10]*b->q[8];

   //col 2
   ret[1] = a->q[0]*b->q[1] + a->q[1]*b->q[5] + a->q[2]*b->q[9];
   ret[5] = a->q[4]*b->q[1] + a->q[5]*b->q[5] + a->q[6]*b->q[9];
   ret[9] = a->q[8]*b->q[1] + a->q[9]*b->q[5] + a->q[10]*b->q[9];

   //col 3
   ret[2] = a->q[0]*b->q[2] + a->q[1]*b->q[6] + a->q[2]*b->q[10];
   ret[6] = a->q[4]*b->q[2] + a->q[5]*b->q[6] + a->q[6]*b->q[10];
   ret[10] = a->q[8]*b->q[2] + a->q[9]*b->q[6] + a->q[10]*b->q[10];

   //col 1
   ret[3] = 0.0;
   ret[7] = 0.0;
   ret[11] = 0.0;

   a->q[0] = ret[0];
   a->q[1] = ret[1];
   a->q[2] = ret[2];
   a->q[3] = ret[3];
   a->q[4] = ret[4];
   a->q[5] = ret[5];
   a->q[6] = ret[6];
   a->q[7] = ret[7];
   a->q[8] = ret[8];
   a->q[9] = ret[9];
   a->q[10] = ret[10];
   a->q[11] = ret[11];
}

/** Transpose of a
*/
BTINLINE matr_3* T_m3(matr_3* a)
{
   btreal ret[7];

   //swap variables
   ret[1] = a->ret->q[1];
   ret[2] = a->ret->q[2];
   ret[6] = a->ret->q[6];

   a->ret->q[1] = a->q[4];
   a->ret->q[2] = a->q[8];
   a->ret->q[6] = a->q[9];

   a->ret->q[4] = ret[1];
   a->ret->q[8] = ret[2];
   a->ret->q[9] = ret[6];
   return a->ret;
}

/** Multiply a 3x3 matrix by a 3-element vector
   \param a 3x3 matr_mn matrix to be multiplied
   \param b 3-element vect_3 to multiply against the matrix
   */
BTINLINE vect_3* matr_mnXvect_3(matr_mn* a, vect_3* b)
{
   b->ret->q[0] = a->q[0]*b->q[0] + a->q[1]*b->q[1] + a->q[2]*b->q[2];
   b->ret->q[1] = a->q[3]*b->q[0] + a->q[4]*b->q[1] + a->q[5]*b->q[2];
   b->ret->q[2] = a->q[6]*b->q[0] + a->q[7]*b->q[1] + a->q[8]*b->q[2];
   
   return b->ret;
}

BTINLINE vect_3* matXvec_m3(matr_3* a, vect_3* b)
{
   btreal ret[3];

   ret[0] = a->q[0]*b->q[0] + a->q[1]*b->q[1] + a->q[2]*b->q[2];
   ret[1] = a->q[4]*b->q[0] + a->q[5]*b->q[1] + a->q[6]*b->q[2];
   ret[2] = a->q[8]*b->q[0] + a->q[9]*b->q[1] + a->q[10]*b->q[2];

   b->ret->q[0] = ret[0];
   b->ret->q[1] = ret[1];
   b->ret->q[2] = ret[2];

   return b->ret;
}

/** Multiply the transpose of a 3x3 matrix by a 3-element vector 
   \param a 3x3 matr_3 matrix to be transposed then multiplied
   \param b 3-element vect_3 to multiply against the transposed matrix
   */
BTINLINE vect_3* matTXvec_m3(matr_3* a, vect_3* b)
{
   //matXvec_m3(T_m3(a),b);
   btreal ret[3];

   ret[0] = a->q[0]*b->q[0] + a->q[4]*b->q[1] + a->q[8]*b->q[2];
   ret[1] = a->q[1]*b->q[0] + a->q[5]*b->q[1] + a->q[9]*b->q[2];
   ret[2] = a->q[2]*b->q[0] + a->q[6]*b->q[1] + a->q[10]*b->q[2];

   b->ret->q[0] = ret[0];
   b->ret->q[1] = ret[1];
   b->ret->q[2] = ret[2];

   return b->ret;
}

BTINLINE vect_3* eqaxis_m3(matr_3* R, vect_3* a)
{
   btreal theta,st;

   theta = acos((R->q[0] + R->q[5] + R->q[10] - 1.0)/2);
   st = 0.5/sin(theta);
   a->q[0] = theta*st*(R->q[9] - R->q[6]);
   a->q[1] = theta*st*(R->q[2] - R->q[8]);
   a->q[2] = theta*st*(R->q[4] - R->q[1]);

   return a;
}

BTINLINE vect_3* RtoXYZf_m3(matr_3* R, vect_3* XYZ)
{
   //return XYZ
   btreal a,b,g,cb;
   //const btreal pi = 3.14159265359;

   b = atan2_bt(-1.0*R->q[8],sqrt(R->q[0]*R->q[0] + R->q[4]*R->q[4]));
   cb = cos(b);
   if (cb == 0.0) {
      if (b > 0.0) {
         b = pi/2.0;
         a = 0.0;
         g = atan2(R->q[1],R->q[5]);
      } else {
         b = -pi/2.0;
         a = 0.0;
         g = -1.0*atan2(R->q[1],R->q[5]);
      }
   } else {
      a = atan2(R->q[4]/cb,R->q[0]/cb);
      g = atan2(R->q[9]/cb,R->q[10]/cb);
   }
   const_v3(XYZ,a,b,g);
   return XYZ;
}

BTINLINE vect_3* RtoZYZf_m3(matr_3* R, vect_3* XYZ)
{
   //return ZYZ
   btreal a,b,g,sb;
   //const btreal pi = 3.14159265359;

   b = atan2_bt(sqrt(R->q[8] * R->q[8] + R->q[9] * R->q[9]), R->q[10]);
   sb = sin(b);
   if (sb == 0.0) {
      if (b > pi/2) {
         b = pi;
         a = 0.0;
         g = atan2(R->q[1],-R->q[0]);
      } else {
         b = 0.0;
         a = 0.0;
         g = atan2(-R->q[1],R->q[0]);
      }
   } else {
      a = atan2(R->q[6]/sb,R->q[2]/sb);
      g = atan2(R->q[9]/sb,-R->q[8]/sb);
   }
   const_v3(XYZ,a,b,g);
   return XYZ;
}

BTINLINE matr_3* XYZftoR_m3(matr_3* R, vect_3* XYZ)
{
   //Return R
   btreal ca,sa,cb,sb,cg,sg;

   ca = cos(getval_v3(XYZ,0));
   sa = sin(getval_v3(XYZ,0));
   cb = cos(getval_v3(XYZ,1));
   sb = sin(getval_v3(XYZ,1));
   cg = cos(getval_v3(XYZ,2));
   sg = sin(getval_v3(XYZ,2));

   setrow_m3(R,0,ca*cb,ca*sb*sg - sa * cg, ca*sb*cg+sa*sg);
   setrow_m3(R,1,sa*cb,sa*sb*sg + ca * cg, sa*sb*cg - ca*sg);
   setrow_m3(R,2,-sb,cb*sg,cb*cg);
   return R;
}

/***************************  btfilter   **********************************/


/** Allocates memory for and initializes a btfilter object.
 
\code
#include "btmath.h"
int main()
{
  btfilter *filt;
  btreal x,y;
  
  filt = new_btfilter(4);
  init_btfilter_diff(filt,0.001,30);
  while(1){
    y = eval_btfilter(filt,x);
  }
  
}
 
\endcode
 
*/
btfilter * new_btfilter(int size)
{
   void *vmem;
   int size_;
   btfilter *ptr;

   size_ = size;
   if (size_ < 5)
      size_ = 5;

   vmem = btmalloc(sizeof(btfilter)+size_*4*sizeof(btreal));

   //addbtptr(vmem);
   ptr = (btfilter*)vmem;
   ptr->d = (btreal*)(vmem + sizeof(btfilter));
   ptr->n = (btreal*)(vmem + sizeof(btfilter) + size_*sizeof(btreal));
   ptr->x = (btreal*)(vmem + sizeof(btfilter) + 2*size_*sizeof(btreal));
   ptr->y = (btreal*)(vmem + sizeof(btfilter) + 3*size_*sizeof(btreal));
   ptr->size = size_;
   return ptr;
}

void syslog_filter(btfilter *filt)
{
   int i;
   syslog(LOG_ERR,"Filter Parameters Dump: ptr %x",filt);
   syslog(LOG_ERR,"Order: %d, size: %d, zeta: %d, idx: %d",filt->order,filt->size,filt->zeta,filt->index);

   for (i = 0; i < filt->size; i++) {
      syslog(LOG_ERR,"d[%d]:%f",i,filt->d[i]);
   }
   for (i = 0; i < filt->size; i++) {
      syslog(LOG_ERR,"n[%d]:%f",i,filt->n[i]);
   }
   for (i = 0; i < filt->size; i++) {
      syslog(LOG_ERR,"x[%d]:%f",i,filt->x[i]);
   }
   for (i = 0; i < filt->size; i++) {
      syslog(LOG_ERR,"y[%d]:%f",i,filt->y[i]);
   }

}

/** Evaluates a btfilter object.
 
After a btfilter object is allocated with new_btfilter() and 
initialized with init_btfilter_xx(), use eval_btfilter() to add the present 
incoming value and get the resulting filtered value.
 
*/
btreal eval_btfilter(btfilter *filt, btreal xnew)
{
   int       i,j,idx;
   btreal      ynew;

   idx = filt->index;

   filt->x[idx] = xnew;
   filt->y[idx] = 0;

   for (i=0; i<=filt->order; i++) {
      j = (idx+i+1)%(filt->order+1);
      filt->y[idx] += filt->n[i] * filt->x[j]  - filt->d[i] * filt->y[j];
   }

   ynew  = filt->y[idx];
   filt->index = (++idx) % (filt->order+1);

   return(ynew);
}

/**
  Initializes coefficients for differentiator plus first or
  second order lowpass filter.
 
    You must initialize the following arguments in main:
       diff.order, diff.cutoffHz, sample_time
*/
void  init_btfilter_diff(btfilter *filt, int order, btreal sample_time, btreal cutoffHz)
{
   btreal woT;
   btreal temp;
   int i;
   //const btreal pi = 3.14159265359;

   filt->index = 0;
   filt->order = order;
   for(i=0; i<=4; i++) {
      filt->x[i] = 0;
      filt->y[i] = 0;
   }

   if(order == 1) {
      woT = sample_time*cutoffHz*2.0*pi;
      temp = 1.0/(2+woT);

      filt->d[1] = 0.0;
      filt->d[0] = -(2-woT)*temp;

      filt->n[1] = 2.0*cutoffHz*pi*2.0*temp;
      filt->n[0] = -(filt->n[1]);
   } else if(order == 2) {
      woT = sample_time*cutoffHz*2.0*pi;
      temp = 1.0/ ( 4.0*(1.0+woT) + woT*woT  );

      filt->d[2] = 0.0;
      filt->d[1] = -(8.0 - 2.0*woT*woT) * temp;
      filt->d[0] = ( 4.0*(1.0-woT) + woT*woT  ) * temp;

      filt->n[2] = 2.0*cutoffHz*pi*2.0*woT * temp;
      filt->n[1] = 0.0;
      filt->n[0] = -(filt->n[2]);
   } else {
      syslog(LOG_ERR,"btfilter: Filter order must be 1 or 2.\n");

   }

}
/**\bug something wrong with this.*/
void  init_btfilter_butterworth_diff(btfilter *filt, btreal sample_time, btreal cutoffHz)
{
   //  Initializes coefficients for differentiator plus first or
   //  second order lowpass filter.

   // You must initialize the following arguments in main:
   //    diff.order, diff.cutoffHz, sample_time

   double woT,wo;
   double norm;
   //const btreal pi = 3.14159265359;
   int i;

   filt->index = 0;
   filt->order = 2;
   for(i=0; i<=4; i++) {
      filt->x[i] = 0.0;
      filt->y[i] = 0.0;
   }
   wo = cutoffHz;
   woT = wo*sample_time*2.0*pi;

   norm = 1+sqrt(2.0)*woT+woT*woT;

   filt->n[0]=0;
   filt->n[1]=-woT*wo/norm;
   filt->n[2]=wo*wo/norm;

   filt->d[0]=1/norm;
   filt->d[1]=-(2+sqrt(2.0)*woT)/norm;
   filt->d[2]=0;


}
void  init_btfilter_lowpass(btfilter *filt, btreal sample_time, btreal cutoffHz, btreal zeta)
{
   //  Initializes coefficients for lowpass filter with 2 complex poles
   //  (or real poles if zeta=1).
   //  Zeta is the damping ratio, filtcutHz is the cutoff frequency.

   // You must initialize the following arguments in main:
   //    lowpass.order, lowpass.cutoffHz, lowpass.zeta, sample_time

   double woT;
   double uu, uu2;
   int i;
   //const btreal pi = 3.14159265359;

   filt->index = 0;
   filt->order = 2;
   filt->zeta = zeta;

   for(i=0; i<=4; i++) {
      filt->x[i] = 0;
      filt->y[i] = 0;
   }

   woT = sample_time*cutoffHz*2.0*pi;
   uu  = 2.0/woT;
   uu2 = uu*uu;

   filt->d[2] = (uu2 + 2.0*filt->zeta*uu + 1.0);
   filt->d[1] = -(2.0*uu2-2.0)/filt->d[2];
   filt->d[0] = ( uu2 - 2.0*filt-> zeta*uu + 1.0 )/filt->d[2];

   filt->n[2] = 1.0/filt->d[2];
   filt->n[1] = 2.0/filt->d[2];
   filt->n[0] = 1.0/filt->d[2];

   filt->d[2] = 0.0;
}

/** 2 parameter arc tangent
 
*/
BTINLINE btreal atan2_bt(btreal y, btreal x)
{
   /* returns angle from -pi to pi */
   //const btreal pi = 3.14159265359;

   if(x > 0.0)
      return(atan(y / x));
   
   if(x < 0.0) {
      if(y >= 0.0)
         return( pi + atan(y / x));
      else
         return( -pi + atan(y / x));
   }
   
   /* x = 0 */
   if(y > 0.0)
      return(pi/2.0);
   else
      return(-pi/2.0);
}
#if 0
BTINLINE btreal atan2_bt(btreal arg1, btreal arg2)
{
   /* returns angle from -pi to pi */
   //const btreal pi = 3.14159265359;


   if(arg2 > 0.0) {
      if(arg1 >= 0.0) {
         return(atan(arg1 / arg2));
      } else {
         return(-1.0 * atan(arg1 / arg2));
      }
   }
   else if(arg2 < 0.0) {
      if(arg1 >= 0.0) {
         return(pi - atan(arg1 / arg2));
      } else {
         return( -1.0 * (pi - atan(arg1 / arg2)));
      }
   }
   else {
      /* x = 0 */
      if(arg1 > 0.0)
         return(pi/2.0);
      else
         return(-pi/2.0);
   }
}
#endif

btreal interp_bt(btreal x1, btreal y1, btreal x2, btreal y2, btreal x)
{
   if ((x2-x1) != 0.0)
      return (x*(y2-y1)/(x2-x1));
   else
      return (y2+y1)/2;
}

/**
 
*/
btreal interpolate_bt(btreal x1, btreal y1, btreal x2, btreal y2, btreal x)
{
   if ((x2-x1) != 0.0)
      return (y1 + (x-x1)*(y2-y1)/(x2-x1));
   else
      return (y2+y1)/2;
}

btfilter_vn * new_btfilter_vn(int size,int vsize)
{
   void *vmem;
   btfilter_vn *ptr;
   int size_;

   size_ = size;
   if (size_ < 5)
      size_ = 5;

   vmem = btmalloc(sizeof(btfilter_vn));

   //addbtptr(vmem);
   ptr = (btfilter_vn*)vmem;

   ptr->d = new_vr(vsize,size_);
   ptr->n = new_vr(vsize,size_);
   ptr->x = new_vr(vsize,size_);
   ptr->y = new_vr(vsize,size_);
   ptr->scratch1 = new_vn(vsize);
   ptr->scratch2 = new_vn(vsize);
   ptr->scratch3 = new_vn(vsize);

   return ptr;
}

vect_n * eval_btfilter_vn(btfilter_vn *filt, vect_n *xnew)
{
   int       i,idx,idx2;
   btreal      ynew;
   vect_n *ret;

   idx = filt->index;

   set_vn(idx_vr(filt->x,idx),xnew);
   fill_vn(idx_vr(filt->y,idx),0.0);

   for (i=0; i<=filt->order; i++) {
      idx2 = (idx+i+1)%(filt->order+1);
      set_vn(idx_vr(filt->y,idx),add_vn(e_mul_vn( mapdat_vr(filt->scratch2,filt->n,i),mapdat_vr(filt->scratch1,filt->x,idx2)),
                                        add_vn(scale_vn(-1.0,e_mul_vn(mapdat_vr(filt->scratch1,filt->d,i),
                                                                      mapdat_vr(filt->scratch2,filt->y,idx2))),
                                               mapdat_vr(filt->scratch3,filt->y,idx))));

   }
   ret = idx_vr(filt->y,idx);
   filt->index = (++idx) % (filt->order+1);

   return(ret);
}

/**\bug something wrong with this.*/
void  init_btfilter_vn_butterworth_diff(btfilter_vn *filt, int order, btreal sample_time, btreal cutoffHz)
{
   double woT,wo;
   double norm;
   int i;

   filt->index = 0;
   filt->order = 2;

   for(i=0; i<=4; i++) {
      fill_vn(idx_vr(filt->x,i),0.0);
      fill_vn(idx_vr(filt->y,i),0.0);
   }
   wo = cutoffHz;
   woT = wo*sample_time;

   norm = 1+sqrt(2.0)*woT+woT*woT;

   fill_vn(idx_vr(filt->n,0),0.0);
   fill_vn(idx_vr(filt->n,1),-woT*wo/norm);
   fill_vn(idx_vr(filt->n,2),wo*wo/norm);

   fill_vn(idx_vr(filt->d,0),1/norm);
   fill_vn(idx_vr(filt->d,1),-(2+sqrt(2.0)*woT)/norm);
   fill_vn(idx_vr(filt->d,2),0.0);

}

/**
  Initializes coefficients for differentiator plus first or
  second order lowpass filter.
 
    You must initialize the following arguments in main:
       diff.order, diff.cutoffHz, sample_time
*/
void  init_btfilter_vn_diff(btfilter_vn *filt, int order, btreal sample_time, btreal cutoffHz)
{
   btreal woT;
   btreal temp;
   int i;
   //const btreal pi = 3.14159265359;

   filt->index = 0;
   filt->order = order;

   for(i=0; i<=4; i++) {
      fill_vn(idx_vr(filt->x,i),0.0);
      fill_vn(idx_vr(filt->y,i),0.0);
   }

   if(order == 1) {
      woT = sample_time*cutoffHz*2.0*pi;
      temp = 1.0/(2+woT);

      fill_vn(idx_vr(filt->d,0),-(2-woT)*temp);
      fill_vn(idx_vr(filt->d,1),0.0);

      fill_vn(idx_vr(filt->n,0),-2.0*cutoffHz*pi*2.0*temp);
      fill_vn(idx_vr(filt->n,1),2.0*cutoffHz*pi*2.0*temp);
   } else if(order == 2) {
      woT = sample_time*cutoffHz*2.0*pi;
      temp = 1.0/ ( 4.0*(1.0+woT) + woT*woT  );

      fill_vn(idx_vr(filt->d,0),( 4.0*(1.0-woT) + woT*woT  ) * temp);
      fill_vn(idx_vr(filt->d,1),-(8.0 - 2.0*woT*woT) * temp);
      fill_vn(idx_vr(filt->d,2),0.0);

      fill_vn(idx_vr(filt->n,0),-2.0*cutoffHz*pi*2.0*woT * temp);
      fill_vn(idx_vr(filt->n,1),0.0);
      fill_vn(idx_vr(filt->n,2),2.0*cutoffHz*pi*2.0*woT * temp);
   } else {
      syslog(LOG_ERR,"btfilter: Filter order must be 1 or 2.\n");
   }
}

void test_btfilter()
{
   btfilter *test;
   double res,inp;
   int cnt;

   test = new_btfilter(5);
   //init_btfilter_butterworth_diff(test,0.002,30);
   init_btfilter_diff(test,2,0.002,30);
   for (cnt = 0; cnt < 4; cnt++) {
      printf("%8.4f, %8.4f\n",test->n[cnt],test->d[cnt]);
   }
   res = 0.0;
   inp = 1.0;

   for (cnt = 0; cnt < 30; cnt++) {
      inp += 0.0;
      res = eval_btfilter(test,inp);
      if ((cnt % 1) == 0)
         printf("%8.4f\n",res);
   }

}

void test_filter_vn()
{
   btfilter_vn *test;
   vect_3 *res,*inp;
   int cnt,cnt2;
   char buf1[100],buf2[100],buf3[100],buf4[100];


   res = new_v3();
   inp = new_v3();

   test = new_btfilter_vn(5,3);
   init_btfilter_vn_diff(test,2,0.002,30);
   for (cnt2 = 0; cnt2 < 4; cnt2++) {
      sprint_vn(buf1,idx_vr(test->n,cnt2));
      sprint_vn(buf2,idx_vr(test->d,cnt2));
      printf("\n%s %s ",buf1,buf2);
   }
   const_v3(inp,1.0,1.0,1.1);
   for (cnt = 0; cnt < 30; cnt++) {
      //set_v3(inp,add_v3(inp,C_v3(1.0,0.0,0.0)));
      set_v3(res,(vect_3*)eval_btfilter_vn(test,(vect_n*)inp));
      if ((cnt % 1) == 0) {
         //    printf("%d:\n",cnt);
         print_vn((vect_n*)res);
         /*
         for (cnt2 = 0; cnt2 < 4; cnt2++)
      {
           sprint_vn(buf1,idx_vr(test->x,cnt2));
           sprint_vn(buf2,idx_vr(test->y,cnt2));
           //sprint_vn(buf3,idx_vr(test->n,cnt2));
           //sprint_vn(buf4,idx_vr(test->d,cnt2));
           printf("\n%s %s ",buf1,buf2);
      }*/
      }
   }

}

btreal max_bt(btreal x,btreal y)
{
   if (x > y)
      return x;
   else
      return y;

}

btreal min_bt(btreal x,btreal y)
{
   if (x < y)
      return x;
   else
      return y;

}

#define TINY 1.0e-20

/** LU-Decomposition of a matrix.
   \param a Input matrix, overwritten by the LU-Decomposition
   \param indx Output vector of row permutations
   \param d Output +1 for even permutation count, else -1
   */
matr_mn* ludcmp(matr_mn *a, vect_n *indx, btreal *d)
{
   int i, imax, j, k;
   btreal big, dum, sum, temp;
   btreal *vv;
   
   vv = a->ret->q; // Using a->m elements of the scratch space of matrix a
   
   *d = 1.0;
   for(i = 0; i < a->m; i++){
      big = 0.0;
      for(j = 0; j < a->m; j++)
         if((temp = fabs(getval_mn(a, i, j))) > big) 
            big = temp;
      if(big == 0.0)
         syslog(LOG_ERR, "Singular matrix in ludcmp");
      vv[i] = 1.0 / big;
   }
   for(j = 0; j < a->m; j++){
      for(i = 0; i < j; i++){
         sum = getval_mn(a,i,j);
         for(k = 0; k < i; k++)
            sum -= getval_mn(a,i,k) * getval_mn(a,k,j);
         setval_mn(a,i,j,sum);
      }
      big = 0.0;
      for(i = j; i < a->m; i++){
         sum = getval_mn(a,i,j);
         for(k = 0; k < j; k++)
            sum -= getval_mn(a,i,k) * getval_mn(a,k,j);
         setval_mn(a,i,j,sum);
         if((dum = vv[i] * fabs(sum)) >= big){
            big = dum;
            imax = i;
         }
      }
      if(j != imax){
         for(k = 0; k < a->m; k++){
            dum = getval_mn(a,imax,k);
            setval_mn(a,imax,k,getval_mn(a,j,k));
            setval_mn(a,j,k,dum);
         }
         *d = -(*d);
         vv[imax] = vv[j];
      }
      setval_vn(indx,j,imax);
      if(getval_mn(a,j,j) == 0.0)
         setval_mn(a,j,j,TINY);
      if(j != a->m-1){
         dum = 1.0 / getval_mn(a,j,j);
         for(i = j+1; i < a->m; i++)
            setval_mn(a,i,j,getval_mn(a,i,j) * dum);
      }
   }
   return(a);
}

vect_n* lubksb(matr_mn *a, vect_n *indx, vect_n *b)
{
   int i, ii = 0, ip, j;
   float sum;
   
   for(i = 0; i < a->m; i++){
      ip=getval_vn(indx,i);
      sum = getval_vn(b,ip);
      setval_vn(b,ip,getval_vn(b,i));
      if(ii)
         for(j = 0; j < i; j++)
            sum -= getval_mn(a,i,j) * getval_vn(b,j);
      else if (sum)
         ii = 1;
      setval_vn(b,i,sum);
   }
   for(i = a->m-1; i >= 0; i--){
      sum = getval_vn(b,i);
      for(j = i+1; j < a->m; j++)
         sum -= getval_mn(a,i,j) * getval_vn(b,j);
      setval_vn(b,i,sum / getval_mn(a,i,i));
   }
}

/** Invert a square matrix
   \param a Input square matrix to invert
   \param indx Output vector of row permutations
   \param col Output column selection
   
   \note Input matrix will be destroyed
   */
matr_mn* inv_mn(matr_mn *a, vect_n *indx, vect_n *col)
{
   btreal d;
   int i, j;
   
   ludcmp(a, indx, &d);
   for(j = 0; j < a->m; j++){
      for(i = 0; i < a->m; i++)
         setval_vn(col, i, 0.0);
      setval_vn(col, j, 1.0);
      lubksb(a, indx, col);
      setcol_mn(a->ret, col, j);
      //for(i = 0; i < a->m; i++)
      //   setval_mn(a->ret, i, j, getval_vn(col, i));
   }
   return a->ret;
}

/** Find the determinant of a square matrix
   \param a Input square matrix
   \param indx Output vector of row permutations
   
   \note Input matrix will be destroyed
   */
btreal det_mn(matr_mn *a, vect_n *indx)
{
   btreal d;
   int j;
   
   ludcmp(a, indx, &d);
   for(j = 0; j < a->m; j++)
      d *= getval_mn(a,j,j);
   
   return d;
}

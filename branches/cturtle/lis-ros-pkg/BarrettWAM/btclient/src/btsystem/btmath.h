/* ======================================================================== *
 *  Module ............. libbtsystem
 *  File ............... btmath.h
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

/** \file btmath.h
    \brief A common set of mathematical operations. 
    
    This code provides objects and methods for:
    - N Element Vectors
      - Optimization for 3 Element Vectors
    - NxM Element Matrices
      - Optimization for SO(3) (3x3 rotation matrices)
      - Optimization for SE(3) (4x4 rotation & translation matrices)
    - Quaternions
    - Digital Filters
 
    The core functionality for vectors, matrices and quaternions are supported 
    by the *_vn, *_mn, and *_q functions. Use these functions as a starting point in any
    algorithm you are developing. This is a performance library and so optimized 
    fuctions are provided for specific cases. *_v3 (3 element vector) *_m3 (3x3 rotation
    matrix) *_qu (unit length rotation quaternion).  You may call any general functions 
    on the optimized data types by normal C typecasting. For example, the following is
    valid code.
    \code 
      vect_3 *v,*x;
      v = new_v3();
      x = new_v3();
      const_v3(v,4.4,5.5,6.5);
      set_v3(x,(vect_3*)bound_vn((vect_n*)v,1.0,5,0));
    \endcode
    
    
    The vector and matrix functions are written to provide a consistent syntax.
Prefix notation is used for all operations. See the test_*() functions for
example of use.
 
The only potentially tricky thing about this api is that you should never declare
a vector or matrix variable. Only declare pointers. The new_xx function expects the
address of your pointer, and will allocate memory and redirect your pointer for you.
    


Segfaults are a problem with realtime code because we don't necessarily exit on the 
instruction that is accessing the wrong memory.

Particularly frustrating is when gdb says that we segfaulted on a sleep function.
The following is a list of instances of when this has happened to give you some ideas
where to look first.

1. calling sprint_vn() on an uninitialized vect_n *
\internal 
\todo We need to do the following
 - Add code and compiler switches for counting the number of operations.
 - Change optimized functions to use generalized objects.
 - Use Numerical recipies conventions for matrices (array of pointers to rows)
 - Freelist memory implementation for fast vector allocation. (See thfaststack.c in
 experimental code section)
*/
/** @addtogroup btmath Extended Math Library

This is a general description so that I can see the output.
*/

#ifndef _BTMATH_H
#define _BTMATH_H

#include "btos.h"

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
 



#define PRACTICALLY_ZERO 0.0000001 //For avoiding divide by zero

//#define BTINLINE  
#define BTINLINE inline

/* ELEM(m,r,c) evaluates to m[r][c] for matrix m. Zero-based.
   Unfortunately, addressing a matrix as m[r][c] requires the dimensions
   to be defined at compile time. Since we define matrices dynamically at
   run-time, we must use this function to access matrix elements by row and
   column. This is what the compiler does when it sees m[r][c], anyway.
   */
#define ELEM(m,r,c) ((m)->q[(r) * (m)->n + (c)])

#define EPSILON (0.001)
#define pi (3.14159265359)

typedef double btreal; //!< Typedef floating point type

/** The list of all pointers allocated by btmath

  Btmemlist maintains a list of all pointers allocated by
  the btmath code and provides a function to free them all at once.
  This functionality is used internally. See addbtptr() for more 
  details. freebtptr() should be called at the end of every program
  that uses the btsystem library.
  
  \deprecated 
  \internal chk'd TH 051101
*/
typedef struct {
  int n,max;      //size of vector
  void **plist;  //pointer to data
}btmemlist;
//@{
int addbtptr(void *ptr);
int rembtptr(int idx);
void freebtptr();
//@}

/** @addtogroup vect_n N element vectors.
    @ingroup btmath
 */
//@{
/** An N element vector object.

vect_n is a linear algebra vector object. The elements are of type btreal. Function
definitions are in btmath.c. This 
object is optimized for speed and prefix notation. It occupies twice the memory
you would expect to accomplish this. In addition to standard vector math operations
there are some functions that will map an operation to the vector as an array of btreals 
similar to what matlab can do.

The vect_n object operator functions can be nested in prefix function form. set_vn() 
is the assignment function.

See new_vn(), and set_vn() for more information. A typical block of code using vect_n looks like:

\code 
      vect_n *a,*b; 
      
        //create a new object (required)
      a = new_vn(4);
      b = new_vn(4);
      
      const_vn(a,4.4,5.5,6.5,7.7);
      
      //b = a + 4 * a + a * a + a
      set_vn(b,add_vn(a,
               add_vn(scale_vn(4.0,a),
               add_vn(mul_vn(a,a),a))));
\endcode




  Rules: The output of any function can be used as the input of another. The leftmost
  argument will hold the results in it's scratch data space;
  
  \warning There is no bounds checking in the code. All vect_n objects must be created 
  with new_vn() and if you are mixing vectors of different sizes be careful to read the
  btmath.c code so that you understand the dangers of doing so. 
  
  \warning This code uses preallocated memory associated with the left parameter to pass return values. 
  \code
    /******* Example of bad code *******
    set_vn(b,add_vn(mul_vn(a,b),scale_vn(2.0,mul_vn(a,b))));
    
    /******* Recursion breakdown ******
    scale_vn(2.0,mul_vn(a,b)); //Return value in a.ret
    mul_vn(a,b); //A different return value is now overwriting a.ret
    add_vn(a.ret,a.ret); //
    
    /******* Example of good code ******
    set_vn(b,add_vn(mul_vn(b,a),scale_vn(2.0,mul_vn(a,b)))); 
  \endcode
   \internal chk'd TH 051101
  \todo We should add a compiler define switched set of code to each function to
add in bounds checking and verbose error reporting
*/

/*================================*/
typedef struct barrett_vect_n{
  //void *vtable;  //filler for D compatibility
  //void *monitor; //filler for D compatibility
  struct barrett_vect_n *ret;  //pointer to return data for stacked operations
 // btvect_stack *stk; //Stack object for temporary variable allocation
 // int stk_id; //location on the stack. -1 = not on the stack
  int n;      //size of vector
  btreal *q;  //pointer to data
  //pthread_mutex_t mutex;
  //int idx; //Listing in btptrs garbage collection structure
}vect_n;


/** Create a local vect_n
\param x variable name you want to use locally;
\param n Number of elements in vector

\code Usage
vect_n* add3()
{
   vect_n ret;
   local_vn(x,6);
   local_vn(y,7);
   
   init_vn(x,6);
   init_vn(y,6);
   
   ret = new_vn(6);
   fill_vn(x,1.0);
   fill_vn(y,2.0);
   set_vn(ret,add_vn(x,y));
   return ret;
}
\endcode
*/

/** #local_vn(x,n) 
x = variable name
n = max number of elements

*/
#define local_vn(x,n) vect_n x[(1+(sizeof(vect_n)*2+sizeof(btreal)*2*(n))/sizeof(vect_n))]

/*================================*/
//@}


/** @addtogroup vect_n_if Initialization & Data Access Functions
    @ingroup vect_n
    N element vector operation functions.
*/
//@{
vect_n* new_vn(int size); //allocate an n-vector
vect_n* init_vn(vect_n* v, int size); //Link up a vect_n object
void destroy_vn(vect_n **p);
int len_vn(vect_n *src); //number of elements in the vector
int sizeof_vn(vect_n *src); //return sizeof info for whole vector

vect_n* init_local_vn(vect_n* header,btreal* data, int size);
int new_vn_group(int num, int size, ...); //allocate a group of n-vectors
void set_vn(vect_n* dest, vect_n* src); //assignment, copy
void setrange_vn(vect_n* dest, vect_n* src, int dest_start, int src_start, int num);
void extract_vn(btreal* dest, vect_n* src); //copy vector to a btreal array
void inject_vn(vect_n* dest, btreal* src); //copy btreal array to vector
btreal* valptr_vn(vect_n* src); //return a pointer to the btreal array holding the data
void setval_vn(vect_n* dest, int idx, btreal val); //function to set single value, zero indexed
btreal getval_vn(vect_n* dest, int idx);
vect_n* const_vn(vect_n* a, ...); //set vector to a constant array of btreals
void einit_vn(vect_n* dest,int i); // einit_vn(&a,3) = <0,0,0,1,0,0>
void fill_vn(vect_n* dest, btreal val);
vect_n* subset_vn(vect_n* src,int start,int end);
void reset_vn(vect_n* src);
//@}


/** @addtogroup vect_n_of Operator Functions
    @ingroup vect_n
    N element vector operation functions.
*/
//@{
vect_n* neg_vn(vect_n* a); //negate a vector
vect_n* scale_vn(btreal a, vect_n* v);
vect_n* add_vn(vect_n* a, vect_n* b);
vect_n* sub_vn(vect_n* a, vect_n* b);




//vect_n* wedge_vn(vect_n* a, vect_n*b); 
btreal  dot_vn(vect_n* a, vect_n* b);
btreal  angle_vn(vect_n* a,vect_n* b);
btreal  norm_vn(vect_n* a); //euclidian norm
vect_n* unit_vn(vect_n* a); //unit vector
vect_n* unit_interp_vn(vect_n* a, vect_n* b,btreal s); //linear interpolation, if s is greater than available, it is exstrapolated
vect_n* interp_vn(vect_n* a, vect_n* b,btreal ds,btreal s);
vect_n* bound_vn(vect_n* a, btreal min, btreal max);
//int eq_vn(vect_n* a, vect_n* b); //equals
//int zero_vn(vect_n* a, vect_n* b); //zero vector test

//@}
/** @addtogroup vect_n_pe Per element operator Functions
    @ingroup vect_n 
    Vector math functions that treat vectors as arrays of btreals.
 */
//@{
vect_n* e_mul_vn(vect_n* a, vect_n* b); // Per Element multiply
vect_n* e_div_vn(vect_n* a, vect_n* b); // Per Element divide
vect_n* e_pow_vn(vect_n* a, btreal b);
vect_n* e_sqrt_vn(vect_n* a);
vect_n* e_sqr_vn(vect_n* a);

//@}
/** @addtogroup vect_n_io Input/Output Functions.
    @ingroup vect_n 
    Functions for vect_n screen and file IO.
 */
//@{
void print_vn(vect_n* src);
char* sprint_vn(char *dest,vect_n* src);
char* sprint_csv_vn(char *dest,vect_n* src);
char* sprint_plt_vn(char *dest,vect_n* src);//gnu plot format
void syslog_vn(char* header,vect_n* src);

int strcount_vn(char **src_string,char *delimiter); //Count the number of double values in a string
vect_n * sscan_vn(char *src);
vect_n * strto_vn(vect_n *dest,char *src,char *delimiters); //Convert a string to a vect_n
vect_n * csvto_vn(vect_n* dest, char *src); //Convert a string into a vect_n
int test_vn(btreal error); //test & verification suite
//@}








/** @addtogroup vectray Vector Array Functions.
    @ingroup btmath
    These functions are for manipulating arrays of vectors.
 */
//@{
/*===================== Vector Arrays=============================*/
/** A vector array object, each vector has the same number of elements.

vectray is an object for holding arrays of vectors of the same size. Function
definitions are in btmath.c.
A block of data is allocated all at once in the beginning. The vect_n data type 
is used to access it. All vectray objects must be created with new_vr().

\code
int test_vr(void)
{
  vectray *vr,*vr2;
  int cnt,idx;
  
  vr = new_vr(3,10);
  for (cnt = 0; cnt < 10; cnt++){
    fill_vn(idx_vr(vr,cnt),cnt);
    set_vn(idx_vr(vr,cnt),scale_vn(2.0,idx_vr(vr,cnt)));
    print_vn(idx_vr(vr,cnt));
  }
  
  read_csv_file_vr("test.csv",&vr2);
  for (cnt = 0; cnt < size_vr(vr2); cnt++){
    print_vn(idx_vr(vr2,cnt));
  }
}
\endcode
*/

typedef struct barrett_vectarray_n{
  btreal *data;
  vect_n *rayvect,*lval,*rval,*eval;
  unsigned int n;    //!< Number of elements in the vectors in this array
  unsigned int stride;  //!< Number of memory locations between each row. allows spliting one data memory block between vectrays.
  unsigned int max_rows; //!< Number of rows allocated (MAX Rows)
  int num_rows; //!< Number of rows filled so far
  int edit_point;//!< present index into the array
}vectray;

vectray * new_vr(int vect_size,int max_rows);
vectray * resize_vr(vectray **vr,int max_rows);
void destroy_vr(vectray **vr);

//Access

vect_n * mapdat_vr(vect_n *dest, vectray *ray, int idx); //map the data pointer of dest onto the index
BTINLINE vect_n * idx_vr(vectray *ray,int idx); // pointer into vectray
vect_n * lval_vr(vectray *ray,int idx);
vect_n * rval_vr(vectray *ray,int idx);
vect_n * edit_vr(vectray *ray);
vect_n * getvn_vr(vect_n *dest,vectray *ray, int idx); //copy data at idx to dest
BTINLINE int numrows_vr(vectray *ray); //returns the index of the last point
BTINLINE int numelements_vr(vectray *ray);
BTINLINE int maxrows_vr(vectray *ray); //ray->rows

//Add & Remove data
void copy_sub_vr(vectray *dest, vectray *src, int src_r, int dest_r, int rows, 
                                              int src_c, int dest_c, int columns);

void clear_vr(vectray *ray); //erase everything and set it to zero

// Edit point movement
void next_vr(vectray *ray);
void prev_vr(vectray *ray);
void start_vr(vectray *ray);
void end_vr(vectray *ray);
int edit_at_vr(vectray *ray,int idx);
int edit_point_vr(vectray *ray);
// Edit functions
int append_vr(vectray *ray, vect_n* v);
int insert_vr(vectray *ray, vect_n* v);
int delete_vr(vectray *ray);


// helper functions
btreal arclength_vr(vectray *ray);
//File I/O
int read_csv_file_vr(char *fileName, vectray **vr);
int write_csv_file_vr(char *filename, vectray *vr);

int test_vr(btreal error);
//@}
/** @addtogroup vect_3 3 Element Vector Optimized Functions
    @ingroup vect_n
    These functions are optimized for 3 element vectors.
 */
//@{

/*=======================================================================*/
/** A vect_n object optimized for 3 elements

vect_3 is structurally compatible with vect_n. As such, they can be typecast
as each other and used in each others functions. set_v3() 
is the assignment function. See new_v3(), and set_v3() for more. Function
definitions are in btmath.c.

\code
vect_n *a;
vect_3 *b;

a = new_vn(5);
b = new_v3();

fill_v3((vect_3*)a,1.0); //= <1.0, 1.0, 1.0, 0.0, 0.0>
set_v3(b,(vect_3*)a); //b= <1.0, 1.0, 1.0>
set_v3(b,neg_vn((vect_n*)b)); //b= <-1.0, -1.0, -1.0>
\endcode
*/
typedef struct barrett_vect_3{
  //void *vtable;  //filler for D compatibility
  //void *monitor; //filler for D compatibility
  struct barrett_vect_3* ret;  
  int n;      //size of vector
  btreal *q;
  btreal data[3];
}vect_3;

typedef struct {
  vect_3 main,scratch;
}staticv3;

vect_3 * new_v3(); //allocate an n-vector
vect_3 * init_staticv3(staticv3 *sv3);
vect_3* set_v3(vect_3* dest, vect_3* src); //assignment, copy
void extract_v3(btreal* dest, vect_3* src); //copy vector to a btreal array
void inject_v3(vect_3* dest, btreal* src); //copy btreal array to vector
void setval_v3(vect_3* dest, int idx, btreal val); //function to set single value, zero indexed
btreal getval_v3(vect_3* dest, int idx);
vect_3* const_v3(vect_3* dest, btreal v1, btreal v2, btreal v3); //set vector to a constant array of btreals
vect_3* C_v3(btreal v1, btreal v2, btreal v3); //set vector to a constant array of btreals
void einit_v3(vect_3* dest,int i); // einit_vn(&a,3) = <0,0,0,1,0,0>
void fill_v3(vect_3* dest, btreal val);

vect_3* neg_v3(vect_3* a); //negate a vector
vect_3* scale_v3(btreal a, vect_3* v);
vect_3* add_v3(vect_3* a, vect_3* b);
vect_3* sub_v3(vect_3* a, vect_3* b);

vect_3* cross_v3(vect_3* a, vect_3*b); 
btreal  dot_v3(vect_3* a, vect_3* b);
btreal  norm_v3(vect_3* a); //euclidian norm
vect_3* unit_v3(vect_3* a); //unit vector

vect_3* bound_v3(vect_3* a, btreal min, btreal max);

char* sprint_v3(char *dest,vect_3* src);
void print_v3(vect_3* src);
//int eq_v3(vect_3* a, vect_3* b); //equals
//int zero_v3(vect_3* a, vect_3* b); //zero vector test

/*=======================================================================*/
//@}

/*=======================Quaternion===========================================*/
/** @addtogroup quat Quaternion Functions
    @ingroup btmath
    These functions operate on 4 element vectors (quaternions).
 */
//@{
/** Quaternion object

The quat object is a quaternion; a 4 element imaginary number.
q = a + b*i + c*j + d*k 

The quaternion is stored in vector form as <a,b,c,d>. The quaternion objects use 
the same prefix function form as the vect_n object (and quat can be typecast as a
vect_n).

See new_q(), and set_q() for more. Function
definitions are in btmath.c.


*/
typedef struct barrett_quat{
  //void *vtable;  //filler for D compatibility
  //void *monitor; //filler for D compatibility
  struct barrett_quat* ret;  
  int n;      //size of vector
  btreal *q;
  btreal data[4];
}quat;

quat * new_q(); //allocate an n-vector
quat* set_q(quat* dest, quat* src); //assignment, copy
void extract_q(btreal* dest, quat* src); //copy vector to a btreal array
void inject_q(quat* dest, btreal* src); //copy btreal array to vector
void setval_q(quat* dest, int idx, btreal val); //function to set single value, zero indexed
btreal getval_q(quat* dest, int idx);
quat* const_q(quat* dest, btreal v1, btreal v2, btreal v3, btreal v4); //set vector to a constant array of btreals
quat* C_q(btreal v1, btreal v2, btreal v3, btreal v4); //set vector to a constant array of btreals
void fill_q(quat* dest, btreal val);


quat* conj_q(quat* a); //!< Conjugate. =s + -1*v
quat* inv_q(quat* a); //!< Inverse
quat* exp_q(quat* a); //!< Exponential
quat* log_q(quat* a); //!< Log o
quat* neg_q(quat* a); //negate a vector
quat* scale_q(btreal a, quat* v);
quat* add_q(quat* a, quat* b);
quat* sub_q(quat* a, quat* b);

quat* mul_q(quat* a, quat*b); 
quat* pow_q(quat* a, btreal b);
btreal dot_q(quat* a, quat* b);
btreal norm_q(quat* a); //euclidian norm
quat* unit_q(quat* a); //unit vector


btreal GCdist_q(quat* start, quat* end); //!< Great circle distance between two quaternions
btreal angle_q(quat* src); //!< Returns the angle represented by the quaternion
vect_3* axis_q(vect_3* dest, quat* src); //!< Returns the unit vector representing the rotation
vect_3* GCaxis_q(vect_3* dest, quat* start, quat* end); //!< Great circle axis between two quaternions
quat * force_closest_q(quat* src);
quat* slerp_q(quat* q0,quat* q1,btreal t);

char* sprint_q(char *dest,quat* src);
void print_q(quat* src);
//@}
//int eq_v3(vect_3* a, vect_3* b); //equals
//int zero_v3(vect_3* a, vect_3* b); //zero vector test

/*=======================================================================*/
/** @addtogroup matr_h SE(3) Optimized Matrix Functions
    @ingroup matr_n
    These functions are optimized to operate on the subset of SE(3) that applies 
    to physical systems. In particular, Rotation and Translation are supported, 
    skewing and reflecting are not.
 */
//@{
/** Homogeneous 4x4 matrix optimized for robots. 4th row is not used in calculations.
    \code
    [ n0 o0 a0 p0 ] where n = normal vector
    [ n1 o1 a1 p1 ]       o = orientation vector
    [ n2 o2 a2 p2 ]       a = approach vector
    [ 0  0  0  1  ]       p = position vector
    
    Can also be cast as a vector:
    (vect_n*)mh = <n0, o0, a0, p0, n1, o1, a1, p1, n2, o2, a2, p2, 0, 0, 0, 1>
    \endcode
*/
typedef struct barrett_matr_h{
  struct barrett_matr_h *ret;
  int s; // Total size of matrix (rows x columns)
  btreal *q;
  int m,n; //m rows, n cols
  btreal data[16];
}matr_h;

matr_h * new_mh();
BTINLINE void set_mh(matr_h* dest, matr_h* src);
BTINLINE void ident_mh(matr_h* dest); // identity matrix
BTINLINE void setrow_mh(matr_h* dest,int row, btreal s1, btreal s2, btreal s3, btreal s4);
void getcol_mh(vect_3* dest, matr_h* src, int n); //get the specified column
//void getpos_mh(vec_3* dest, matr_h* src); //get the last column
BTINLINE btreal getval_mh(matr_h* src, int row, int col);
//void setval_mh(matr_h* src, int row, int col, btreal val);
//void setrow_mh(matr_h* src, int row, btreal v1, btreal v2, btreal v3, btreal v4 );
//void getrot_mh(matr_n* dest, matr_h* src);
BTINLINE matr_h* mul_mh(matr_h* a,matr_h* b);
BTINLINE void setmul_mh(matr_h* a,matr_h* b); //multiply and store in a
BTINLINE vect_3* matXvec_mh(matr_h* a, vect_3* b);
void print_mh(matr_h* src);
int test_mh(btreal error);
//@}
/*=======================================================================*/
/** @addtogroup matr_3 so(3) Optimized Matrix Functions
    @ingroup matr_n
    
 */
//@{
/** 3x3 rotation matrix
   NOTE: matr_3 is actually a 4x4 "homogeneous" matrix:
     [ r11 r12 r13 x ]
     [ r21 r22 r23 y ]
     [ r31 r32 r33 z ]
     [  0   0   0  1 ]
   Many functions only operate on the inner 3x3 "rotation" matrix.
Function definitions are in btmath.h.
*/
typedef struct barrett_matr_h matr_3;

matr_3 * new_m3(); //allocates memory and sets to identity
BTINLINE void set_m3(matr_3* dest, matr_3* src);
BTINLINE void setrow_m3(matr_h* dest,int row, btreal s1, btreal s2, btreal s3);

BTINLINE void ident_m3(matr_3* dest); // identity matrix

//void zero_m3(matr_3* dest); // zero matrix
BTINLINE void getcol_m3(vect_3* dest, matr_3* src, int n); //get the specified column
BTINLINE btreal getval_m3(matr_3* src, int row, int col);

BTINLINE matr_3* mul_m3(matr_3* a,matr_3* b);
BTINLINE void setmul_m3(matr_3* a,matr_h* b); //multiply and store in a
BTINLINE matr_3* T_m3(matr_3* a); //transpose
BTINLINE vect_3* matXvec_m3(matr_3* a, vect_3* b);
BTINLINE vect_3* matTXvec_m3(matr_3* a, vect_3* b); //matXvec_m3(T_m3(a),b);

void print_m3(matr_3* src);
int test_m3(btreal error);
BTINLINE vect_3* RtoXYZf_m3(matr_3* R, vect_3* XYZ); //return XYZ
BTINLINE vect_3* RtoZYZf_m3(matr_3* R, vect_3* ZYZ); //return ZYZ
BTINLINE matr_3* XYZftoR_m3(matr_3* R, vect_3* XYZ); //Return R
BTINLINE vect_3* eqaxis_m3(matr_3* R, vect_3* a); //return axis of rotation with amount encoded in length
quat* R_to_q(quat* dest, matr_3* src);
matr_3* q_to_R(matr_3* dest, quat* src);
//vect_3* EUL_m3(matr_3* src); //return Euler angles
//vect_3* RPY_m3(matr_3* src); //return Roll pitch yaw
//@}
/*=======================================================================*/
/** @addtogroup matr_n General Matrix Functions
    @ingroup btmath
    
 */
//@{
/** general matrix.

Function definitions are in btmath.c.
*/
typedef struct barrett_matr_mn{
  struct barrett_matr_mn *ret;
  int s; // Total size of matrix (m x n)
  btreal *q;
  int m,n; //m rows, n cols
}matr_mn;

matr_mn * new_mn();
matr_mn * new_mn_ptr(matr_mn *a, int r, int c, int offset);
void destroy_mn(matr_mn **src);
BTINLINE void set_mn(matr_mn* dest, matr_mn* src);
BTINLINE matr_mn* ident_mn(matr_mn* dest); // identity matrix
BTINLINE void setrow_mn(matr_mn* dest, vect_n* src,int row);
BTINLINE void setcol_mn(matr_mn* dest, vect_n* src,int col);
void getcol_mn(vect_n* dest, matr_mn* src, int col); //get the specified column

BTINLINE btreal getval_mn(matr_mn* src, int row, int col);
BTINLINE void setval_mn(matr_mn* src, int row, int col, btreal val);

BTINLINE void setmul_mn(matr_mn* a,matr_mn* b); //multiply and store in a
BTINLINE vect_n* matXvec_mn(matr_mn* a, vect_n* b,vect_n* ret);
BTINLINE matr_mn* mul_mn(matr_mn* r, matr_mn* a, matr_mn* b);
BTINLINE matr_mn* T_mn(matr_mn* a);
void zero_mn(matr_mn* dest);
BTINLINE matr_mn* add_mn(matr_mn* r, matr_mn* a, matr_mn* b);
BTINLINE matr_mn* scale_mn(btreal x, matr_mn* a);
BTINLINE vect_3* matr_mnXvect_3(matr_mn* a, vect_3* b); // matr_mn must be 3x3!

/* Matrix inversion functions */
matr_mn* ludcmp(matr_mn *a, vect_n *indx, btreal *d);
vect_n* lubksb(matr_mn *a, vect_n *indx, vect_n *b);
matr_mn* inv_mn(matr_mn *a, vect_n *indx, vect_n *col);
btreal det_mn(matr_mn *a, vect_n *indx);

void print_mn(matr_mn* src);
char* sprint_mn(char *dest,matr_mn* src);
char* sprint_mh(char *dest,matr_h* src);
void strto_mn(matr_mn* dest,char *str);
int test_mn(btreal error);
//@}
/*=======================================================================*/
/** @addtogroup filt Digital Filters
    @ingroup btmath
    
 */
//@{
/** A digital filter object

btfilter is a digital filter object. new_btfilter() allocates memory for a new object.
init_btfilter_*() will create simple generic filters. eval_btfilter() is used for the
actual filter calculation. See new_btfilter() for more. Function definitions are in btmath.c.

*/
typedef struct
{
  int   order;             // FIRST_ORDER, SECOND_ORDER, OR FOURTH_ORDER
  int size;
  
  btreal  zeta;        // damping factor (between 0 and 1)
  
  int   index;         // index of current value in input and output arrays
  btreal  *d;  // coefficients of denominator
  btreal  *n;  // coefficients of numerator
  btreal  *x;  // old input values
  btreal  *y;  // old output values
    
} btfilter;
btfilter * new_btfilter(int size);
btreal  eval_btfilter(btfilter *filt, btreal xnew);
void syslog_filter(btfilter *filt);
void  init_btfilter_diff(btfilter *filt, int order, btreal sample_time, btreal cutoffHz);
void  init_btfilter_butterworth_diff(btfilter *filt, btreal sample_time, btreal cutoffHz);
void  init_btfilter_lowpass(btfilter *filt, btreal sample_time, btreal cutoffHz, btreal zeta);
void test_btfilter();

/* Define the Filter structure */
typedef struct
{
  int   order;             // FIRST_ORDER, SECOND_ORDER, OR FOURTH_ORDER
  int size;
  
  btreal  zeta;        // damping factor (between 0 and 1)
  
  int   index;         // index of current value in input and output arrays
  vectray  *d;  // coefficients of denominator
  vectray  *n;  // coefficients of numerator
  vectray  *x;  // old input values
  vectray  *y;  // old output values
  vect_n *scratch1,*scratch2,*scratch3;
    
} btfilter_vn;
btfilter_vn * new_btfilter_vn(int size,int vsize);
vect_n * eval_btfilter_vn(btfilter_vn *filt, vect_n *xnew);
//void  init_btfilter_vn_butterworth_diff(btfilter_vn *filt, int order, btreal sample_time, btreal cutoffHz);
void  init_btfilter_vn_diff(btfilter_vn *filt, int order, btreal sample_time, btreal cutoffHz);

void test_filter_vn();

//@}
/** @addtogroup gen_math General Math Functions
    @ingroup btmath
    
 */
//@{


BTINLINE btreal atan2_bt(btreal arg1, btreal arg2); 
BTINLINE btreal interp_bt(btreal x1, btreal y1, btreal x2, btreal y2, btreal x); //linear interpolation
BTINLINE btreal interpolate_bt(btreal x1, btreal y1, btreal x2, btreal y2, btreal x); //linear interpolation

BTINLINE btreal max_bt(btreal x,btreal y);
BTINLINE btreal min_bt(btreal x,btreal y);
#define Sgn(x) (x>=0.0?1.0:-1.0)
//@}
#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTMATH_H */

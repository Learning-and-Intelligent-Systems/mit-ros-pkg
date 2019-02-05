/* aob.h */
#ifndef _aob_h
#define _aob_h
#include "btmath.h"
   // AOB = Active Observer (Cortesao/Khatib)
   
   // sd = State dimension
   // S = State (sd x 1)
   // Fc = Closed loop system matrix (sd x sd)
   // Sp = Previous State (sd x 1)
   // Gama = Command matrix (sd x 1)
   // rk = Command reference (scalar)
   // rkp = Previous command reference (scalar)
   // Kk = Kalman gain, state uncertainty (sd x 1)
   // yk = Measured system output [Cpos,Cvel]
   // yhk = Model-estimated system output [Cpos,Cvel]
   // Ca = Measurement matrix (1 x sd)
   // Lc = DC gain compensation (scalar)
   // L_NAOB = State feedback gain (1 x sd, transposed)
   
typedef struct {
   int      sd; // State dimensions
   int      nout; // Number of yk outputs
   btreal   rkp; // Previous reference value
   matr_mn  *yk, *yhk; // Estimated output
   matr_mn  *Fc, *Fo, *Pzero, *Qnoise, *Gama, *Ca, *Kk, *L_NAOB, *Rnoise;
   matr_mn  *S; // State vector
   matr_mn  *Mscalar; // Result matrix used for scalar results
}AOB;

int new_aob(char *fn, AOB *aob, btreal *Lc, btreal *m, btreal *Ko, btreal *K2);

// Evaluate the AOB, given the reference value (rk) and measured system output (yk)
btreal eval_aob(AOB* aob, btreal rk, vect_n *yk);
      
#endif /* _aob_h */


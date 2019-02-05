/* ======================================================================== *
 *  Module ............. btdiag
 *  File ............... aob.h
 *  Creation Date ...... 14 Oct 2005
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


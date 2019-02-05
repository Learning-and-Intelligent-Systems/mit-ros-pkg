/* ======================================================================== *
 *  Module ............. btdiag
 *  File ............... extras.c
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

/* extras.c */

/* Extra code for optional inclusion in btdiag.c */

#ifdef AUDIO
/* Audio vars */
extern btreal extVel;
extern int audio;
extern int brake;
extern btreal setBrake;
extern btreal relBrake;
#endif

//vect_3 *f1;
//vect_n *jf;

//double wamdata[8];

// Flags
//int gravity = 0;


//int zDown = 0;
//int flip = 0;

//vectray *vr;

//extern int isZeroed;
//static RT_TASK *mainTask;

//double sample_rate;
//btreal Jpos_filt[7];
//btfilter *j5,*j6,*j7;
//btfilter *yk_xfilt, *yk_yfilt, *yk_zfilt;
//matr_mn *Th;

//int push = 0;
//btreal newt = 1;

//vect_n *wv;

#ifdef RUI
long lastPos[4];
btreal maxVel;
int whip = 0;
long temperature, ot, mt;

// AOB vars
matr_mn *Jtemp, *Ftemp;
vect_n *vn1, *vn2;
matr_mn *Js, *Ms;
vect_n *Gs;
vect_n *fstar;
vect_n *fstar_n;

btreal Lc_x, m_x, Ko_x, K2_x;
btreal Lc_y, m_y, Ko_y, K2_y;
btreal Lc_z, m_z, Ko_z, K2_z;
AOB aob_x, aob_y, aob_z;
vect_n *yk_x, *yk_y, *yk_z;

btreal rk_x, startPos_x, inputX = 0.0;
btreal rk_y, startPos_y, inputY = 0.0;
btreal rk_z, startPos_z, inputZ = 0.0;

vect_n *qVel, *qLast;
btreal fval_x, fval_y, fval_z;
btreal fsc = 1.0;

matr_mn *Jp, *Jptemp, *N, *Fptemp, *J_pseudo;
btreal Lc_xn, m_xn, Ko_xn, K2_xn;
btreal Lc_yn, m_yn, Ko_yn, K2_yn;
AOB aob_xn, aob_yn;
vect_n *yk_xn, *yk_yn;
btreal rk_xn, rk_yn;
vect_3 *p_null, *p_tool, *p_trocar, *p_init;
btreal dtr;
vect_n *torq_t, *torq_p;
matr_mn *J5, *aux, *J5temp, *Jaux2, *I7;

int null = 0;

int sd;
btreal Lc;
btreal mass;
btreal rkp_x, rk_x, startPos_x, inputX = 0.0;
btreal rkp_y, rk_y, startPos_y, inputY = 0.0;
btreal rkp_z, rk_z, startPos_z, inputZ = 0.0;
matr_mn *S_x, *S_y, *S_z;
matr_mn *yk_x, *yk_y, *yk_z;
matr_mn *yhk_x, *yhk_y, *yhk_z;
matr_mn *Fc, *Fo, *Pzero, *Qnoise, *Gama, *Ca, *Kk, *L_NAOB, *Rnoise;
vect_n *qVel, *qLast, *vLast;
matr_mn *Mscalar;
btreal Ko, K2;
btreal x_force, x_forcePrev, x_forceDeriv;
btreal y_force, y_forcePrev, y_forceDeriv;
btreal z_force, z_forcePrev, z_forceDeriv;
int nout;
btreal fsc = 1.0;
 
btreal rkp_x_f, rk_x_f;
btreal rkp_y_f, rk_y_f;
btreal rkp_z_f, rk_z_f;
matr_mn *S_x_f, *S_y_f, *S_z_f;
matr_mn *yk_x_f, *yk_y_f, *yk_z_f;
matr_mn *yhk_x_f, *yhk_y_f, *yhk_z_f;
matr_mn *Fc_f, *Fo_f, *Pzero_f, *Qnoise_f, *Gama_f, *Ca_f, *Kk_f, *L_NAOB_f, *Rnoise_f;
 
btreal fval_x, fval_y, fval_z;
btreal xa, ya, za, xaf, yaf, zaf;
int force = 0;

#endif

main(){
   #ifdef RUI
   //f1 = new_v3();
   //jf = new_vn(4);

   nout = new_aob("ImpedanceBarrett.txt", &aob_x, &Lc_x, &m_x, &Ko_x, &K2_x);
   yk_x = new_vn(nout);

   nout = new_aob("ImpedanceBarrett.txt", &aob_y, &Lc_y, &m_y, &Ko_y, &K2_y);
   yk_y = new_vn(nout);

   nout = new_aob("ImpedanceBarrett.txt", &aob_z, &Lc_z, &m_z, &Ko_z, &K2_z);
   yk_z = new_vn(nout);

   nout = new_aob("NullSpaceBarrett.txt", &aob_xn, &Lc_xn, &m_xn, &Ko_xn, &K2_xn);
   yk_xn = new_vn(nout);

   nout = new_aob("NullSpaceBarrett.txt", &aob_yn, &Lc_yn, &m_yn, &Ko_yn, &K2_yn);
   yk_yn = new_vn(nout);
#endif

#if 0
   //usleep(3000000);
   yk_xfilt = new_btfilter(5);
   yk_yfilt = new_btfilter(5);
   yk_zfilt = new_btfilter(5);
   init_btfilter_lowpass(yk_xfilt,0.002,50,0.8);
   init_btfilter_lowpass(yk_yfilt,0.002,50,0.8);
   init_btfilter_lowpass(yk_zfilt,0.002,50,0.8);

   j5 = new_btfilter(5);
   j6 = new_btfilter(5);
   j7 = new_btfilter(5);
   init_btfilter_lowpass(j5,0.002,30,0.8);
   init_btfilter_lowpass(j6,0.002,10,0.8);
   init_btfilter_lowpass(j7,0.002,1,0.8);
   syslog_filter(j5);
   syslog_filter(j6);
   syslog_filter(j7);
#endif

// After OpenWAM()

#ifdef RUI
   //wam->Cvel = new_vn(3);
   Jtemp = new_mn(3,wam->dof);
   qVel = new_vn(wam->dof);
   qLast =  new_vn(wam->dof);
   //vLast = new_vn(3);
   Gs = new_vn(wam->dof);
   Ms = new_mn(wam->dof,wam->dof);


   Ftemp = new_mn(3,3);
   //Ftemp = new_mn(5,5);
   //Ftemp = new_mn(2,2);
   vn1 = new_vn(wam->dof);
   vn2 = new_vn(wam->dof);
   //fstar = new_vn(5);
   fstar = new_vn(3);
   aux = new_mn(wam->dof,wam->dof);
   //fstar = new_vn(2);
   //Js = new_mn(3,wam->dof);

   Th = new_mn(4,4);

   Jp = new_mn(2,wam->dof);
   Jptemp = new_mn(2,wam->dof);
   N = new_mn(wam->dof,wam->dof);
   Fptemp = new_mn(2,2);
   J_pseudo = new_mn(3,wam->dof);
   fstar_n = new_vn(2);
   p_null = new_v3();
   p_tool = wam->Cpos;
   p_init = new_v3();
   p_trocar = new_v3();
   torq_t = new_vn(wam->dof);
   torq_p = new_vn(wam->dof);

   J5 = new_mn_ptr(wam->robot.J, 3, wam->dof, 0);
   J5temp = new_mn(3,wam->dof);
   Jaux2 = new_mn(2,wam->dof);
   I7 = new_mn(wam->dof,wam->dof);
   ident_mn(I7);
#endif

#if 0
   /* Check and handle any additional command line arguments */
   for(i = 1; i < argc; i++) {
      if(!strcmp(argv[i],"-g")) // If gimbals are being used
      {
         initGimbals(wam);
         useGimbals = TRUE;
         syslog(LOG_ERR, "Gimbals expected.");
      }
   }
#endif

#if 0
   wam->logdivider = 1;
   PrepDL(&(wam->log),35);
   AddDataDL(&(wam->log),&(wam->log_time),sizeof(double),2,"Time");
   AddDataDL(&(wam->log),valptr_vn(fstar),sizeof(btreal)*3,BTLOG_BTREAL,"Fstar");
   AddDataDL(&(wam->log),&(aob_x.S->q[0]),sizeof(btreal)*4,BTLOG_BTREAL,"ControlAOB_State_x");
   AddDataDL(&(wam->log),&(aob_y.S->q[0]),sizeof(btreal)*4,BTLOG_BTREAL,"ControlAOB_State_y");
   AddDataDL(&(wam->log),&(aob_z.S->q[0]),sizeof(btreal)*4,BTLOG_BTREAL,"ControlAOB_State_z");
   //AddDataDL(&(wam->log),&(S_x_f->q[0]),sizeof(btreal)*4,BTLOG_BTREAL,"SensorAOB_State");
   AddDataDL(&(wam->log),valptr_vn((vect_n*)wam->Cpos),sizeof(btreal)*3,BTLOG_BTREAL,"Cpos");
   //AddDataDL(&(wam->log),valptr_vn((vect_n*)wam->Cvel),sizeof(btreal)*3,BTLOG_BTREAL,"Cvel");
   //AddDataDL(&(wam->log),&x_force,sizeof(btreal),BTLOG_BTREAL,"x_force");
   //AddDataDL(&(wam->log),&x_forceDeriv,sizeof(btreal),BTLOG_BTREAL,"x_forceDeriv");
   //AddDataDL(&(wam->log),&x_vel,sizeof(btreal),BTLOG_BTREAL,"Xvel");
   AddDataDL(&(wam->log),valptr_vn(wam->Jpos),sizeof(btreal)*7,BTLOG_BTREAL,"Jpos");
   InitDL(&(wam->log),1000,"datafile.dat");
#endif
}

int WAMcallback(struct btwam_struct *w)
{
   int i;
   
#if 0
   btreal rho;
   int cnt;
   //calcMatrix(wam->dof, wam->Jpos, Ms, Gs, Js);

   // qVel = (wam->Jpos - qLast) / Ts;
   set_vn(qVel, scale_vn(1.0/Ts, add_vn(wam->Jpos, scale_vn(-1.0, qLast))));

   // wam->Cvel = Js_v * qVel;
   matXvec_mn(wam->robot.Jv, qVel, wam->Cvel);

   // Get the location of the frame where the tool is attached in world coordinates
   set_v3(p_init, Ln_to_W_bot(&wam->robot, wam->dof-1, const_v3(p_init, 0.0, 0.0, 0.0)));

   // Find the point (p_null) on the tool shaft closest to the trocar (hole)
   rho = -dot_v3(add_v3(p_tool, neg_v3(p_trocar)), add_v3(p_init, neg_v3(p_tool)))
         / (D_Pt2Pt(p_init, p_tool) * D_Pt2Pt(p_init, p_tool));
   set_v3(p_null, add_v3(p_tool, scale_v3(rho, add_v3(p_init, neg_v3(p_tool)))));
   //set_v3(p_null, p_tool);

   // Find the shortest distance between the trocar (hole)
   // and the null space control point on the tool shaft
   dtr = D_Pt2Pt(p_null, p_trocar);

   // Find the Jacobian at p_null
   for (cnt = 0;cnt < wam->robot.num_links;cnt++)
   {
      setcol_mn(Jp, (vect_n*)cross_v3(wam->robot.links[cnt-1].z,sub_v3(p_null, wam->robot.links[cnt-1].o)), cnt);
   }

   // Find task space outputs
   yk_x->q[0] = Ko_x * (wam->Cpos->q[0] - startPos_x);
   yk_y->q[0] = Ko_y * (wam->Cpos->q[1] - startPos_y);
   yk_z->q[0] = Ko_z * (wam->Cpos->q[2] - startPos_z);

   // Find null space outputs
   yk_xn->q[0] =  (p_null->q[0] - p_trocar->q[0]);
   yk_yn->q[0] =  (p_null->q[1] - p_trocar->q[1]);

   // Calc derivative terms for task space outputs
   yk_x->q[1] = (yk_x->q[0] - yk_x->ret->q[0]) / Ts;
   yk_y->q[1] = (yk_y->q[0] - yk_y->ret->q[0]) / Ts;
   yk_z->q[1] = (yk_z->q[0] - yk_z->ret->q[0]) / Ts;

   // Calc derivative terms for null space outputs
   yk_xn->q[1] = (yk_xn->q[0] - yk_xn->ret->q[0]) / Ts;
   yk_yn->q[1] = (yk_yn->q[0] - yk_yn->ret->q[0]) / Ts;

   if(force)
   {
      // ***************************************************
      // Control
      // ***************************************************

      // Calc x,y,z control force for task space
      fstar->q[0] = rk_x - eval_aob(&aob_x, rk_x, yk_x) - (K2_x * wam->Cvel->q[0]);
      fstar->q[1] = rk_y - eval_aob(&aob_y, rk_y, yk_y) - (K2_y * wam->Cvel->q[1]);
      fstar->q[2] = rk_z - eval_aob(&aob_z, rk_z, yk_z) - (K2_z * wam->Cvel->q[2]);
      //fstar->q[2] = 0.0;
      //fstar->q[3] = 0.0;
      //fstar->q[4] = 0.0;
      const_vn(fstar, 10.0, 0.0, 0.0);

      // Calc x,y control force for null space
      fstar_n->q[0] = rk_xn - eval_aob(&aob_xn, rk_xn, yk_xn);
      fstar_n->q[1] = rk_yn - eval_aob(&aob_yn, rk_yn, yk_yn);
      //const_vn(fstar_n, 0.0, 0.0);
   } else
   {
      const_vn(fstar, 0.0, 0.0, 0.0);
      //const_vn(fstar, 0.0, 0.0, 0.0, 0.0, 0.0);
      const_vn(fstar_n, 0.0, 0.0);
   }

   //const_vn(wam->Jtrq, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   // Get joint torque due to task space control
   // Jtorq = T_mn(J) * (inv_mn(J * inv_mn(M) * T_mn(J)) * fstar)
/*   
   inv_mn(wam->robot.M, vn1, vn2); // A->ret <= inv(A)
   mul_mn(Jtemp, wam->robot.Jv, wam->robot.M->ret); // A <= B * C
   T_mn(wam->robot.Jv); // A->ret <= Trans(A)
   mul_mn(Ftemp, Jtemp, wam->robot.Jv->ret); // A <= B * C
   inv_mn(Ftemp, vn1, vn2); // A->ret <= inv(A)
   matXvec_mn(Ftemp->ret, fstar, fstar->ret); // A * B => C
   matXvec_mn(wam->robot.Jv->ret, fstar->ret, torq_t);  // A * B => C
*/ 
   inv_mn(wam->robot.M, vn1, vn2); // A->ret <= inv(A)
   mul_mn(Jtemp, J5, wam->robot.M->ret); // A <= B * C
   T_mn(J5); // A->ret <= Trans(A)
   mul_mn(Ftemp, Jtemp, J5->ret); // A <= B * C
   inv_mn(Ftemp, vn1, vn2); // A->ret <= inv(A)
   matXvec_mn(Ftemp->ret, fstar, fstar->ret); // A * B => C
   matXvec_mn(J5->ret, fstar->ret, torq_t);  // A * B => C
 /*  
   inv_mn(wam->robot.M, vn1, vn2); // A->ret <= inv(A)
   mul_mn(Jptemp, Jp, wam->robot.M->ret); // A <= B * C
   T_mn(Jp); // A->ret <= Trans(A)
   mul_mn(Ftemp, Jptemp, Jp->ret); // A <= B * C
   inv_mn(Ftemp, vn1, vn2); // A->ret <= inv(A)
   matXvec_mn(Ftemp->ret, fstar, fstar->ret); // A * B => C
   matXvec_mn(Jp->ret, fstar->ret, torq_t);  // A * B => C
 */ 

   set_vn(wam->Jtrq, add_vn(wam->Jtrq, torq_t)); // Add the task torque to the Joint Torque

   if(null)
   {
      // Get joint torque due to null space control
      // J_pseudo = inv(M) * T(Jv) * inv(Jv * inv(M) * T(Jv));
      // J_pseudo(dofx3) = wam->robot.M->ret(dofxdof) * wam->robot.Jv->ret(dofx3) * Ftemp->ret(3x3)
      
      // J5(5xdof) 
      // J5->ret(dofx5)
      // Ftemp->ret(5x5) (Lambda_t)
      // J5temp(dofx5)
      // wam->robot.M->ret(dofxdof) (Mass Inverse)
      // J_pseudo(dofx5)
      // J_pseudo->ret(5xdof)
      // aux(dofxdof)
      // I7(dofxdof)
      // aux->ret(dofxdof)
      // N(dofxdof)
      // Jp(2xdof)
      // Jptemp(2xdof)
      // Jp->ret(dofx2)
      // Fptemp(2x2)
      // Fptemp->ret(2x2) (Lambda_p)
      // N->ret(dofxdof)
      // Jptemp->ret(dofx2)
      // fstar_n(2x1)
      // fstar_n->ret(2x1)
      // torq_p(dofx1)
      
      mul_mn(J5temp, J5->ret, Ftemp->ret);
      mul_mn(J_pseudo, wam->robot.M->ret, J5temp);
      // N = ident(dof) - (T(Jv) * T(J_pseudo));
      // N(dofxdof) = ident(dof) - (wam->robot.Jv->ret(dofx3) * T(J_pseudo)(3xdof))
      T_mn(J_pseudo);
      mul_mn(aux, J5->ret, J_pseudo->ret);
      scale_mn(-1.0, aux);
      
      add_mn(N, I7, aux->ret);
      //set_mn(N, T_mn(N));
      //mul_mn(J5, J_pseudo->ret, T_mn(N));
      //ident_mn(N); //Test only
      // T_posture = T(Jp * N) * inv(Jp * inv(M) * T(N) * T(Jp)) * fstar_n;
      // T_posture = (dofx2) * inv(Jp(2xdof) * inv(M)(dofxdof) * T(N)(dofxdof) * T(Jp)(dofx2))(2x2) * fstar_n(2x1);
      mul_mn(Jptemp, Jp, wam->robot.M->ret); // A <= B * C
      //mul_mn(Jaux2, Jptemp, N); // A <= B * C
      T_mn(Jp);
      mul_mn(Fptemp, Jptemp, Jp->ret); // A <= B * C
      inv_mn(Fptemp, vn1, vn2); // A->ret <= inv(A)
      //T_mn(N);
      //mul_mn(Jptemp, Jp, N->ret);
      T_mn(Jp);
      mul_mn(Jptemp->ret, N, Jp->ret);
      //mul_mn(Jptemp, Jptemp->ret, Fptemp->ret);
      mul_mn(Jptemp, Jptemp->ret, Fptemp->ret); // A * B => C
      matXvec_mn(Jptemp, fstar_n, torq_p);

      set_vn(wam->Jtrq, add_vn(wam->Jtrq, torq_p)); // Add the null torque to the Joint Torque
   }

   // Save the present outputs for next time
   set_vn(qLast, wam->Jpos);
   set_vn(yk_x->ret, yk_x);
   set_vn(yk_y->ret, yk_y);
   set_vn(yk_z->ret, yk_z);
   set_vn(yk_xn->ret, yk_xn);
   set_vn(yk_yn->ret, yk_yn);
#endif
}

   //brake = 0;
   //setBrake = 0.15;
   //relBrake = 0.0;
   //init_pl_btg(&planes[0], const_v3(p1, 0.7, 0.0, 0.0), const_v3(p1, 0.7, 0.0, 0.1), const_v3(p1, 0.7, 0.1, 0.1));
   //init_normal_plane_bth(&objects[objectCount],&planes[0],(void*)&bpwall[0],bulletproofwall_nf);
   //init_wickedwall(&wickedwalls[0],500.0, 2.0,0.5,0.1,0.01);
   //init_normal_sphere_bth(&objects[objectCount],&spheres[0],(void*)&wickedwalls[0],wickedwall_nf);
   //addobject_bth(&bth,&objects[objectCount++]);
   
   
#ifdef AUDIO
/** Spins in a loop, handles audio cues.
    Runs as its own thread, handles audio cues.
*/
void AudioThread()
{
   int volume;
   char volStr[64];
   char *aplay[4], *amixer[6];
   char mixerVol[64];
   int status;
   pid_t pID;
   
   aplay[0] = "aplay";
   aplay[1] = "-q";
   aplay[2] = "hitSounds/hit1.wav";
   aplay[3] = NULL;
   
   amixer[0] = "amixer";
   amixer[1] = "-q";
   amixer[2] = "cset";
   amixer[3] = "numid=2";
   amixer[4] = mixerVol;
   amixer[5] = NULL;
   
   while (!done) {
      switch(brake){
         case 0:
         brake = 1;
         // Release the brake
         serialWriteString(&p, "set torq 0\r");
         break;
         case 2:
         brake = 3;
         // Activate the brake
         serialWriteString(&p, "set torq 10\r");
         break;
         case 4:
         brake = 5;
         // Release the brake
         serialWriteString(&p, "set torq 0\r");
         break;
         default:
         break;
      }
      
      if(audio == 2){
         audio = 0;
         volume = abs((int)(extVel * 100));
         if(volume > 100)
            volume = 100;
         syslog(LOG_ERR, "Volume = %d", volume);
         sprintf(mixerVol, "%d%%", volume);
         if(!(pID=vfork())){
            execvp(amixer[0], amixer);
            exit(0);
         }
         //waitpid (pID, &status, 0);
         if(!vfork()){
            execvp(aplay[0], aplay);
            exit(0);
         }
         
         
         /*
         pid_t pID = vfork();
         if (pID == 0)                // child
         {
            // Code only executed by child process
            //syslog(LOG_ERR, "Child about to aplay");
            execvp(argv[0], argv);
            exit(0);
            //sIdentifier = "Child Process: ";
            //globalVariable++;
            //iStackVariable++;
          }
          else if (pID < 0)            // failed to fork
          {
              syslog(LOG_ERR, "Failed to fork()");
              exit(1);
              // Throw exception
          }
          else                                   // parent
          {
            // Code only executed by parent process
            //waitpid (pID, &status, 0);
            //sIdentifier = "Parent Process:";
          }
         */
         /*
         if (!fork()){
            //sprintf(volStr, "cset iface=MIXER,name='PCM Playback Volume' %d%%", volume);
            //execvp("amixer", volStr);
            execvp("aplay", "SOUND34.WAV");
            exit(0);
         }
         */
            
      }
      usleep(10000);
   }
}
#endif


   /*
   wam->Jvel->q[0] = (wam->act[0].puck.position - lastPos[0]) * 10 * 60.0 / 4096;
   wam->Jvel->q[1] = (wam->act[1].puck.position - lastPos[1]) * 10 * 60.0 / 4096;
   wam->Jvel->q[2] = (wam->act[2].puck.position - lastPos[2]) * 10 * 60.0 / 4096;
   wam->Jvel->q[3] = (wam->act[3].puck.position - lastPos[3]) * 10 * 60.0 / 4096;
   
   if(fabs(wam->Jvel->q[0]) > maxVel)
      maxVel = fabs(wam->Jvel->q[0]);
   
   //wam->Jvel->q[0] = (wam->Jpos->q[0] - lastPos[0]) / 0.1 * 60 / 6.28;
   //wam->Jvel->q[1] = (wam->Jpos->q[1] - lastPos[1]) / 0.1 * 60 / 6.28;
   //wam->Jvel->q[2] = (wam->Jpos->q[2] - lastPos[2]) / 0.1 * 60 / 6.28;
   //wam->Jvel->q[3] = (wam->Jpos->q[3] - lastPos[3]) / 0.1 * 60 / 6.28;
   
   lastPos[0] = wam->act[0].puck.position;
   lastPos[1] = wam->act[1].puck.position;
   lastPos[2] = wam->act[2].puck.position;
   lastPos[3] = wam->act[3].puck.position;
   */
   
   /*
      
   mvprintw(line, 0, "pauseCnt:");
   line++;
   mvprintw(line, 0, "%d     ", pauseCnt);
   line+= 1;
   mvprintw(line, 0, "Gtrq:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)wam->Gtrq));
   line+= 1;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)wam->torq_limit));
   line+= 1;
   */
/*
   for(cnt=0;cnt<=wam->dof;cnt++){
	   mvprintw(line, 0, "g[%d] = %+8.4f    ", cnt,
			   getval_v3(wam->robot.links[cnt].g,2));
	   line++;
   }
   */
 /* 
      mvprintw(line, 0, "Jvel:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)wam->Jvel));
   line+= 1;
   
      mvprintw(line, 0, "maxVel:");
   line++;
   mvprintw(line, 0, "%+8.4f    ", maxVel);
   line+= 1;
         mvprintw(line, 0, "temperature:");
   line++;
   mvprintw(line, 0, "%ld     ", temperature);
   line+= 1;
            mvprintw(line, 0, "peak t:");
   line++;
   mvprintw(line, 0, "%ld     ", ot);
   line+= 1;
            mvprintw(line, 0, "mt:");
   line++;
   mvprintw(line, 0, "%ld     ", mt);
   line+= 1;
   */
/*
   mvprintw(line, 0, "p_tool:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)p_tool));
   line+= 1;

   mvprintw(line, 0, "p_trocar:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)p_trocar));
   line+= 1;

   mvprintw(line, 0, "p_init:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)p_init));
   line+= 1;

   mvprintw(line, 0, "p_null:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)p_null));
   line+= 1;

   mvprintw(line, 0, "dtr:");
   line++;
   mvprintw(line, 0, "%+8.4f", dtr);
   line+= 1;
   

   mvprintw(line, 0, "Jacobian @ p_null:");
   line++;
   mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Jp));
   line+= 3;

   mvprintw(line, 0, "lam_p * fp:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)fstar_n->ret));
   line+= 1;
   mvprintw(line, 0, "lam_t * ft:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)fstar->ret));
   line+= 1;


   mvprintw(line, 0, "fstar_t:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)fstar));
   line+= 1;

   mvprintw(line, 0, "fstar_n:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)fstar_n));
   line+= 1;

   mvprintw(line, 0, "tau_task:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, torq_t));
   line+= 1;

   mvprintw(line, 0, "tau_null:");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, torq_p));
   line+= 1;

   mvprintw(line, 0, "LNAOB:");
   line++;
   mvprintw(line, 0, "%s", sprint_mn(vect_buf1, aob_xn.L_NAOB));
   line+= 1;

   mvprintw(line, 0, "yk_x");
   line++;
   mvprintw(line, 0, "%s", sprint_vn(vect_buf1, yk_x));
   line+= 1;

   mvprintw(line, 0, "State:x");
   line++;
   mvprintw(line, 0, "%s", sprint_mn(vect_buf1, aob_x.S));
   line+= 4;

      mvprintw(line, 0, "L_NAOB * S:");
      line++;
      mvprintw(line, 0, "%+8.4f", aob_xn.Mscalar->q[0]);
      line+= 1;
   

      mvprintw(line, 0, "Lambda_task:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Ftemp->ret));
      line+= 5;
   
      mvprintw(line, 0, "Lambda_null:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Fptemp->ret));
      line+= 2;
      
      mvprintw(line, 0, "J5 * Jpseudo:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, aux));
      line+= 7;
      
      mvprintw(line, 0, "Jacobian Matrix:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, J5->ret));
      line+= 7;
      
      mvprintw(line, 0, "Jacobian Pseudo:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, J_pseudo->ret));
      line+= 7;

      mvprintw(line, 0, "J_pseudoT * NT:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, J5));
      line+= 3;

      mvprintw(line, 0, "N:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, N));
      line+= 7;

      mvprintw(line, 0, "J_pseudo:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, J_pseudo));
      line+= 3;
    

      mvprintw(line, 0, "Jacobian Matrix (s):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Js));
      line+= 4;
    
      mvprintw(line, 0, "Jacobian Matrix (com):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.links[3].Jcom));
      line+= 7;
    
      mvprintw(line, 0, "Jacobian Matrix (vcom):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.Jvcom));
      line+= 4;
    
      mvprintw(line, 0, "Jacobian Matrix (w):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.Jw));
      line+= 4;
    
      mvprintw(line, 0, "fval_z:");
      line++;
      mvprintw(line, 0, "%+8.4f", fval_z);
      line+= 1;
    
      mvprintw(line, 0, "nout:");
      line++;
      mvprintw(line, 0, "%d", nout);
      line+= 1;
    
      mvprintw(line, 0, "mass:");
      line++;
      mvprintw(line, 0, "%+8.4f", mass);
      line+= 1;
    
      mvprintw(line, 0, "yk_z:");
      line++;
      mvprintw(line, 0, "%+8.4f", yk_z->q[0]);
      line+= 1;
    
        for(cnt = 0; cnt < wam->dof; cnt++){
              mvprintw(line, 0, "I(%d):", cnt+1);
              line++;
              mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.links[cnt].I));
              line+= 3;
              mvprintw(line, 0, "%s", sprint_vn(vect_buf1, (vect_n*)wam->robot.links[cnt].rotorI));
              line++;
        }
      
      mvprintw(line, 0, "Ro(4):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.links[3].Ro));
      line+= 4;
    
    
      mvprintw(line, 0, "Th (s):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Th));
      line +=5;
    
      mvprintw(line, 0, "Th:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.links[3].Ro));
      line +=5;
      
      mvprintw(line, 0, "Mass Matrix:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, wam->robot.M->ret));
      line +=7;
       
      mvprintw(line, 0, "Mass Matrix (s):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Ms));
      line +=7;
      
      mvprintw(line, 0, "Mass Matrix (diff):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, add_mn(wam->robot.M->ret, wam->robot.M, scale_mn(-1.0, Ms))));
      line +=7;
     
      mvprintw(line, 0, "Kk_f:");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, Kk_f));
      line +=5;
      
      //mvprintw(line, 0, "yk_x:");
      //line++;
      //mvprintw(line, 0, "%s", sprint_mn(vect_buf1, yk_x));
      //line+= 4;
      
      mvprintw(line, 0, "vLast:");
      line++;
      mvprintw(line, 0, "%s", sprint_vn(vect_buf1, vLast));
      line+= 1;
    
      mvprintw(line, 0, "State Vector (S_x_f):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, S_x_f));
      line+= 5;
     
      mvprintw(line, 0, "State Vector (S_x):");
      line++;
      mvprintw(line, 0, "%s", sprint_mn(vect_buf1, aob_x.S));
      line+= 5;
     
      //mvprintw(line, 0, "L_NAOB*S (z):");
      //line++;
      //mvprintw(line, 0, "%+8.4f", Mscalar->q[0]);
      //line+= 1;
      
      mvprintw(line, 0, "fstar:");
      line++;
      mvprintw(line, 0, "%s", sprint_vn(vect_buf1, fstar));
      line+= 1;
    
      mvprintw(line, 0, "Cvel:");
      line++;
      mvprintw(line,0,"%s",sprint_vn(vect_buf1,wam->Cvel));
      line++;
    
    
      mvprintw(line, 0, "Gravity torques (symb):");
      line++;
      mvprintw(line,0,"%s",sprint_vn(vect_buf1,Gs));
      line++;
   
   line++;
   mvprintw(line, 0, "Callback time (ms):");
   line++;
   mvprintw(line,0,"%.4f",wam->user_time/1000000.0);
   line+=2;
   
   mvprintw(line,0,"%s",sprint_vn(vect_buf1,jf));

   mvprintw(line, 0, "O Matrix:");
   line++;
   mvprintw(line, 0, "%s", sprint_v3(vect_buf1, wam->robot.links[3].o));

   line += 2;
   mvprintw(line, 0, "Z Matrix:");
   line++;
   mvprintw(line, 0, "%s", sprint_v3(vect_buf1, wam->robot.links[3].z));

   */
   /*  line += 1;
     mvprintw(line,0,"bts: state:%d",active_bts->mode);
     mvprintw(line,20,"trj: state:%d",active_bts->btt.state);
     line += 1;
   */
   
   
   /*
   case '1': //Turn off puck
	   //setProperty(0, 2, MODE, FALSE, MODE_IDLE);
	   //setProperty(0, 3, MODE, FALSE, MODE_IDLE);
      getProperty(0, 1, TEMP, &temperature);
      getProperty(0, 1, PTEMP, &ot);
      getProperty(0, 1, MT, &mt);
	   break;
   case '2': //Whip
   maxVel = 0.0;   
   whip = !whip;
      
      break;
*/      

 /*  case 'z'://Send home-position to WAM
      const_vn(wv, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //gimbals
      DefineWAMpos(wam,wv);
      break;
  */ 
  
  #ifdef RUI
   case '[':
      fsc *= 0.5;
      break;
   case ']':
      fsc *= 2.0;
      break;
   case 't':// Set trocar
      set_v3(p_trocar, wam->Cpos);
      null = 1;
      break;
   case 'z':// Toggle rk
      //if(inputZ < 0.001)
      //   inputZ = -2.5;
      //else
      //   inputZ = 0.0;
      inputZ += -1.0;

      rk_z = Lc_z * inputZ;

      break;
   case 'r':// Toggle rk
      //if(inputX < 0.001)
      //   inputX = 5.0;
      //else
      //   inputX = 0.0;
      inputX += 2.5;

      rk_x = Lc_x * inputX;

      break;
   case 'f'://Toggle force
      //if(!force)
      //   DLon(&(wam->log));
      //else
      //   DLoff(&(wam->log));

      //usleep(2000);

      // Set rk = rkp = x0
      rk_x = rk_y = rk_z = 0.0;
      rk_xn = rk_yn = 0.0;

      //rk_x_f = rkp_x_f = 0.0;
      //rk_y_f = rkp_y_f = 0.0;
      //rk_z_f = rkp_z_f = 0.0;

      // Set S->q[0] = rk
      zero_mn(aob_x.S);
      zero_mn(aob_y.S);
      zero_mn(aob_z.S);
      zero_mn(aob_xn.S);
      zero_mn(aob_yn.S);
      //zero_mn(S_y);
      //zero_mn(S_z);

      //zero_mn(S_x_f);
      //zero_mn(S_y_f);
      //zero_mn(S_z_f);

      startPos_x = wam->Cpos->q[0];
      startPos_y = wam->Cpos->q[1];
      startPos_z = wam->Cpos->q[2];

      yk_x->ret->q[0] = 0.0;
      yk_y->ret->q[0] = 0.0;
      yk_z->ret->q[0] = 0.0;
      yk_xn->ret->q[0] = 0.0;
      yk_yn->ret->q[0] = 0.0;

      /*
            x_forcePrev = 0.0;
            y_forcePrev = 0.0;
            z_forcePrev = 0.0;
            
            xaf = yaf = zaf = 0.0;
      */
      fval_x = fval_y = fval_z = 0.0;

      fsc = 1.0;

      force = !force;
      break;
#endif

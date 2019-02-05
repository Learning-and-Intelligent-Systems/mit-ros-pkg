//test Manfred's Barrett Arm IK function

#include "WAMKinematics.h"
#include "wamik.h"

#define SQR(x) ((x)*(x))


int main()
{
  int err, errcnt = 0, cnt = 0;
  double angles[JNTS], resangles[JNTS], angles_ref[JNTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double theta2 = 0.0;
  double tool[4][4], tool2[4][4];
  double angstep = 0.3;
  double angerr, totangerr = 0.0;
  double errmag, toterrmag = 0.0;

  angles[0] = 0.0;//wam_theta_min[0]+0.001;
  while (angles[0] < wam_theta_max[0]) {
    angles[1] = 0.0;//wam_theta_min[1]+0.001;
    while (angles[1] < wam_theta_max[1]) {
      angles[2] = wam_theta_min[2]+0.001;
      while (angles[2] < wam_theta_max[2]) {
				angles[3] = 1.0;//wam_theta_min[3]+0.001;
				while (angles[3] < wam_theta_max[3]) {
					angles[4] = wam_theta_min[4]+0.001;
					while (angles[4] < wam_theta_max[4]) {
						angles[5] = 0.0;//wam_theta_min[5]+0.001;
						while (angles[5] < wam_theta_max[5]) {
							angles[6] = 0.0;//wam_theta_min[6]+0.001;
							//	      while (angles[6] < wam_theta_max[6]) {
							WAMForwardKinematics(angles, tool, NULL);
							/*
							for(theta2=wam_theta_min[2]+0.001; theta2<wam_theta_max[2]; theta2+=.02){
								err = WAMInverseKinematics(theta2, tool, angles, resangles);
								WAMForwardKinematics(resangles, tool2, NULL);
								errmag = SQR(tool[0][3] - tool2[0][3]) + SQR(tool[1][3] - tool2[1][3]) + SQR(tool[2][3] - tool2[2][3]);
								if(err > 0 && errmag <= 0.001){
									break;
								}
								}*/
							
							double rot4[16];
							int i, j;
							for(i=0; i<4; i++){
								for(j=0; j<4; j++){
									rot4[i*4+j] = tool[i][j];
								}
							}
							err = wam_ik(rot4, angles_ref, resangles);
							WAMForwardKinematics(resangles, tool2, NULL);
							errmag = SQR(tool[0][3] - tool2[0][3]) + SQR(tool[1][3] - tool2[1][3]) + SQR(tool[2][3] - tool2[2][3]);								
							
							printf("   angles %.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t-> %.3f\t%.3f\t%.3f\n", angles[0], angles[1], angles[2], 
										 angles[3], angles[4], angles[5], angles[6], tool[0][3], tool[1][3], tool[2][3]);
							printf("resangles %.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t-> %.3f\t%.3f\t%.3f\n", resangles[0], resangles[1], 
										 resangles[2], resangles[3], resangles[4], resangles[5], resangles[6], tool2[0][3], 
										 tool2[1][3], tool2[2][3]);
							angerr = SQR(resangles[0] - angles[0]) + SQR(resangles[1] - angles[1]) + SQR(resangles[2] - angles[2]) + 
								SQR(resangles[3] - angles[3]) + SQR(resangles[4] - angles[4]) + SQR(resangles[5] - angles[5]) + 
								SQR(resangles[6] - angles[6]);
							totangerr += angerr;
							toterrmag += errmag;
							cnt++;
							if (err <= 0) {
								errcnt++;
								printf("No Solution found %d of %d:", errcnt, cnt);
								printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f -> %.3f %.3f %.3f\n", angles[0], angles[1], 
											 angles[2], angles[3], angles[4], angles[5], angles[6], tool[0][3], tool[1][3], tool[2][3]);
							}
							else {
								if (errmag > 0.001) {
									printf("Incorrect solution:");
									printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f -> %.3f %.3f %.3f\n", angles[0], angles[1], angles[2], 
												 angles[3], angles[4], angles[5], angles[6], tool[0][3], tool[1][3], tool[2][3]);
									printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f -> %.3f %.3f %.3f\n", resangles[0], resangles[1], 
												 resangles[2], resangles[3], resangles[4], resangles[5], resangles[6], tool2[0][3], 
												 tool2[1][3], tool2[2][3]);
								}
								//		}
								angles[6] += angstep;
							}
							angles[5] += angstep;
						}		
						angles[4] += angstep;
					}		
					angles[3] += angstep;
				}		
				angles[2] += angstep;
      }		
      angles[1] += angstep;
      printf("Increasing 1 Err: %lf %lf\n", toterrmag, totangerr);
    }		
    angles[0] += angstep;
    printf("Increasing 0 Err: %lf %lf\n", toterrmag, totangerr);
  }
	printf("cnt: %d, errcnt: %d, percent correct: %f\n", cnt, errcnt, 1.0-(float)errcnt/cnt); 
  return(1);
}

//Run Manfred's inverse kinematics function while searching over theta2

#include "WAMKinematics.h"

#define SQ(x) ((x)*(x))

//vect mag
double vect_mag(double vect[3]){
	return sqrt(SQ(vect[0])+SQ(vect[1])+SQ(vect[2]));
}

//vector sum
void vect_sum(double vect1[3], double vect2[3], double result[3]){
	int i;
	for(i=0; i<3; i++) result[i] = vect1[i]+vect2[i];
}

//cross product
void vect_cross(double vect1[3], double vect2[3], double result[3]){
	result[0] = vect1[1]*vect2[2] - vect1[2]*vect2[1];
	result[1] = vect1[2]*vect2[0] - vect1[0]*vect2[2];
	result[2] = vect1[0]*vect2[1] - vect1[1]*vect2[0];
}

//rotation error
double rot_diff(double rot1[4][4], double rot2[4][4]){
	double cross[3];
	double error[3] = {0,0,0};
	double vect1[3];
	double vect2[3];
	int i, j;
	for(i=0; i<3; i++){
		for(j=0; j<3; j++){
			vect1[j] = rot1[j][i];
			vect2[j] = rot2[j][i];
		}
		vect_cross(vect1, vect2, cross);
		vect_sum(error, cross, error);
	}
	return vect_mag(error);
}
						 

//check to see how far away the IK result is from the desired pos/rot
double ik_err(double tool[4][4], double resulttool[4][4]){
	int i, j;

	//difference in position
	double poserr = sqrt(SQ(tool[0][3]-resulttool[0][3])+SQ(tool[1][3]-resulttool[1][3])+SQ(tool[2][3]-resulttool[2][3]));

	//difference in rotation
	double roterr = rot_diff(tool, resulttool);

	//printf("poserr: %.2f, roterr: %.2f\n", poserr, roterr);

	//weight rotation versus position (1m = 2 radians)
	return poserr + .5*roterr;
}


//tries to find an ik solution, returns 1 if found and 0 otherwise
int find_ik_solution(double theta2, double tool[4][4], double currentangles[7], double resultangles[7]){
	int verbose = 0;
#ifdef DEBUG
	verbose = 1
#endif
	int err;
	double errmag;
	double resulttool[4][4];
	if(verbose) printf("running WAMInverseKinematics\n");
	err = WAMInverseKinematics(theta2, tool, currentangles, resultangles);
	if(verbose) printf("running WAMForwardKinematics\n");
	WAMForwardKinematics(resultangles, resulttool, NULL);
	if(verbose) printf("done WAMForwardKinematics\n");
	errmag = ik_err(tool, resulttool);
	if(verbose) printf("errmag:%f\n", errmag);
	if(err > 0 && errmag <= .001) return 1;
	return 0;
}


//run IK to find joint angles to make a Cartesian position
//searches over joint angle 2 (first joint is 0) and solves the rest analytically
//tries to stay close to currentangles
//takes in a 4x4 homogeneous transformation matrix for the hand base (meters) as a 16-vector in row order (rot4)
//resulting angles in resultangles
//returns 1 if solution is found, 0 otherwise
int wam_ik(double rot4[16], double currentangles[7], double resultangles[7]){
	int verbose = 0;
#ifdef DEBUG
	verbose = 1
#endif
	int i, j;
	double preferredtheta2 = currentangles[2];
	double diff;
	double theta2;
	double disttolower = fabs(preferredtheta2 - wam_theta_min[2]);
	double disttoupper = fabs(wam_theta_max[2] - preferredtheta2);
	double maxdist = 0;
	int found = 0;

	//convert rot4 to a 4x4 matrix
	double tool[4][4];
	for(i=0; i<4; i++){
		for(j=0; j<4; j++){
			tool[i][j] = rot4[i*4+j];
		}
	}

	if(verbose){
		printf("currentangles %.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t\n", currentangles[0], currentangles[1], currentangles[2], 
				currentangles[3], currentangles[4], currentangles[5], currentangles[6]);
		printf("rot4:\n");
		for(i=0; i<4; i++){
			for(j=0; j<4; j++){
				printf("%.2f\t", rot4[i*4+j]);
			}
			printf("\n");
		}
	}

	//search over possible theta2 values
	if(disttolower > disttoupper) maxdist = disttolower;
	else maxdist = disttoupper;
	for(diff = 0; diff < maxdist; diff+=.02){
		if(verbose) printf("diff:%f\n", diff);
		theta2 = preferredtheta2 + diff;
		if(theta2 < wam_theta_max[2]){
			found = find_ik_solution(theta2, tool, currentangles, resultangles);
			if(found) break;
		}
		theta2 = preferredtheta2 - diff;
		if(theta2 > wam_theta_min[2]){
			found = find_ik_solution(theta2, tool, currentangles, resultangles);
			if(found) break;
		}
	}
	
	//debug prints
	if(verbose && found){
		double tool2[4][4];
		double errmag;
		WAMForwardKinematics(resultangles, tool2, NULL);
		errmag = SQ(tool[0][3] - tool2[0][3]) + SQ(tool[1][3] - tool2[1][3]) + SQ(tool[2][3] - tool2[2][3]);								
							
		printf("   angles %.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t-> %.3f\t%.3f\t%.3f\n", currentangles[0], currentangles[1], currentangles[2], 
					 currentangles[3], currentangles[4], currentangles[5], currentangles[6], tool[0][3], tool[1][3], tool[2][3]);
		printf("resangles %.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t-> %.3f\t%.3f\t%.3f\n", resultangles[0], resultangles[1], 
					 resultangles[2], resultangles[3], resultangles[4], resultangles[5], resultangles[6], tool2[0][3], 
										 tool2[1][3], tool2[2][3]);
		printf("errmag: %.3f\n", errmag);
	}	
	return found;
}

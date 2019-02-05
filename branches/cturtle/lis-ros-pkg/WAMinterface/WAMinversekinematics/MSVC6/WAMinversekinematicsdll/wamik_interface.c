//functions to export to Python in the form of a Windows DLL

#include "../../wamik.h"
#include "../../WAMKinematics.h"

#define DllExport  __declspec(dllexport)

//#define DEBUG

void print_mat4(double mat[4][4]){
	int i, j;
	for(i=0; i<4; i++){
		for(j=0; j<4; j++){
			printf("%.2f ", mat[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

void print_list(int size, double *list){
	int i;
	for(i=0; i<size; i++) printf("%.2f ", list[i]);
	printf("\n");
}

//run IK to find joint angles to make a Cartesian position
//searches over joint angle 2 (first joint is 0) and solves the rest analytically
//tries to stay close to currentangles[7]
//handbaserot4 is a 16-list (4x4 homogeneous transformation mat in row order)
//for the hand base frame (position in meters) 
//relative to the joint0 origin (z up, x forward, y left)
//handbaserot4 origin is at the base of the hand (.1 m behind palm surface)
//resulting angles in resultangles[7]
//returns 1 if solution is found, 0 otherwise
DllExport int run_wam_ik(double *handbaserot4, double *currentangles, double *resultangles){
	int result = wam_ik(handbaserot4, currentangles, resultangles);

#ifdef DEBUG
	printf("handbaserot4: ");
	print_list(16, pos);
	printf("currentangles: ");
	print_list(7, currentangles);
	printf("resultangles: ");
	print_list(7, resultangles);
#endif

	return result;
}


//run forward kinematics on the arm 
//(origin at joint0 origin, Barrett coordinates (z up, x forward, y left))
//handbasemat point is at the base of the hand (.1 m behind palm surface if there's a hand attached)
//input joint angles in theta[7], output as handbasemat, elbowmat, and wristmat (all 4x4 rotation matrices as double[16] in row-order)
DllExport void run_wam_fk(double *theta, double *handbasemat, double *elbowmat, double *wristmat){
	double x[4][4];
	double elbwrist[2][4][4];
	int i, j;

	WAMForwardKinematics(theta, x, elbwrist);

	for(i=0; i<4; i++){
		for(j=0; j<4; j++){
			handbasemat[i*4+j] = x[i][j];
			elbowmat[i*4+j] = elbwrist[0][i][j];
			wristmat[i*4+j] = elbwrist[1][i][j];
		}
	}

#ifdef DEBUG
	printf("theta:");
	print_list(7, theta);

	printf("x:\n");
	print_mat4(x);

	printf("elbow:\n");
	print_mat4(elbwrist[0]);
	printf("wrist:\n");
	print_mat4(elbwrist[1]);
#endif
}


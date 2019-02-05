//run IK to find joint angles to make a Cartesian position
//searches over joint angle 2 (first joint is 0) and solves the rest analytically
//tries to stay close to currentangles
//takes in a 4x4 homogeneous transformation matrix for the hand base (meters) as a 16-vector in row order (rot4)
//resulting angles in resultangles
//returns 1 if solution is found, 0 otherwise
int wam_ik(double rot4[16], double currentangles[7], double resultangles[7]);

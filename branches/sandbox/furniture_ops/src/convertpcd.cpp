/*
 * convertpcd.cpp
 *
 *  Created on: Oct 4, 2010
 *      Author: garratt
 */

#include <iostream>
#include <fstream>

int main(){

	std::ifstream infile("asd.dat");
	double x,y,z;
	double xoff=-2.1513, yoff= 1.1796, zoff= 1.029;
    while(infile.good() && !infile.eof()){
		infile>>x>>y>>z;
		std::cout<<x-xoff<<" "<<y-yoff<<" "<<z-zoff<<std::endl;
    }


    infile.close();

	return 0;
}

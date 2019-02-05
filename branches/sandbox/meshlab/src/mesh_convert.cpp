#include <iostream>
#include "ply2stl.h"
int main(int argc, char** argv) {
  if(argc!=3){
    std::cout<<"USAGE "<<argv[0]<<" infile.ply outfile.stl"<<std::endl;
    return -1;
  }
  
  char* infile = argv[1];
  char* outfile = argv[2];
  ply2stl(infile,outfile);
}

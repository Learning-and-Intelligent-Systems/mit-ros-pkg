/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Garratt Gallagher
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name Garratt Gallagher nor the names of other
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <time.h>
#include <gtest/gtest.h>
#include "nnn/nnn.h"
#include "logs_path.h"

template <typename PointT>
bool checkcluster( pcl::PointCloud<PointT> &cloud,std::vector<int>  &inds, double tol,int pt, std::string s){
      for(uint i=0;i<inds.size();i++)
         if(pcl::euclideanDistance(cloud.points[pt],cloud.points[inds[i]]) > tol){
            ROS_ERROR("Point in %s's clustering is %f from point %d , should be less than %f ",s.c_str(),pcl::euclideanDistance(cloud.points[pt],cloud.points[inds[i]]),pt,tol);
            return false;
         }
      return true;
}
int clustercomp(std::vector<int>  &inds1, std::vector<int>  &inds2){
      if(inds1.size() != inds2.size())
         return -1;
      for(uint i=0;i<inds1.size();i++){
         bool found=false;
         for(uint j=0;j<inds2.size();j++)
            if(inds2[j]==inds1[i]){
               found=true;
               break;
            }
         if(!found)
            return i+1;
      }
      return 0;
}

template <typename PointT>
bool testclusters(pcl::PointCloud<PointT> &cloud,std::vector<int>  &inds1, std::vector<int>  &inds2, double tol,int pt, std::string s1, std::string s2 ){

   if(inds1.size() > inds2.size()){
      if(checkcluster(cloud,inds1,tol,pt,s1))
         ROS_ERROR("trial: %f   %s size %d > %d (%s), but %s has all valid points ",tol,s1.c_str(), (int)inds1.size(),(int)inds2.size(),s2.c_str(), s1.c_str());
      return false;
   }

   if(inds2.size() > inds1.size()){
      if(checkcluster(cloud,inds2,tol,pt,s2))
         ROS_ERROR("trial: %f   %s size %d > %d (%s), but %s has all valid points ",tol,s2.c_str(), (int)inds2.size(),(int)inds1.size(),s1.c_str(), s2.c_str());
      return false;
   }

   if(clustercomp(inds1,inds2)){
      ROS_ERROR("trial: %f  the number of indices given by %s and %s are the same, but the values are different",tol,s2.c_str(),s1.c_str());
        return false;

   }

   return true;
}


//just checks to see if the indices match.  they don't have to be in order, so we have to do a more extensive search
void checkresults(std::vector<int>  &inds1, std::vector<int>  &inds2){
	EXPECT_EQ (inds2.size(), inds1.size());
    for(uint i=0;i<inds1.size();i++){
        bool found=false;
        for(uint j=0;j<inds2.size();j++)
           if(inds2[j]==inds1[i]){
              found=true;
              break;
           }
        ASSERT_TRUE(found);
     }
}



//generate test points
void generatePoints(std::vector<int> &testpts, int cloudsize, int numpts){
   srand(time(NULL));
   for(int i=0; i<numpts;i++)
      testpts.push_back(rand()%cloudsize);
}


void loadTestFiles(std::vector<pcl::PointCloud <pcl::PointXYZ> > &clouds){
   clouds.resize(3);
   for(int i=0;i<num_log_files; i++){
      pcl::io::loadPCDFile(logfiles[i],clouds[i]);
   }

}


////Test the simplest case: just the NNN algorithm, without the splitcloud structure:
//TEST (NNN, SimpleNNN)
//{
//   std::vector<pcl::PointCloud <pcl::PointXYZ> > clouds;
//   std::vector< int > inds,testindices;
//   loadTestFiles(clouds);
//   generatePoints(testindices,clouds[0].points.size(),100);
//   double testtolerance[]={.01,.02,.03,.04,.05,.06,.07,.08,.09,.1,.2,.3,.4,.5,.6,.7,.8,.9,1.0};  // 19 tolerances
//   int tolnum=19;
//   for(uint i=0;i<testindices.size();++i)
//	   for(int j=0;j<tolnum;j++){
//		   NNN(clouds[0],clouds[0].points[testindices[i]], inds, testtolerance[j]);
//	       EXPECT_EQ (checkcluster(clouds[0],inds,testtolerance[j],testindices[i],"simple NNN"), true);
//	   }
//}
//
////Test the 3-way splitcloud::
//TEST (NNN, SplitCloud)
//{
//   std::vector<pcl::PointCloud <pcl::PointXYZ> > clouds;
//   std::vector< int > indsNNN,indsSC,testindices;
//   loadTestFiles(clouds);
//   generatePoints(testindices,clouds[0].points.size(),100);
//   double testtolerance[]={.01,.02,.03,.04,.05,.06,.07,.08,.09,.1,.2,.3,.4,.5,.6,.7,.8,.9,1.0};  // 19 tolerances
//   int tolnum=19;
//   for(int j=0;j<tolnum;j++){
//	   SplitCloud sc(clouds[0],testtolerance[j]);
//	   for(uint i=0;i<testindices.size();++i){
//		   NNN(clouds[0],clouds[0].points[testindices[i]], indsNNN, testtolerance[j]);
//		   sc.NNN(clouds[0].points[testindices[i]],indsSC, testtolerance[j]);
//		   checkresults(indsNNN,indsSC);
//	   }
//   }
//}
//
////Test the 6-way splitcloud::
//TEST (NNN, SplitCloud2)
//{
//   std::vector<pcl::PointCloud <pcl::PointXYZ> > clouds;
//   std::vector< int > indsNNN,indsSC,testindices;
//   loadTestFiles(clouds);
//   generatePoints(testindices,clouds[0].points.size(),100);
//   double testtolerance[]={.01,.02,.03,.04,.05,.06,.07,.08,.09,.1,.2,.3,.4,.5,.6,.7,.8,.9,1.0};  // 19 tolerances
//   int tolnum=19;
//   for(int j=0;j<tolnum;j++){
//	   SplitCloud2 sc2(clouds[0],testtolerance[j]);
//	   for(uint i=0;i<testindices.size();++i){
//		   NNN(clouds[0],clouds[0].points[testindices[i]], indsNNN, testtolerance[j]);
//		   sc2.NNN(clouds[0].points[testindices[i]],indsSC, testtolerance[j]);
//		   checkresults(indsNNN,indsSC);
//	   }
//   }
//}

//Test all clouds at once: (much faster...)
//If there is a problem, you can re-enable the other tests, to debug in fine grained way...
TEST (NNN, CompareNNN)
{
   std::vector<pcl::PointCloud <pcl::PointXYZ> > clouds;
   std::vector< int > indsNNN,indsSC,testindices;
   loadTestFiles(clouds);
   generatePoints(testindices,clouds[0].points.size(),100);
   double testtolerance[]={.01,.02,.03,.04,.05,.06,.07,.08,.09,.1,.2,.3,.4,.5,.6,.7,.8,.9,1.0};  // 19 tolerances
   int tolnum=19;
   for(int j=0;j<tolnum;j++){
	   SplitCloud2 sc2(clouds[0],testtolerance[j]);
	   SplitCloud sc(clouds[0],testtolerance[j]);
	   for(uint i=0;i<testindices.size();++i){
		   NNN(clouds[0],clouds[0].points[testindices[i]], indsNNN, testtolerance[j]);
		   //make sure all the points were close enough
	       ASSERT_TRUE(checkcluster(clouds[0],indsNNN,testtolerance[j],testindices[i],"simple NNN"));
	       //check that splitCloud gives same results
		   sc.NNN(clouds[0].points[testindices[i]],indsSC, testtolerance[j]);
		   checkresults(indsNNN,indsSC);
//		   check that SplitCloud2 gives same results:
		   sc2.NNN(clouds[0].points[testindices[i]],indsSC, testtolerance[j]);
		   checkresults(indsNNN,indsSC);
	   }
   }
}


TEST (NNN, SplitCloud2Indexing){
	std::vector<pcl::PointCloud <pcl::PointXYZ> > clouds;
	loadTestFiles(clouds);
    double testtolerance[]={.01,.02,.03,.04,.05,.06,.07,.08,.09,.1,.2,.3,.4,.5,.6,.7,.8,.9,1.0};  // 19 tolerances
    int tolnum=19;
    for(int j=0;j<tolnum;j++){
		for(uint c=0;c<clouds.size();++c){
			 SplitCloud2 sc2(clouds[c],testtolerance[j]);
			 for(uint i=0;i<clouds[c].points.size();i++){
				 ASSERT_TRUE(sc2.checkIndex(i));
			 }
		}
    }
}

////test the AssignInds function of SC2.
////AssignInds is a optimization shortcut, which just marks the indices instead of making a vector of them
//TEST (NNN, SplitCloud2Assign){
//	   std::vector<pcl::PointCloud <pcl::PointXYZ> > clouds;
//	   std::vector< int > cloudinds,extractedinds,indsSC,testindices;
//	   loadTestFiles(clouds);
//	   generatePoints(testindices,clouds[0].points.size(),100);
//	   double testtolerance[]={.01,.02,.03,.04,.05,.06,.07,.08,.09,.1,.2,.3,.4,.5,.6,.7,.8,.9,1.0};  // 19 tolerances
//	   int tolnum=19;
//	   for(uint c=0;c<clouds.size();++c){
//		   for(int j=0;j<tolnum;j++){
//			   SplitCloud2 sc2(clouds[c],testtolerance[j]);
//			   for(uint i=0;i<testindices.size();++i){
//				   cloudinds.clear();
//				   cloudinds.resize(clouds[c].points.size(),0);
//				   sc2.NNN(clouds[c].points[testindices[i]],indsSC, testtolerance[j]);
//				   sc2.AssignInds(clouds[c].points[testindices[i]],cloudinds, testtolerance[j],1);
//				   checkresults(indsNNN,indsSC);
//			   }
//		   }
//	   }
//
//}




//  for(uint i=0;i<cloud.points.size();i++){
//	 int index=getIndex(cloud.points[i]);
//	 if(!isIndex(cloud.points[i],index)){
//		ROS_ERROR("index error. pt at %f, %f, %f is not in index %d",cloud.points[i].x,cloud.points[i].y,cloud.points[i].z,index);
//		break;
//	 }
//
//  }



int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}






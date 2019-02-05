/*
 * imu_decode.cpp
 *
 *  Created on: Dec 4, 2010
 *      Author: garratt
 */

//Playing with the imu values from the kinect...




#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <Eigen3/Core>
#include <Eigen3/Geometry>
#include <deque>


template <typename PointT>
Eigen3::Vector4f toEigen(PointT in){
	Eigen3::Vector4f v;
	v(0)=in.x; v(1)=in.y, v(2)=in.z;
	return v;
}



Eigen3::Vector4f median2(std::deque<Eigen3::Vector4f> &accs,std::vector< std::vector<int> > &counts){
	for(uint ind=0;ind<3;ind++)
		counts.push_back(std::vector<int>(accs.size(),0));
	//if there are duplicates, all the  counts will get attributed to the first occurrence
	for(int i=0;i<accs.size();++i){ //for each value in accs,
		 bool seen[]={false,false,false};
		  for(int j=0;j<accs.size();++j){ //start checking against all the others. this will not go past i
			  for(uint ind=0;ind<3;ind++)
				  if(!seen[ind] && accs[j](ind)==accs[i](ind)){ //if there is a match, stop checking that val
					  counts[ind][j]++;
					  seen[ind]=true;
				  }
			  if(seen[0] && seen[1] && seen[2]) //if we've gotten each value, we stop
				  break;
		  }
	}
	//now we have counts for every value
	int maxes[]={0,0,0}, maxval[]={0,0,0};
	for(int i=0;i<accs.size();++i){
		for(uint ind=0;ind<3;ind++){
			if(maxval[ind] < counts[ind][i]){
				maxval[ind] = counts[ind][i];
				maxes[ind]=i;
			}
		}
	}
	Eigen3::Vector4f ret;
	for(uint ind=0;ind<3;ind++)
		ret(ind) = accs[maxes[ind]](ind);
	return ret;

}


Eigen3::Vector4f median(std::deque<Eigen3::Vector4f> &accs){
	std::vector< std::vector<int> > counts;
	return median2(accs,counts);
}


Eigen3::Vector4f informedMean(std::deque<Eigen3::Vector4f> &accs, Eigen3::Vector4f &variance){
	std::vector< std::vector<int> > counts;
	Eigen3::Vector4f ret = median2(accs,counts);
	//now we have counts for every value
	int maxes[]={0,0,0}, maxval[]={0,0,0},realmaxval[]={0,0,0};
	for(int i=0;i<accs.size();++i){
		for(uint ind=0;ind<3;ind++){
			if(ret(ind) == accs[i](ind)){
				if(realmaxval[ind] < counts[ind][i])
				realmaxval[ind] = counts[ind][i];
			}
			else{ //find next runner up
				if(maxval[ind] < counts[ind][i] ){
					maxval[ind] = counts[ind][i];
					maxes[ind]=i;
				}
			}
		}
	}
	Eigen3::Vector4f ret2;
	for(uint ind=0;ind<3;ind++){
		ret2(ind) = accs[maxes[ind]](ind);
		ret(ind) = (((float)realmaxval[ind])*ret(ind) + ret2(ind)*((float)maxval[ind]))/((float)(realmaxval[ind]+maxval[ind]));
		variance(ind)=((float)(accs.size()-(realmaxval[ind]+maxval[ind]))) / ((float)(accs.size()));
//		printf(" ( %d %d %d ) ",accs.size(),realmaxval[ind],maxval[ind]);
	}


	return ret;
}



Eigen3::Vector4f mean(std::deque<Eigen3::Vector4f> &accs){
	Eigen3::Vector4f ret;
	ret.Zero();
	for(int i=0;i<accs.size();++i)
		ret+=accs[i];
	ret/=(accs.size());
	return ret;
}



class ImuDecode
{

private:
  tf::TransformListener tl_;
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  std::deque<float> norms;
  std::deque<Eigen3::Vector4f> accs;
  double tstart;
  int dequesize;
public:

  ImuDecode():tl_(ros::Duration(120.0))
  {
    sub_=n_.subscribe("/camera/imu", 1, &ImuDecode::imucb, this);
    tstart=ros::Time::now().toSec();
    dequesize=30;
  }

  void imucb(const sensor_msgs::ImuConstPtr &imu){

	  Eigen3::Vector4f acc = toEigen(imu->linear_acceleration);
	  float ave=0.0;
	  norms.push_back(acc.norm());
	  if(norms.size() > dequesize) norms.pop_front();
	  for(uint i=0;i<norms.size(); ++i) ave+=norms[i];
	  ave/=norms.size();
	  if(norms.size()>1){
		  printf("%.04f   %.04f  %.04f %.04f %.04f",ros::Time::now().toSec()-tstart,norms.back(),acc(0),acc(1),acc(2));
		  Eigen3::Vector4f vars,lastvals=median(accs);
//		  for(int i=0;i<accs.size();++i)
//			  if(accs[i](0) != acc(0) && accs[i](1) != acc(1) && accs[i](2) != acc(2)){
//				  lastvals=accs[i];
//				  break;
//			  }
		  printf(" %.04f %.04f %.04f ",lastvals(0),lastvals(1),lastvals(2));
		  lastvals=mean(accs);
		  printf("  mean %.04f %.04f %.04f ",lastvals(0),lastvals(1),lastvals(2));
		  lastvals=informedMean(accs,vars);
		  printf("  if  %.04f %.04f %.04f ",lastvals(0),lastvals(1),lastvals(2));
		  printf("   var  %.04f %.04f %.04f \n",vars(0),vars(1),vars(2));
	  }
//	  printf("mag:  %.03f  ave:  %.04f   deviation:   %.04f \n",norms.back(),ave, fabs(norms.back()-ave));
//	  std::cout<<"mag: "<<acc.norm()<<std::endl;
	  accs.push_front(acc);
	  if(accs.size() > dequesize) accs.pop_back();

  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_decoder");
  ros::NodeHandle n;
  ImuDecode snapshotter;
  ros::spin();
  return 0;
}

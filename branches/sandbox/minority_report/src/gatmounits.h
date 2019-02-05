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


#ifndef GATMOUNITS_H_
#define GATMOUNITS_H_


#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <cstring>

#define VERBOSE_MUTEX 0
#define glock(mutex) mutex.mylock(__PRETTY_FUNCTION__)
#define gunlock(mutex) mutex.myunlock(__PRETTY_FUNCTION__)
#define gscopedlock(mutex) gatmo_scoped_lock lock(mutex,__PRETTY_FUNCTION__)


//These are a collection of units taken from the original GATMO system

/** \brief @b gatmo_pose represents a 2d point and a direction
 * \author Garratt Gallagher
 */
class gatmo_pose_t{
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & px;
        ar & py;
        ar & pa;
        ar & timestamp;
    }


public:
   float px;
   float py;
   float pa;
     double timestamp;
   gatmo_pose_t(float x, float y, float a){px=x;py=y;pa=a;}
   gatmo_pose_t(){px=0.0;py=0.0;pa=0.0;}

   void generate(gatmo_pose_t mean, gatmo_pose_t range){
      //TODO: look into making these normally distributed using muller-box transform...
      //generates from a uniform distribution on the interval (mean -range, mean+range)
      px=((float)(rand()%(((int)(range.px*20000.0))))/10000.0-range.px)+mean.px;
      py=((float)(rand()%(((int)(range.py*20000.0))))/10000.0-range.py)+mean.py;
      pa=((float)(rand()%(((int)(range.pa*20000.0))))/10000.0-range.pa)+mean.pa;
   }

   gatmo_pose_t(gatmo_pose_t mean, gatmo_pose_t range){
      generate(mean,range);

   }
   bool operator!=(gatmo_pose_t p){
      return fabs(p.pa-pa)>.0001 || fabs(p.px-px)>.0001 || fabs(p.py-py)>.0001;

   }
   gatmo_pose_t operator+(gatmo_pose_t p){
      gatmo_pose_t out;
      out.px=px+cos(pa)*p.px -sin(pa)*p.py;
      out.py=py+cos(pa)*p.py+sin(pa)*p.px;
      out.pa=pa+p.pa;
      return out;
   }

};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b gatmo_point represents a 2d point
 * \author Garratt Gallagher
 */
class gatmo_point_t{
public:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
        ar & weight;
    }

   float x;
   float y;
   float weight;
   
   /** \brief subtracts two points as if they were vectors. */
   gatmo_point_t operator-(gatmo_point_t p) const{
      gatmo_point_t out;
      out.x=x-p.x;
      out.y=y-p.y;
      return out;
   }
   
   /** \brief adds two points as if they were vectors. */
   gatmo_point_t operator+(gatmo_point_t p) const{
      gatmo_point_t out;
      out.x=x+p.x;
      out.y=y+p.y;
      return out;
   }
   
   /** \brief adds one point to the current point as if they were vectors. */
   void operator+=(gatmo_point_t p){
      x+=p.x;
      y+=p.y;
   }
   
   /** \brief multiply a point by a scalar. */   
   gatmo_point_t operator*(float m) const{
      gatmo_point_t out;
      out.x=x*m;
      out.y=y*m;
      out.weight=weight*m;
      return out;
   }
   
   
   /** \brief divide a point by a scalar. */  
   gatmo_point_t operator/(float m) const{
      return (*this)*(1.0/m);
   }

   gatmo_point_t(float xin, float yin, float win){x=xin; y=yin; weight=win;}
   gatmo_point_t(float xin, float yin){x=xin; y=yin; weight=0.0;}
   gatmo_point_t(){x=0.0; y=0.0; weight=0.0;}
   gatmo_point_t(gatmo_pose_t p){x=p.px; y=p.py; weight=p.pa;}


   /** \brief given another point, converts this point into a pose, using the difference between the points as a direction*/ 
   gatmo_pose_t toPose(gatmo_point_t prev) const{
      gatmo_point_t diff = *this-prev;
      float angle=atan2(diff.y,diff.x);
      return gatmo_pose_t(x,y,angle);
   }
   //does not give heading
   
   /** \brief convertsw a point to a pose without adding heading information. */   
   gatmo_pose_t toPose() const{
      float angle=0.0;
      return gatmo_pose_t(x,y,angle);
   }
   
   gatmo_point_t(gatmo_point_t mean, gatmo_point_t range){generate(mean,range); weight=0.0;}

   /** \brief determine if two points are equal (if their x and y are close). */
   bool operator==(gatmo_point_t p){
      return fabs(x-p.x)<.00001 && fabs(y-p.y)<.00001;
   }
   
      /** \brief rotates a point around the origin */
   gatmo_point_t rotate(float theta){
      gatmo_point_t out;
      out.x=cos(theta)*x-sin(theta)*y;
      out.y=cos(theta)*y+sin(theta)*x;
      return out;
   }
   
      /** \brief finds the euclidean distance between points */
   float dist(gatmo_point_t p){
      gatmo_point_t p2=*this-p;
      return sqrt(p2.x*p2.x+p2.y*p2.y);
   }
   
   /** \brief finds the euclidean distance between this point and a pose */
   float dist(gatmo_pose_t pose){
      gatmo_point_t p;
      p.x=pose.px; p.y=pose.py;
      gatmo_point_t p2=*this-p;
      return sqrt(p2.x*p2.x+p2.y*p2.y);

   }

   /** \brief generate function is used for testing.  it takes in a mean, and a range (similar to a variance). */
   void generate(gatmo_point_t mean, gatmo_point_t range){
      //TODO: look into making these normally distributed using muller-box transform...
      //generates from a uniform distribution on the interval (mean -range, mean+range)
      x=((float)(rand()%(((int)(range.x*20000.0))))/10000.0-range.x)+mean.x;
      y=((float)(rand()%(((int)(range.y*20000.0))))/10000.0-range.y)+mean.y;
   }
   
   /** \brief print point info. */
   void print(){
      std::cout<<"("<<x<<", "<<y<<")"<<std::endl;
   }


};








/** \brief @b gatmo_mutex is a wrapper around the pthread mutex system.  It adds a human readable name and print functions, to make mutex programming easily debuggable.
 * \author Garratt Gallagher
 */
struct gatmo_mutex_t{
   char mutex_name[500];
   pthread_mutex_t myMutex;
   bool enable;  //if this is false, we won't use the mutex.  this is useful if we are trying to see if a mutex is really needed...
   bool verbose;  // a control to allow each mutex to print independently

   void mylock(const char  *func){
      (VERBOSE_MUTEX || verbose) && std::cout<<"Approaching "<<mutex_name<<" Lock in   "<<func<<std::endl;
      pthread_mutex_lock(&myMutex);
      (VERBOSE_MUTEX || verbose) && std::cout<<"Inside "<<mutex_name<<" Lock in   "<<func<<std::endl;
   }
   void myunlock(const char  *func){
      (VERBOSE_MUTEX || verbose) && std::cout<<"Unlocking "<<mutex_name<<" Lock in   "<<func<<std::endl;
      pthread_mutex_unlock(&myMutex);
   }
   gatmo_mutex_t(const char * name){
      verbose=false;
      enable=true;
      strcpy(mutex_name,name);
      pthread_mutex_init(&myMutex, NULL);
   }
};


/** \brief @b gatmo_mutex version of a scoped lock
 * \author Garratt Gallagher
 */
class gatmo_scoped_lock{
   gatmo_mutex_t &mx;
   std::string funcname;
public:
   gatmo_scoped_lock(gatmo_mutex_t &_mx,const char  *func):mx(_mx){
      mx.mylock(func);
      funcname=func;
   }

   ~gatmo_scoped_lock(){
      funcname+=(":--> end of scope");
      mx.myunlock(funcname.c_str());
   }

};


/** \brief @b gatmo_point_3d
 * \author Garratt Gallagher
 */
class gatmo_point3D_t{
public:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
        ar & z;
        ar & weight;
    }

   float x;
   float y;
   float z;
   float weight;
   gatmo_point3D_t operator-(gatmo_point3D_t p) const{
      gatmo_point3D_t out;
      out.x=x-p.x;
      out.y=y-p.y;
      out.z=z-p.z;
      return out;
   }
   gatmo_point3D_t operator+(gatmo_point3D_t p) const{
      gatmo_point3D_t out;
      out.x=x+p.x;
      out.y=y+p.y;
      out.z=z+p.z;
      return out;
   }
   gatmo_point3D_t cross(gatmo_point3D_t p) const{
         gatmo_point3D_t out;
         out.x=y*p.z - z*p.y;
         out.y=z*p.x - x*p.z;
         out.z=x*p.y - y*p.x;
         out.weight=weight;
         return out;
      }
   double dot(gatmo_point3D_t p) const{
         return x*p.x+y*p.y+z*p.z;
      }
   void operator+=(gatmo_point3D_t p){
      x+=p.x;
      y+=p.y;
      z+=p.z;
   }
   gatmo_point3D_t operator*(float m) const{
      gatmo_point3D_t out;
      out.x=x*m;
      out.y=y*m;
      out.z=z*m;
      out.weight=weight*m;
      return out;
   }
   gatmo_point3D_t operator/(float m) const{
      return (*this)*(1.0/m);
   }

   gatmo_point3D_t(float xin, float yin, float zin, float win){x=xin; y=yin; z=zin; weight=win;}
   gatmo_point3D_t(float xin, float yin, float zin){x=xin; y=yin; z=zin; weight=0.0;}
   gatmo_point3D_t(){x=0.0; y=0.0; z=0.0; weight=0.0;}
   gatmo_point3D_t(gatmo_point_t p){x=p.x; y=p.y; z=0.0; weight=p.weight;}


   bool operator==(gatmo_point3D_t p){
      return fabs(x-p.x)<.0005 && fabs(y-p.y)<.0005 && fabs(z-p.z)<.0005;
   }
   bool operator!=(gatmo_point3D_t p){
      return !(*this==p);
   }

   float dist(gatmo_point3D_t p) const{
      gatmo_point3D_t p2=*this-p;
      return sqrt(p2.x*p2.x+p2.y*p2.y+p2.z*p2.z);

   }
   double mag(){
         return sqrt(x*x+y*y+z*z);
   }


///generate function is used for testing.  it takes in a mean, and a range (similar to a variance)
   void generate(gatmo_point3D_t mean, gatmo_point3D_t range){
      //TODO: look into making these normally distributed using muller-box transform...
      //generates from a uniform distribution on the interval (mean -range, mean+range)
      x=((double)(rand()%(((int)(range.x*20000.0))))/10000.0-range.x)+mean.x;
      y=((double)(rand()%(((int)(range.y*20000.0))))/10000.0-range.y)+mean.y;
      z=((double)(rand()%(((int)(range.z*20000.0))))/10000.0-range.z)+mean.z;
   }
   void print(){
      std::cout<<"("<<x<<", "<<y<<", "<<z<<")"<<std::endl;
   }


};



struct gatmo_axis_angle_t{
   double x,y,z,angle;
   void toImplicit(double &ax, double &ay, double &az){
      double mag=ax*ax+ay*ay+az*az;
      if(mag==0) mag=1;
      ax=x*angle/mag;
      ay=y*angle/mag;
      az=z*angle/mag;
   }
   void fromImplicit(double ax, double ay, double az){
      angle=sqrt(ax*ax+ay*ay+az*az);
      if(angle==0)
         angle=1.0; //because it won't matter what it is.
      x=ax/angle;
      y=ay/angle;
      z=az/angle;
   }

};


//  qw + i qx + j qy + k qz
class gatmo_quaternion_t{
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
        ar & z;
        ar & w;
    }
public:
   double x,y,z,w;
   void operator=(gatmo_quaternion_t p) {
      x=p.x;
      y=p.y;
      z=p.z;
      w=p.w;
   }
   bool operator==(gatmo_quaternion_t p){
      return (fabs(x-p.x)<.00001 && fabs(y-p.y)<.00001 && fabs(z-p.z)<.00001 && fabs(w-p.w)<.00001) ||
            (fabs(x+p.x)<.00001 && fabs(y+p.y)<.00001 && fabs(z+p.z)<.00001 && fabs(w+p.w)<.00001);
   }
   bool operator!=(gatmo_quaternion_t p){
      return  (fabs(x-p.x)>=.00001 || fabs(y-p.y)>=.00001 || fabs(z-p.z)>=.00001 || fabs(w-p.w)>=.00001) &&
            (fabs(x+p.x)>=.00001 || fabs(y+p.y)>=.00001 || fabs(z+p.z)>=.00001 || fabs(w+p.w)>=.00001);
   }
   gatmo_quaternion_t(){
      x=0.0; y=0.0; z=0.0; w=1.0; //identity
   }
   gatmo_quaternion_t(double _x, double _y, double _z, double _w){
      x=_x; y=_y; z=_z; w=_w; //identity
   }
   gatmo_quaternion_t(gatmo_point3D_t p){
      x=p.x; y=p.y; z=p.z; w=0.0;
   }
   gatmo_quaternion_t operator-(gatmo_quaternion_t p) const{
      gatmo_quaternion_t out;
      out.x=x-p.x;
      out.y=y-p.y;
      out.z=z-p.z;
      out.w=w-p.w;
      return out;
   }
   gatmo_quaternion_t operator+(gatmo_quaternion_t p) const{
      gatmo_quaternion_t out;
      out.x=x+p.x;
      out.y=y+p.y;
      out.z=z+p.z;
      out.w=w+p.w;
      return out;
   }
   gatmo_quaternion_t operator*(gatmo_quaternion_t p) const{
      gatmo_quaternion_t out;
      out.w = w*p.w - x*p.x - y*p.y- z*p.z;
      out.x=  x*p.w + w*p.x + y*p.z - z*p.y;
      out.y=  w*p.y - x*p.z + y*p.w + z*p.x;
      out.z=   w*p.z + x*p.y - y*p.x + z*p.w;
      return out;
   }
   gatmo_quaternion_t conj() const{
      gatmo_quaternion_t out;
      out.w = w;
      out.x=  -x;
      out.y=  -y;
      out.z=   -z;
      return out;
   }
   double mag() const{
      return w*w+x*x+y*y+z*z;
   }
   gatmo_quaternion_t inv() const{
      return conj()/mag();
   }
// gatmo_quaternion_t operator*(double p) const{
//    gatmo_quaternion_t out;
//    out.w = w*p;
//    out.x=  x*p;
//    out.y=  y*p;
//    out.z=   z*p;
//    return out;
// }
   gatmo_quaternion_t operator*(double p) const{
      gatmo_quaternion_t out;
      if(fabs(w-1.0)<.00001) return *this;
      double angle, ax, ay, az;
      angle=2*acos(w);
      double temp=1.0/(sqrt(1-w*w));
      ax=x*temp;
      ay=y*temp;
      az=z*temp;
      //normalize
      temp=sqrt(ax*ax+ay*ay+az*az);
      if(fabs(temp)<.0001) temp=1.0;
      out.fromAxisAngle(angle*p,ax/temp,ay/temp,az/temp);
      out.normalize();
      return out;
   }
   gatmo_quaternion_t operator/(double p) const{
      gatmo_quaternion_t out;
      out.w = w/p;
      out.x=  x/p;
      out.y=  y/p;
      out.z=   z/p;
      return out;
   }
   void normalize(){
      double len=sqrtf(mag());
      x/=len;
      y/=len;
      z/=len;
      w/=len;
   }

   gatmo_point3D_t toPoint() const{
      return gatmo_point3D_t(x,y,z);
   }

   gatmo_point3D_t getPerpComponent(gatmo_point3D_t pin) const{
      gatmo_quaternion_t  qin=gatmo_quaternion_t(pin);
      gatmo_quaternion_t qout=(qin-*this*qin*(*this))*.5;
      return qout.toPoint();
   }

   gatmo_point3D_t rotate(gatmo_point3D_t p) const{
      gatmo_quaternion_t out = *this * gatmo_quaternion_t(p) * conj();
      return out.toPoint();
   }

   void fromAxisAngle(double angle, double ax, double ay, double az){
         x = ax * sin(angle/2);
         y = ay * sin(angle/2);
         z = az * sin(angle/2);
         w = cos(angle/2);
   }

   void fromAxisAngle(gatmo_axis_angle_t in){
      fromAxisAngle(in.angle, in.x, in.y, in.z);
   }

   gatmo_axis_angle_t toAxisAngle(){
      normalize();
      gatmo_axis_angle_t out;
      out.angle = 2 * acos(w);
      double denom=sqrt(1-w*w);
      if(fabs(denom) < .0001)
         denom=1;
      out.x = x / denom;
      out.y = y / denom;
      out.z = z / denom;
      return out;
   }

   void fromEuler(double roll, double pitch, double yaw){
       // Assuming the angles are in radians.
       double c1 = cos(yaw/2);
       double s1 = sin(yaw/2);
       double c2 = cos(pitch/2);
       double s2 = sin(pitch/2);
       double c3 = cos(roll/2);
       double s3 = sin(roll/2);
       double c1c2 = c1*c2;
       double s1s2 = s1*s2;
       w =c1c2*c3 - s1s2*s3;
      x =c1c2*s3 + s1s2*c3;
      y =s1*c2*c3 + c1*s2*s3;
      z =c1*s2*c3 - s1*c2*s3;
   }

   void toEuler(double &roll, double &pitch, double &yaw){
      double test = x*y + z*w;
      if (test > 0.499) { // singularity at north pole
         yaw = 2 * atan2(x,w);
         pitch = 1.570796;
         roll = 0;
         return;
      }
      if (test < -0.499) { // singularity at south pole
         yaw = -2 * atan2(x,w);
         pitch = - 1.570796;
         roll = 0;
         return;
      }
       double sqx = x*x;
       double sqy = y*y;
       double sqz = z*z;
       yaw = atan2(2*y*w-2*x*z , 1 - 2*sqy - 2*sqz);
      pitch = asin(2*test);
      roll = atan2(2*x*w-2*y*z , 1 - 2*sqx - 2*sqz);
   }

   ///transforms into a three by three matrix represented in rowfirst single array
   //  p[0] p[1] p[2]
   //  p[3] p[4] p[5]
   //  p[6] p[7] p[8]
   void toMatrix(double *&p) const{
      p[0]=1 - 2*y*y - 2*z*z;
      p[1]=2*x*y - 2*z*w;
      p[2]=2*x*z + 2*y*w;
      p[3]=2*x*y + 2*z*w;
      p[4]=1 - 2*x*x - 2*z*z;
      p[5]=2*y*z - 2*x*w;
      p[6]=2*x*z - 2*y*w;
      p[7]=2*y*z + 2*x*w;
      p[8]=1 - 2*x*x - 2*y*y;
   }

   //general transformation
   //  p[0] p[1] p[2]
   //  p[3] p[4] p[5]
   //  p[6] p[7] p[8]
   void fromMatrix(double m[9]){
     double trace = m[0] + m[4] + m[8];
     if( trace > 0 ) {
        double s = 0.5f / sqrtf(trace+ 1.0f);
       w = 0.25f / s;
       x = ( m[7] - m[5] ) * s;
       y = ( m[2] - m[6] ) * s;
       z = ( m[3] - m[1] ) * s;
     } else {
       if ( m[0] > m[4] && m[0] > m[8] ) {
         double s = 2.0f * sqrtf( 1.0f + m[0] - m[4] - m[8]);
         w = (m[7] - m[5] ) / s;
         x = 0.25f * s;
         y = (m[1] + m[3] ) / s;
         z = (m[2] + m[6] ) / s;
       } else if (m[4] > m[8]) {
         double s = 2.0f * sqrtf( 1.0f + m[4] - m[0] - m[8]);
         w = (m[2] - m[6] ) / s;
         x = (m[1] + m[3] ) / s;
         y = 0.25f * s;
         z = (m[5] + m[7] ) / s;
       } else {
         double s = 2.0f * sqrtf( 1.0f + m[8] - m[0] - m[4] );
         w = (m[3] - m[1] ) / s;
         x = (m[2] + m[6] ) / s;
         y = (m[5] + m[7] ) / s;
         z = 0.25f * s;
       }
     }
   }

   void print(){
      std::cout<<x<<", "<<y<<", "<<z<<", "<<w<<std::endl;

   }
   void generate(){
      x=((double)(rand()%(((int)(20000.0))))/10000.0)-1.0;
      y=((double)(rand()%(((int)(20000.0))))/10000.0)-1.0;
      z=((double)(rand()%(((int)(20000.0))))/10000.0)-1.0;
      w=((double)(rand()%(((int)(20000.0))))/10000.0)-1.0;
      normalize();
   }



};



class gatmo_pose6D_t{
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & position;
        ar & orientation;
        ar & timestamp;
    }


public:
   gatmo_point3D_t position;
   gatmo_quaternion_t orientation;
     double timestamp;
     gatmo_pose6D_t(gatmo_point3D_t p):position(p){timestamp=0.0;}
     gatmo_pose6D_t(float _x, float _y, float _z):position(_x,_y,_z){timestamp=0.0;}
     gatmo_pose6D_t(float _x, float _y, float _z, gatmo_quaternion_t q):position(_x,_y,_z),orientation(q){timestamp=0.0;}
     gatmo_pose6D_t(gatmo_quaternion_t q):position(0.0,0.0,0.0),orientation(q){timestamp=0.0;}
     gatmo_pose6D_t(){timestamp=0.0;}
     gatmo_pose6D_t operator+(gatmo_pose6D_t p) const{
       gatmo_pose6D_t out;
      out.position=position+p.position;
      out.orientation=orientation+p.orientation;
      return out;
   }
     gatmo_pose6D_t operator-(gatmo_pose6D_t p) const{
       gatmo_pose6D_t out;
      out.position=position-p.position;
      out.orientation=orientation-p.orientation;
      return out;
   }

     bool operator==(gatmo_pose6D_t p) const{
      return p.orientation==orientation && p.position == position;
   }

     //treat the pose like a transform, and multiply transforms
     gatmo_pose6D_t operator*(gatmo_pose6D_t p) const{
       gatmo_pose6D_t out;
       //translation is 2nd translation+ rotation *first translation
      out.position=position+orientation.rotate(p.position);
      out.orientation=orientation*p.orientation;
      return out;
   }
     gatmo_pose6D_t operator*(double s) const{
       gatmo_pose6D_t out;
       //translation is 2nd translation+ rotation *first translation
      out.position=position*s;
      out.orientation=orientation*s;
      return out;
   }
     //transform a point into this frame
     gatmo_point3D_t operator*(gatmo_point3D_t p) const{
       gatmo_point3D_t out;
       //translation is = translation+ rotation *first translation
      return position+orientation.rotate(p);
   }

     //transform a point into this frame
     gatmo_point3D_t operator*(gatmo_point_t p) const{
       //translation is = translation+ rotation *first translation
      return position+orientation.rotate(gatmo_point3D_t(p));
   }

     //the matrix order for opengl is opposite from everything else:
     //returns a 4by4 matrix, in column first form
    void toOpenGLMatrix(double *mat) const{
       double *mp = new double[9];
       //first get matrix from quaternion:
       orientation.toMatrix(mp);
         //  mp[0] mp[1] mp[2]
         //  mp[3] mp[4] mp[5]
         //  mp[6] mp[7] mp[8]
       //OpenGL format:
//     C[0]    C[4]     C[8]     C[12]
//     C[1]    C[5]  C[9]  C[13]
//     C[2]    C[6]  C[10]    C[14]
//     C[3]    C[7]  C[11]    C[15]

       mat[0]=mp[0];  mat[4]=mp[1];  mat[8]=mp[2];       mat[12]=position.x;
       mat[1]=mp[3];  mat[5]=mp[4];  mat[9]=mp[5];    mat[13]=position.y;
       mat[2]=mp[6];  mat[6]=mp[7];  mat[10]=mp[8];      mat[14]=position.z;
       mat[3]=0.0;    mat[7]=0.0;    mat[11]=0.0;     mat[15]=1.0;
       delete  [] mp;
   }


    void fromOpenGLMatrix(double *mat){
       double *mp = new double[9];
       mp[0]=mat[0];  mp[1]=mat[4];  mp[2]=mat[8];  position.x=mat[12];
       mp[3]=mat[1];  mp[4]=mat[5];  mp[5]=mat[9];  position.y=mat[13];
       mp[6]=mat[2];  mp[7]=mat[6];  mp[8]=mat[10]; position.z=mat[14];
       orientation.fromMatrix(mp);
       delete  [] mp;
    }



    //input a 3by4 matrix
    gatmo_pose6D_t(double *p):position(p[3],p[7],p[11]){timestamp=0.0;
       double mp[9];
       mp[0]=p[0];    mp[1]=p[1];    mp[2]=p[2];
       mp[3]=p[4];    mp[4]=p[5];    mp[5]=p[6];
       mp[6]=p[8];    mp[7]=p[9];    mp[8]=p[10];
       orientation.fromMatrix(mp);
    }


    double distToXYPlane(gatmo_point3D_t p){
      p=p-position; //subtract off 'origin'
//    std::cout<<"p after translate = "<<p.x<<", "<<p.y<<", "<<p.z<<std::endl;
      p=orientation.inv().rotate(p); //rotate by inverse of orientation, to get point in this coordinate system
//    std::cout<<"p after rotate = "<<p.x<<", "<<p.y<<", "<<p.z<<std::endl;
      gatmo_quaternion_t axis(0, 0, 1, 0); //project it onto z axis
      p=axis.getPerpComponent(p);
//    std::cout<<"p after perp component = "<<p.x<<", "<<p.y<<", "<<p.z<<std::endl;
      return p.z;
    }
    void print(){
       std::cout<<" pose: ("<<position.x<<", "<<position.y<<", "<<position.z<<") orientation: ";
       orientation.print();
    }

    //compute the inverse of the transform
    gatmo_pose6D_t inv(){
       gatmo_pose6D_t out;
       out.orientation=orientation.inv();
       out.position=(out.orientation.rotate(position))*-1.0;
       out.timestamp = timestamp;
       return out;
    }


};







class gatmo_line3d_t{
   protected:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & start;
        ar & end;
        ar & sev;
        ar & dist;
        ar & rdist;
        ar & weight;
        ar & points;

    }
   double dist,rdist;
   gatmo_point3D_t sev;
public:
   gatmo_point3D_t start, end; //start and end are the aligned coordinates
   std::list<gatmo_point3D_t> points;
   float weight;

   ///no caching version
   double distto(gatmo_point3D_t p){
      //vector to start:
      gatmo_point3D_t v=start-p, startendvector=end-start;
      return fabs(startendvector.cross(v).mag())/startendvector.mag();
   }

   ///cached version:
   double disttoCached(gatmo_point3D_t p){
      //vector to start:
      gatmo_point3D_t v=start-p;
      return fabs(sev.cross(v).mag())*rdist;
   }

   ///Find distance to XY plane at given pose
   double disttoPlane(gatmo_pose6D_t p){
      return (fabs(p.distToXYPlane(start))+fabs(p.distToXYPlane(end)))/2.0;
   }


   void set(gatmo_point3D_t s, gatmo_point3D_t e){
      ///set the line by defining the start and end point
      start =s;
      end = e;
      sev=e-s;
      dist = e.dist(s);
      rdist=1/dist;
   }
   gatmo_line3d_t(gatmo_point3D_t s, gatmo_point3D_t e){ set(s,e);}
   gatmo_line3d_t(){}


   gatmo_point3D_t getMean(){
      return (start+end)/2.0;
   }
   double getDist(){
      return start.dist(end);
   }
   void extend(gatmo_point3D_t p){
      ///call this when adding a point to the line, to extend the line to the projection of that point
         //new point = sev . (p-s) * sev /sev^2
         gatmo_point3D_t sp = p-start;
         double dotproduct=sp.dot(sev)*rdist*rdist;
//       cout<<"sev.x*sp.x "<<sev.x*sp.x<<"  + sev.y+sp.y "<<sev.y+sp.y<<endl;
//       cout<<"start: "<<start<<"  end: "<<end<<"  point: "<<p<<"  sev: "<<sev<<" sp: "<<sp<<"  dist: "<<dist<<"  dotproduct: "<<dotproduct<<endl;
         if(dotproduct>0 && dotproduct < 1.0) return;  //projection is within current line
         gatmo_point3D_t newpoint(dotproduct*sev.x+start.x,dotproduct*sev.y+start.y,dotproduct*sev.z+start.z);
//       cout<<"newpoint: "<<newpoint<<endl;
         if(dotproduct>0) //point extends end//point is before start
            set(start,newpoint);
         else        //point is before start
            set(newpoint,end);
   }



   void getnearpoints(std::list<gatmo_point3D_t> pts, float maxdist){
//    points.clear();
      set(start,end);
      for(std::list<gatmo_point3D_t>::iterator it = pts.begin(); it != pts.end(); it++)
         if(disttoCached(*it) < maxdist){
            points.push_back(*it);
            extend(*it);
            weight+=pow(maxdist-distto(*it),2);  //wtf?
//           cout<<"dist from line ("<<start.x<<", "<<start.y<<" -> "<<end.x<<", "<<end.y<<") to pt: "<<it->x<<", "<<it->y<<" = "<<distto(*it)<<" < "<<maxdist<<endl;

         }
//       else
//        cout<<"dist from line ("<<start.x<<", "<<start.y<<" -> "<<end.x<<", "<<end.y<<") to pt: "<<it->x<<", "<<it->y<<" = "<<distto(*it)<<" > "<<maxdist<<endl;

   }


};







#endif /* GATMOUNITS_H_ */

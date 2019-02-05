/*
 * CONSTRAINTS.h
 *
 *  Created on: Jul 23, 2012
 */

#ifndef CONSTRAINTS_H_
#define CONSTRAINTS_H_

#include "detection.h"

namespace objrec {
//#<camera position x y z>
//0.0948893 0.00734959 1.71569
//<camera rotation matrix3x3 (9 numbers)> <inverse camera rotation matrix3x3 (9 numbers)> <image height> <image width> <camera parameter fx> <fy> <cx> <cy> <Tx> <Ty>
//0.0948893 0.00734959 1.71569 0.721068 -0.108548 0.684308 0.0815587 0.994083 0.0717461 -0.688047 0.00407748 0.725655 0.721068 0.0815587 -0.688047 -0.108548 0.994083 0.00407748 0.684308 0.0717461 0.725655 480 640 525 525 319.5 239.5 0 0
//R = [0.721068 -0.108548 0.684308;0.0815587 0.994083 0.0717461;-0.688047 0.00407748 0.725655];
//tx = atan2(R(3,2),R(3,3));
//ty = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2)); %ty is in range (-pi/2,pi/2)
//tz = atan2(R(2,1),R(1,1));
//Rx = [
//    1 0 0
//    0 cos(tx) -sin(tx)
//    0 sin(tx) cos(tx)
//    ];
//Ry = [
//    cos(ty) 0 sin(ty)
//    0 1 0
//    -sin(ty) 0 cos(ty)
//    ];
//Rz = [
//    cos(tz) -sin(tz) 0
//    sin(tz) cos(tz) 0
//    0 0 1
//    ];
//
//R2 = Rz*Ry*Rx;
//
//fx = 525;
//fy = 525;
//cx = 319.5;
//cy = 239.5;
//Tx = 0;
//Ty = 0;
//ry2 = zeros(480,640);
//for y=1:480
//    for x=1:640
//       ry2(y,x) = 180/pi*asin(-(sin(ty)+cos(ty)*(y-cy-Ty)/fx)/sqrt(1+((y-cy-Ty)/fy)^2+((x-cx-Tx)/fx)^2));
//    end
//end

//valid because ry (aka ty) is in range (-pi/2,pi/2)
//f = ArcSin[-(Sin[t]+Cos[t]*(y-cy-Ty)/fy)/Sqrt[1+((y-cy-Ty)/fy)^2+((x-cx-Tx)/fx)^2]]
//Simplify[D[f,x]]
//Simplify[Solve[D[f,x]==0,x]] ==> x = cx+Tx
//Simplify[Solve[D[f,y]==0,y]] ==> y = (fx^2*(cy + Ty) + fy*(cx^2 + fx^2 + 2*cx*(Tx - x) + (Tx - x)^2)*Cot[t])/fx^2   (... this is quadratic in x)


//MATLAB FOR rx
//fx = 525;
//fy = 525;
//cx = 319.5;
//cy = 239.5;
//Tx = 0;
//Ty = 0;
//x = repmat([0:639],480,1);
//y = repmat([0:479]',1,640);
//t = 535*pi/180;
//z = atan2(x-(x-cx*.1*sin(t))/(1-.1*sin(t)),y-(y-fy*.1*cos(t)-cy*.1*sin(t))/(1-(.1)*sin(t)));
//figure(1)
//contour(z)
//z = atan2(tan(t)*(cx-x),tan(t)*(cy-y)+fy);
//figure(2)
//contour(z)
// Simplify[(y - (y - fy*d*Cos[t]-cy*d*Sin[t])/(1-d*Sin[t])) / (x - (x - cx*d*Sin[t])/(1-d*Sin[t]))]

// z = atan2(tan(t)*(cx-x),tan(t)*(cy-y)+fy)

//intrinsic camera parameters
const float fx = 525, fy = 525;
const float cx = 319.5, cy = 239.5;
const float Tx = 0, Ty = 0;
const int im_height = 480, im_width = 640;
// const float z_correction = 0;//-.15; //todo fix this hack!

class table_constraint {
	float camera_tilt_angle, angle_tolerance;
	float sin_cta, cos_cta, tan_cta, cot_cta;
	float camera_height, camera_height_tolerance;
public:
	float rx(float x, float y) const { return atan2(tan_cta*(cx-x),tan_cta*(cy-y)+fy)*180.0/M_PI; }
	void rx_range(const region& r,
			//output parameters:
			float& min_rx, float& max_rx) const {
		min_rx = std::numeric_limits<float>::infinity();
		max_rx = -std::numeric_limits<float>::infinity();
		float rx_test;
		rx_test = rx(r.min.x,r.min.y); min_rx = std::min(min_rx,rx_test); max_rx = std::max(max_rx,rx_test);
		rx_test = rx(r.min.x,r.max.y); min_rx = std::min(min_rx,rx_test); max_rx = std::max(max_rx,rx_test);
		rx_test = rx(r.max.x,r.min.y); min_rx = std::min(min_rx,rx_test); max_rx = std::max(max_rx,rx_test);
		rx_test = rx(r.max.x,r.max.y); min_rx = std::min(min_rx,rx_test); max_rx = std::max(max_rx,rx_test);
		if(r.min.x<cx && cx<r.max.x) {
			float arg_max_x;
			arg_max_x = floor(cx);
			rx_test = rx(arg_max_x,r.min.y); min_rx = std::min(min_rx,rx_test); max_rx = std::max(max_rx,rx_test);
			rx_test = rx(arg_max_x,r.max.y); min_rx = std::min(min_rx,rx_test); max_rx = std::max(max_rx,rx_test);
			arg_max_x = ceil(cx);
			rx_test = rx(arg_max_x,r.min.y); min_rx = std::min(min_rx,rx_test); max_rx = std::max(max_rx,rx_test);
			rx_test = rx(arg_max_x,r.max.y); min_rx = std::min(min_rx,rx_test); max_rx = std::max(max_rx,rx_test);
		}
	}
	float ry(float x, float y) const {
		float projx = (x - cx - Tx)/fx; //todo: factor out this division
		float projy = (y - cy - Ty)/fy; //todo: factor out this division
		float operand = -(sin_cta+cos_cta*projy)/sqrt(1+projy*projy+projx*projx);
		if(operand>1) {
			if(operand>1.1) { //todo: remove this
				std::cerr << "found a very high operand for asin: " << operand << std::endl;
			}
			operand = 1;
		}
		if(operand<-1) {
			if(operand<-1.1) { //todo: remove this
				std::cerr << "found a very low operand for asin: " << operand << std::endl;
			}
			operand = -1;
		}
		return asin(operand)*180.0/M_PI;
	}
	void ry_range(const region& r,
			//output parameters:
			float& min_ry, float& max_ry) const {
		min_ry = std::numeric_limits<float>::infinity();
		max_ry = -std::numeric_limits<float>::infinity();
		float ry_test;
		ry_test = ry(r.min.x,r.min.y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);
		ry_test = ry(r.min.x,r.max.y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);
		ry_test = ry(r.max.x,r.min.y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);
		ry_test = ry(r.max.x,r.max.y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);
		float arg_max_y;
		float dx;
		dx = Tx-r.min.x;
		arg_max_y = (fx*fx*(cy+Ty)+fy*(cx*cx+fx*fx+2*cx*dx+dx*dx)*cot_cta)/(fx*fx); //todo: factor out this division
		if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
			ry_test = ry(r.min.x,arg_max_y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);
		}
		dx = Tx-r.max.x;
		arg_max_y = (fx*fx*(cy+Ty)+fy*(cx*cx+fx*fx+2*cx*dx+dx*dx)*cot_cta)/(fx*fx); //todo: factor out this division
		if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
			ry_test = ry(r.max.x,arg_max_y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);
		}
		float arg_max_x = cx+Tx;
		if(r.min.x<arg_max_x && arg_max_x<r.max.x) {
			ry_test = ry(arg_max_x,r.max.y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);
			ry_test = ry(arg_max_x,r.min.y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);

			dx = Tx-arg_max_x;
			arg_max_y = (fx*fx*(cy+Ty)+fy*(cx*cx+fx*fx+2*cx*dx+dx*dx)*cot_cta)/(fx*fx); //todo: factor out this division
			if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
				ry_test = ry(arg_max_x,arg_max_y); min_ry = std::min(min_ry,ry_test); max_ry = std::max(max_ry,ry_test);
			}
		}
	}
	float depth(float x, float y) const {
		float projx = (x - cx - Tx)/fx; //todo: factor out this division
		float projy = (y - cy - Ty)/fy; //todo: factor out this division
		return camera_height/(projy*cos_cta+sin_cta)*sqrt(1.0+projx*projx+projy*projy);

		/*
	with z_correction = zc:
	projx = (x-cx-Tx)/fx;
	projy = (y-cy-Ty)/fy;
	px = Cos[t]-Sin[t]*projy;
	py = -projx;
	pz = -Sin[t]-Cos[t]*projy;
	k = -h/(-Sin[t]-Cos[t]*projy);
	Simplify[k*Sqrt[px^2+py^2+(pz+zc)^2]]
		 */
//		return (camera_height*sqrt(1+projx*projx+projy*projy+1*projy*z_correction*cos_cta-2*z_correction*sin_cta))/(projy*cos_cta+sin_cta);
	}
//	float uncorrected_depth(float x, float y) const {
//		float projx = (x - cx - Tx)/fx; //todo: factor out this division
//		float projy = (y - cy - Ty)/fy; //todo: factor out this division
//		return camera_height/(projy*cos_cta+sin_cta)*sqrt(1.0+projx*projx+projy*projy);
//	}
	/*
f = h/((y-cy-Ty)/fy*Cos[t]+Sin[t])*Sqrt[1+((x-cx-Tx)/fx)^2+((y-cy-Ty)/fy)^2]
Simplify[Solve[D[f,x]==0,x]]
f = h/((y-cy-Ty)/fy*Cos[t]+Sin[t])*Sqrt[1+jx2+((y-cy-Ty)/fy)^2]
Simplify[Solve[D[f,y]==0,y]]
	 */
	void depth_range(const region& r,
			//output parameters:
			float& min_depth, float& max_depth) const {
		min_depth = std::numeric_limits<float>::infinity();
		max_depth = -std::numeric_limits<float>::infinity();
		float depth_test;
		depth_test = depth(r.min.x,r.min.y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
		depth_test = depth(r.min.x,r.max.y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
		depth_test = depth(r.max.x,r.min.y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
		depth_test = depth(r.max.x,r.max.y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
		float arg_max_y;
		float projx;
		projx = (r.max.x - cx - Tx)/fx; //todo: factor out this division
		arg_max_y = cy + Ty + fy*(1.0f+projx*projx)*cot_cta;
		if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
			depth_test = depth(r.max.x,arg_max_y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
			depth_test = depth(r.min.x,arg_max_y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
		}
		projx = (r.min.x - cx - Tx)/fx; //todo: factor out this division
		arg_max_y = cy + Ty + fy*(1.0f+projx*projx)*cot_cta;
		if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
			depth_test = depth(r.max.x,arg_max_y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
			depth_test = depth(r.min.x,arg_max_y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
		}

		float arg_max_x = cx+Tx;
		if(r.min.x<arg_max_x && arg_max_x<r.max.x) {
			depth_test = depth(arg_max_x,r.max.y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
			depth_test = depth(arg_max_x,r.min.y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);

			projx = (arg_max_x - cx - Tx)/fx; //todo: factor out this division
			arg_max_y = cy + Ty + fy*(1.0f+projx*projx)*cot_cta;
			if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
				depth_test = depth(r.max.x,arg_max_y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
				depth_test = depth(r.min.x,arg_max_y); min_depth = std::min(min_depth,depth_test); max_depth = std::max(max_depth,depth_test);
			}
		}
	}

public:
	table_constraint(const scene_info& scene, float object_center_height, float angle_tolerance = 12.0, float camera_height_tolerance = 0.0135) :
		camera_tilt_angle(scene.camera_tilt_angle*M_PI/180.0), angle_tolerance(angle_tolerance),
		sin_cta(sin(camera_tilt_angle)),
		cos_cta(cos(camera_tilt_angle)),
		tan_cta(tan(camera_tilt_angle)),
		cot_cta(1.0/tan_cta),
		camera_height(scene.camera_height_above_floor-scene.table_height-object_center_height),
		camera_height_tolerance(camera_height_tolerance/sin_cta) //camera_height_tolerance parameter is measured perpendicular to table surface,
		                                                         //but the class member is measured parallel to line-of-sight
	{}
	region intersect(const region& r) const {
		region intersected = r;
		float min_rx, max_rx;
		rx_range(r,min_rx,max_rx);
		intersected.min.rx = std::max(r.min.rx,min_rx-angle_tolerance);
		intersected.max.rx = std::min(r.max.rx,max_rx+angle_tolerance);

		float min_ry, max_ry;
		ry_range(r,min_ry,max_ry);
		intersected.min.ry = std::max(r.min.ry,min_ry-angle_tolerance);
		intersected.max.ry = std::min(r.max.ry,max_ry+angle_tolerance);

		float min_depth, max_depth;
		depth_range(r,min_depth,max_depth);
		intersected.min.depth = std::max(r.min.depth,min_depth-camera_height_tolerance);
		intersected.max.depth = std::min(r.max.depth,max_depth+camera_height_tolerance);

		return intersected;
	}
	bool is_satisfied(const point& p) const {
		float _rx = rx(p.x,p.y);
		float _ry = ry(p.x,p.y);
		float prx = p.rx;
		float pry = p.ry;
		return
				(_rx-angle_tolerance <= prx && prx <= _rx+angle_tolerance) &&
				(_ry-angle_tolerance <= pry && pry <= _ry+angle_tolerance);
	}
};

struct bounding_box {
	short min_x, max_x, min_y, max_y;
};

const char* bounding_box_description = "<min x> <max x> <min y> <max y>";
std::ostream& operator<<(std::ostream& out, const bounding_box& bb) {
	return out << bb.min_x << " " << bb.max_x << " " << bb.min_y << " " << bb.max_y;
}
std::istream& operator>>(std::istream& in, bounding_box& bb) {
	return in >> bb.min_x >> bb.max_x >> bb.min_y >> bb.max_y;
}

std::istream& read_bounding_box(std::istream& in, bounding_box& bbox, int& line_number) {
	in.exceptions(std::ios::failbit);
	std::istringstream bounding_box_line(nextline(in,line_number));
	try {
		bounding_box_line >> bbox;
	}
	catch(...) {
		std::ostringstream err;
		err << "Error reading bounding box on line " << line_number << "." << std::endl;
		throw std::runtime_error(err.str());
	}
	return in;
}
std::istream& read_bounding_box(std::istream& in, bounding_box& bbox) {
	int line_number = 0;
	return read_bounding_box(in,bbox,line_number);
}


std::string corresponding_bounding_box_file(const std::string& image_file) {
	size_t dot = image_file.find_last_of('.');
	std::string bbox = image_file.substr(0,dot)+".bbox";
	std::ifstream bbox_file(bbox.c_str());
	if(bbox_file) {
		return bbox;
	}
	else {
		return "";
	}
}

template <class additional_constraint>
class bounding_box_constraint {
private:
	bounding_box bbox;
	additional_constraint constraint;
//	float overlap_percent; //todo add this back??
public:
	bounding_box_constraint(const bounding_box& bbox,
			const additional_constraint& constraint = no_constraint()) : bbox(bbox), constraint(constraint) {}
	bool is_satisfied(const point& p) const {
		return
				bbox.min_x <= p.x && p.x <= bbox.max_x &&
				bbox.min_y <= p.y && p.y <= bbox.max_y &&
				constraint.is_satisfied(p);
	}
	region intersect(const region& r) const {
		region intersected = r;
		intersected.min.x = std::max(r.min.x,(float)bbox.min_x);
		intersected.max.x = std::min(r.max.x,(float)bbox.max_x);

		intersected.min.y = std::max(r.min.y,(float)bbox.min_y);
		intersected.max.y = std::min(r.max.y,(float)bbox.max_y);

		return constraint.intersect(intersected);
	}
};

//px = Cos[t]-Sin[t]*jy;
//py = -jx;
//pz = -Sin[t]-Cos[t]*jy;
//k = -h/pz;
//Simplify[Sqrt[k^2*(px^2+py^2+pz^2)]]

//class camera_height_constraint {
//	camera_tilt_constraint ctc;
//	float camera_height, camera_height_tolerance;
//public:
//	camera_height_constraint(float camera_height, float camera_tilt_angle,
//			float camera_height_tolerance = .02f, float angle_tolerance = 2.0f) :
//				ctc(camera_tilt_angle, angle_tolerance), camera_height(camera_height), camera_height_tolerance(camera_height_tolerance) {}
//	bool is_satisfied(const point& p) const {
//		if(ctc.is_satisfied(p)) {
//
//		} else {
//			return false;
//		}
//	}
//};

struct vector3 {
	float x, y, z;
	vector3() { x = y = z = std::numeric_limits<float>::quiet_NaN(); }
	vector3(float x, float y, float z) : x(x), y(y), z(z) {}
	float length() {return sqrt(x*x+y*y+z*z);}
	inline vector3 to_optical() { return vector3(-y,-z,x); }
	inline vector3 from_optical() { return vector3(z,-x,-y); }
};
std::ostream& operator<<(std::ostream& out, const vector3& v) { return out << v.x << " " << v.y << " " << v.z; }
std::istream& operator>>(std::istream& in, vector3& v) { return in >> v.x >> v.y >> v.z; }

struct matrix3x3 {
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
	matrix3x3() {
		m11 = m12 = m13 = m21 = m22 = m23 = m31 = m32 = m33 = std::numeric_limits<float>::quiet_NaN();
	}
};
std::ostream& operator<<(std::ostream& out, const matrix3x3& m) {
	return out
			<< m.m11 << " " << m.m12 << " " << m.m13 << " "
			<< m.m21 << " " << m.m22 << " " << m.m23 << " "
			<< m.m31 << " " << m.m32 << " " << m.m33;
}
std::istream& operator>>(std::istream& in, matrix3x3& m) {
	return in
			>> m.m11 >> m.m12 >> m.m13
			>> m.m21 >> m.m22 >> m.m23
			>> m.m31 >> m.m32 >> m.m33;
}

inline vector3 operator*(const matrix3x3& m, const vector3& v) {
	vector3 r;
	r.x = m.m11*v.x + m.m12*v.y + m.m13*v.z;
	r.y = m.m21*v.x + m.m22*v.y + m.m23*v.z;
	r.z = m.m31*v.x + m.m32*v.y + m.m33*v.z;
	return r;
}
inline vector3 operator+(const vector3& v1, const vector3& v2) {
	return vector3(
			v1.x+v2.x,
			v1.y+v2.y,
			v1.z+v2.z
			);
}
inline vector3 operator-(const vector3& v1, const vector3& v2) {
	return vector3(
			v1.x-v2.x,
			v1.y-v2.y,
			v1.z-v2.z
			);
}
struct vector2 {
	float x, y;
	vector2(float x, float y) : x(x), y(y) {}
};
struct camera_info {
	vector3 pos;
	matrix3x3 rot, inv_rot;
	int height, width;
	double fx, fy;
	double cx, cy;
	double Tx, Ty;
	inline vector3 project_pixel_to_a_3d_vector(const vector2& p) const {
		vector3 ray_optical((p.x-cx-Tx)/fx,(p.y-cy-Ty)/fy,1.0); //in camera's optical frame
//		return inv_rot*ray_optical.from_optical(); //now rotate it into the world frame
		return rot*ray_optical.from_optical(); //now rotate it into the world frame
	}
	inline vector3 project_pixel_to_a_3d_point(const vector2& p) const {
		vector3 ray_optical((p.x-cx-Tx)/fx,(p.y-cy-Ty)/fy,1.0); //in camera's optical frame
//		return inv_rot*(ray_optical.from_optical()-pos); //now rotate it into the world frame
		return rot*ray_optical.from_optical()+pos; //now rotate it into the world frame
	}
	inline vector2 project_3d_point_to_pixel(const vector3& p) const {
//		vector3 ray_optical = (rot*p+pos).to_optical(); //rotate it into the camera's optical frame
		vector3 ray_optical = (inv_rot*(p-pos)).to_optical(); //rotate it into the camera's optical frame
		vector3 proj(
				fx*ray_optical.x                 +cx*ray_optical.z+Tx,
		                         fy*ray_optical.y+cy*ray_optical.z+Ty,
	                                                 ray_optical.z
				);
		return vector2(
				proj.x/proj.z,
				proj.y/proj.z
				);
	}
//	p = {{Cos[t]-Sin[t]*jy},{-jx},{-Sin[t]-Cos[t]*jy}};
//	d = {{0},{0},{c}};
//	invRy ={{Cos[t],0,-Sin[t]},{0,1,0},{Sin[t],0,Cos[t]}};

	inline vector2 project_3d_vector_to_pixel(const vector3& p) const {
//		vector3 ray_optical = (rot*p).to_optical(); //rotate it into the camera's optical frame
		vector3 ray_optical = (inv_rot*p).to_optical(); //rotate it into the camera's optical frame
		vector3 proj(
				fx*ray_optical.x                 +cx*ray_optical.z+Tx,
		                         fy*ray_optical.y+cy*ray_optical.z+Ty,
	                                                 ray_optical.z
				);
		return vector2(
				proj.x/proj.z,
				proj.y/proj.z
				);
	}
};
const char* const camera_info_description =
		"<camera position x y z> <camera rotation matrix3x3 (9 numbers)> <inverse camera rotation matrix3x3 (9 numbers)> <image height> <image width> <camera parameter fx> <fy> <cx> <cy> <Tx> <Ty>";
std::ostream& operator<<(std::ostream& out, const camera_info& ci) {
	return out
			<< ci.pos << " "
			<< ci.rot << " " << ci.inv_rot << " "
			<< ci.height << " " << ci.width << " "
			<< ci.fx << " " << ci.fy << " "
			<< ci.cx << " " << ci.cy << " "
			<< ci.Tx << " " << ci.Ty;
}
std::istream& operator>>(std::istream& in, camera_info& ci) {
	return in
			>> ci.pos
			>> ci.rot >> ci.inv_rot
			>> ci.height >> ci.width
			>> ci.fx >> ci.fy
			>> ci.cx >> ci.cy
			>> ci.Tx >> ci.Ty;
}

std::string corresponding_camera_info_file(const std::string& image_file) {
	size_t dot = image_file.find_last_of('.');
	std::string cam = image_file.substr(0,dot)+".cam";
	std::ifstream cam_file(cam.c_str());
	if(cam_file) {
		return cam;
	}
	else {
		return "";
	}
}

std::istream& read_camera_info(std::istream& in, camera_info& info, int& line_number) {
	in.exceptions(std::ios::failbit);
	std::istringstream camera_info_line(nextline(in,line_number));
	try {
		camera_info_line >> info;
	}
	catch(...) {
		std::ostringstream err;
		err << "Error reading camera info on line " << line_number << "." << std::endl;
		throw std::runtime_error(err.str());
	}
	return in;
}
std::istream& read_camera_info(std::istream& in, camera_info& info) {
	int line_number = 0;
	return read_camera_info(in,info,line_number);
}


inline void upright_angles(const vector2& px, const camera_info& info,
		//output parameters:
		float& ry, float& rx) {
	vector3 p = info.project_pixel_to_a_3d_vector(px);
	ry = asin(p.z/p.length())*180.0/M_PI;
	vector3 p1 = vector3(p.x,p.y,p.z+.1);
	vector2 px1 = info.project_3d_vector_to_pixel(p1);
	rx = atan2(px.x-px1.x,px.y-px1.y)*180.0/M_PI;
}

class level_constraint {
	cv::Mat_<cv::Vec2f> angles;
public:
	level_constraint(const camera_info& info) : angles(cv::Size(info.width,info.height)) {
		vector2 px(0,0);
		for(px.y=0;px.y<info.height;px.y++) {
			for(px.x=0;px.x<info.width;px.x++) {
				cv::Vec2f& angle = angles(px.y,px.x);
				upright_angles(px,info,
						//output parameters:
						angle[0],angle[1]);
			}
		}
	}
	inline void order_views(const std::vector<view>& views,
				//output parameter:
				std::vector<int>& sorted_indexes) const {
		sorted_indexes.clear();
		sorted_indexes.reserve(views.size());
		for(int j=0;j<(int)views.size();j++) sorted_indexes.push_back(j);
	}
	inline region bounding_region(const view& v) const {
		region r;
		r.min.x = r.min.y = std::numeric_limits<float>::infinity();
		r.max.x = r.max.y = -std::numeric_limits<float>::infinity();
		const viewpoint& vp = v.get_viewpoint();
		r.min.depth = vp.min_depth;
		r.max.depth = vp.max_depth;
		r.min.rx = vp.min_rx; r.max.rx = vp.max_rx;
		r.min.ry = vp.min_ry; r.max.ry = vp.max_ry;
		r.min.rz = vp.min_rz; r.max.rz = vp.max_rz;

		float drxz = vp.max_ry - vp.min_ry;
		float dryz = vp.max_rx - vp.min_rx;
		float min_rxz = vp.min_ry-drxz/2.f; //vp.min_rxz;
		float max_rxz = vp.max_ry+drxz/2.f; //vp.max_rxz;
		float min_ryz = vp.min_rx-dryz/2.f; //vp.min_ryz;
		float max_ryz = vp.max_rx+dryz/2.f; //vp.max_ryz;

		vector2 px(0,0);
		for(px.y=0;px.y<angles.rows;px.y++) {
			for(px.x=0;px.x<angles.cols;px.x++) {
				float rxz = angles(px.y,px.x)[0];
				float ryz = angles(px.y,px.x)[1];
				if(
						min_rxz <= rxz && rxz <= max_rxz &&
						min_ryz <= ryz && ryz <= max_ryz) {
					r.min.x = std::min(r.min.x,px.x);
					r.min.y = std::min(r.min.y,px.y);
					r.max.x = std::max(r.max.x,px.x);
					r.max.y = px.y; //std::max(r.max.y,px.y); //since y is always increasing
				}
			}
		}
		return r;
	}
	inline bool is_satisfied(const localization& loc) const {
		//todo: fix this so that it actually checks if the region overlaps with the upright constraint
		//for the viewpoint
		return true;
	}
	inline cv::Mat_<cv::Vec3b> draw(const view& v) const {
		cv::Mat_<cv::Vec3b> im(cv::Size(angles.cols,angles.rows));
		region r;
		r.min.x = r.min.y = std::numeric_limits<float>::infinity();
		r.max.x = r.max.y = -std::numeric_limits<float>::infinity();
		const viewpoint& vp = v.get_viewpoint();
		r.min.depth = vp.min_depth;
		r.max.depth = vp.max_depth;

		float drxz = vp.max_ry - vp.min_ry;
		float dryz = vp.max_rx - vp.min_rx;

		vector2 px(0,0);
		for(px.y=0;px.y<angles.rows;px.y++) {
			for(px.x=0;px.x<angles.cols;px.x++) {
				float rxz = angles(px.y,px.x)[0];
				float ryz = angles(px.y,px.x)[1];
				im(px.y,px.x) = cv::Vec3b(0,0,0);
				if(vp.min_ry-drxz/2.f<=rxz && rxz<vp.max_ry+drxz/2.f) im(px.y,px.x)[0] = 255;
				if(vp.min_rx-dryz/2.f<=ryz && ryz<vp.max_rx+dryz/2.f) im(px.y,px.x)[1] = 255;
			}
		}
		return im;
	}
};



class level_polygon {
public:
	const std::vector<vector2>& vertices;
	const camera_info& camera;
	int min_x, max_x, min_y, max_y;
private:
	float min_z, max_z;
private:
	inline static bool segment_crosses_ray_from_pt_paralell_to_x_axis(const vector2& s0, const vector2& s1, const vector2& pt) {
		if((s0.y<=pt.y)==(s1.y<=pt.y)) return false;
		float p = s0.x + (s1.x-s0.x)*(pt.y-s0.y)/(s1.y-s0.y);
		return p>pt.x;
	}
	//cast a ray from the point along the positive x direction, and count the number of edges that cross it
	inline bool point_in_polygon(const vector2& pt) const {
		int total_crossings = 0;

		if(segment_crosses_ray_from_pt_paralell_to_x_axis(vertices.back(),vertices[0],pt)) total_crossings++;
		for(int j=1;j<(int)vertices.size();j++) {
			if(segment_crosses_ray_from_pt_paralell_to_x_axis(vertices[j],vertices[j-1],pt)) total_crossings++;
		}
		return (total_crossings&1)==1;
	}
	inline void depth_horizontal_surface(const vector2& pixel, float z,
			//output parameters:
			float& min_depth, float& max_depth) const {
		vector3 d = camera.project_pixel_to_a_3d_vector(pixel);

		float depth = (z - camera.pos.z)/d.z;
		vector2 pt(
				camera.pos.x + d.x*depth,
				camera.pos.y + d.y*depth);

		if(point_in_polygon(pt)) {
			min_depth = std::min(min_depth,depth);
			max_depth = std::max(max_depth,depth);
		}
	}
	static inline void line_params(const vector2& s0, const vector2& s1, float& a, float& b, float& c) {
		a = s0.y-s1.y;
		b = s1.x-s0.x;
		c = a*s0.x+b*s0.y;
	}
	static inline vector2 line_intersection(const vector2& s0, const vector2& s1, const vector2& t0, const vector2& t1) {
		float a0, b0, c0;
		line_params(s0,s1,a0,b0,c0);

		float a1, b1, c1;
		line_params(t0,t1,a1,b1,c1);

		//find the intersection of the two lines by inverting the matrix [a0 b0;a1 b1] and multiplying
		float reciprocal_det = 1./(a0*b1-b0*a1);
		vector2 intersection(
				(b1*c0-b0*c1)*reciprocal_det,
				(-a1*c0+a0*c1)*reciprocal_det);
		return intersection;
	}
	inline void depth_edge_vertical_surface(const vector2& s0, const vector2& s1, const vector2& pixel,
			//output parameters:
			float& min_depth, float& max_depth) const {

		vector3 p = camera.project_pixel_to_a_3d_point(pixel);
		vector2 intersection2d = line_intersection(s0,s1,vector2(camera.pos.x,camera.pos.y),vector2(p.x,p.y));

		//check if it is between s0 and s1
		float eps = 1e-3;
		if(
				((s0.x<=s1.x && s0.x-eps<=intersection2d.x && intersection2d.x<=s1.x+eps) ||
				 (s1.x<=s0.x && s1.x-eps<=intersection2d.x && intersection2d.x<=s0.x+eps))
				 &&
				((s0.y<=s1.y && s0.y-eps<=intersection2d.y && intersection2d.y<=s1.y+eps) ||
				 (s1.y<=s0.y && s1.y-eps<=intersection2d.y && intersection2d.y<=s0.y+eps))
				 ) {

			float z = camera.pos.z+(p.z-camera.pos.z)*(intersection2d.x-camera.pos.x)/(p.x-camera.pos.x);
			vector3 intersection(intersection2d.x,intersection2d.y,z);

			//check if it is between z0 and z1
			if(min_z <= intersection.z && intersection.z <= max_z) {
				float depth = (intersection-camera.pos).length();
				min_depth = std::min(min_depth,depth);
				max_depth = std::max(max_depth,depth);

			}
		}
	}
	inline void bounding_box() {
		min_x = std::numeric_limits<int>::max();
		max_x = std::numeric_limits<int>::min();
		min_y = std::numeric_limits<int>::max();
		max_y = std::numeric_limits<int>::min();
		for(int j=0;j<(int)vertices.size();j++) {
			const vector2& v = vertices[j];
			vector2 p0 = camera.project_3d_point_to_pixel(vector3(v.x,v.y,min_z));
			vector2 p1 = camera.project_3d_point_to_pixel(vector3(v.x,v.y,max_z));
			min_x = std::min(std::min(min_x,(int)floor(p0.x)),(int)floor(p1.x));
			max_x = std::max(std::max(max_x,(int)ceil(p0.x)),(int)ceil(p1.x));
			min_y = std::min(std::min(min_y,(int)floor(p0.y)),(int)floor(p1.y));
			max_y = std::max(std::max(max_y,(int)ceil(p0.y)),(int)ceil(p1.y));
		}
		//crop by the image size:
		min_x = std::max(min_x,0);
		max_x = std::min(max_x,camera.width-1);
		min_y = std::max(min_y,0);
		max_y = std::min(max_y,camera.height-1);
	}
public:
	level_polygon(const std::vector<vector2>& vertices, float min_z, float max_z, const camera_info& camera) :
		vertices(vertices), camera(camera), min_z(min_z), max_z(max_z) { bounding_box(); }
	inline void depth_range(const vector2& pixel,
			//output parameters:
			float& min_depth, float& max_depth
			) const {

		min_depth = std::numeric_limits<float>::infinity();
		max_depth = -std::numeric_limits<float>::infinity();

		depth_horizontal_surface(pixel,min_z,min_depth,max_depth); //bottom surface
		depth_horizontal_surface(pixel,max_z,min_depth,max_depth); //top surface
		depth_edge_vertical_surface(vertices.back(),vertices[0],pixel,min_depth,max_depth); //first vertical edge of prism
		for(int j=1;j<(int)vertices.size();j++) {
			depth_edge_vertical_surface(vertices[j-1],vertices[j],pixel,min_depth,max_depth); //rest of vertical edges of prism
		}
	}
private:
	inline void line(cv::Mat_<cv::Vec3b>& image, const vector3& p0, const vector3& p1) {
		vector2 px0 = camera.project_3d_point_to_pixel(p0);
		vector2 px1 = camera.project_3d_point_to_pixel(p1);
		cv::line(image,cv::Point(px0.x,px0.y),cv::Point(px1.x,px1.y),cv::Scalar(0,255,0));
	}
public:
	void draw(cv::Mat_<cv::Vec3b> image) {
		vector3 prev_min_z(vertices[0].x, vertices[0].y, min_z);
		vector3 prev_max_z(vertices[0].x, vertices[0].y, max_z);
		line(image, vector3(vertices.back().x, vertices.back().y, min_z), prev_min_z);
		line(image, vector3(vertices.back().x, vertices.back().y, max_z), prev_max_z);
		line(image, prev_min_z, prev_max_z);
		for(int j=1;j<(int)vertices.size();j++) {
			vector3 this_min_z(vertices[j].x, vertices[j].y, min_z);
			vector3 this_max_z(vertices[j].x, vertices[j].y, max_z);

			line(image, prev_min_z, this_min_z);
			line(image, prev_max_z, this_max_z);
			line(image, this_min_z, this_max_z);

			prev_min_z = this_min_z;
			prev_max_z = this_max_z;
		}
	}
};

class level_polygon_constraint {
private:
	const level_polygon& poly;
	int min_x, max_x, min_y, max_y;
	cv::Mat_<cv::Vec2f> angles, depths;
public:
	level_polygon_constraint(const level_polygon& poly) :
		poly(poly),
		min_x(poly.min_x), max_x(poly.max_x),
		min_y(poly.min_y), max_y(poly.max_y),
		angles(cv::Size(max_x-min_x+1,max_y-min_y+1)),
		depths(cv::Size(max_x-min_x+1,max_y-min_y+1))
	{
		vector2 px(0,0);
		for(px.y=min_y;px.y<=max_y;px.y++) {
			for(px.x=min_x;px.x<=max_x;px.x++) {
				cv::Vec2f& angle = angles(px.y-min_y,px.x-min_x);
				upright_angles(px,poly.camera,
						//output parameters:
						angle[0],angle[1]);

				float min_depth = std::numeric_limits<float>::infinity();
				float max_depth = -std::numeric_limits<float>::infinity();
				poly.depth_range(px,
						//output parameters:
						min_depth,max_depth);
				cv::Vec2f& depth = depths(px.y-min_y,px.x-min_x);
				depth[0] = min_depth;
				depth[1] = max_depth;
			}
		}
	}
	inline void order_views(const std::vector<view>& views,
				//output parameter:
				std::vector<int>& sorted_indexes) const {
		sorted_indexes.clear();
		sorted_indexes.reserve(views.size());
		for(int j=0;j<(int)views.size();j++) sorted_indexes.push_back(j);
	}
	inline region bounding_region(const view& v) const {
		region r;
		r.min.x = r.min.y = std::numeric_limits<float>::infinity();
		r.max.x = r.max.y = -std::numeric_limits<float>::infinity();
		const viewpoint& vp = v.get_viewpoint();
		r.min.depth = vp.min_depth;
		r.max.depth = vp.max_depth;
		r.min.rx = vp.min_rx; r.max.rx = vp.max_rx;
		r.min.ry = vp.min_ry; r.max.ry = vp.max_ry;
		r.min.rz = vp.min_rz; r.max.rz = vp.max_rz;

		float drxz = vp.max_ry - vp.min_ry;
		float dryz = vp.max_rx - vp.min_rx;
		float min_rxz = vp.min_ry-drxz/2.f; //vp.min_rxz;
		float max_rxz = vp.max_ry+drxz/2.f; //vp.max_rxz;
		float min_ryz = vp.min_rx-dryz/2.f; //vp.min_ryz;
		float max_ryz = vp.max_rx+dryz/2.f; //vp.max_ryz;

		int width = max_x-min_x+1;
		int height = max_y-min_y+1;
		for(int y=0;y<height;y++) {
			for(int x=0;x<width;x++) {
				float rxz = angles(y,x)[0];
				if(min_rxz <= rxz && rxz <= max_rxz) {
					float ryz = angles(y,x)[1];
					if(min_ryz <= ryz && ryz <= max_ryz) {
						float min_depth = depths(y,x)[0];
						if(min_depth!=std::numeric_limits<float>::infinity()) {
							float max_depth = depths(y,x)[1];
							r.min.depth = std::min(r.min.depth,min_depth);
							r.max.depth = std::max(r.max.depth,max_depth);
							float pixel_x = min_x+x;
							float pixel_y = min_y+y;
							r.min.x = std::min(r.min.x,pixel_x);
							r.min.y = std::min(r.min.y,pixel_y);
							r.max.x = std::max(r.max.x,pixel_x);
							r.max.y = pixel_y; //std::max(r.max.y,pixel_y); //since y is always increasing
						}
					}
				}
			}
		}
		return r;
	}
	inline bool is_satisfied(const localization& loc) const {
		//todo: fix this so it actually tests to see if the region overlaps with the polygon
		return true;
	}
	inline cv::Mat_<cv::Vec3b> draw(const view& v) const {
		cv::Mat_<cv::Vec3b> im(cv::Size(angles.cols,angles.rows));
		region r;
		r.min.x = r.min.y = std::numeric_limits<float>::infinity();
		r.max.x = r.max.y = -std::numeric_limits<float>::infinity();
		const viewpoint& vp = v.get_viewpoint();
		r.min.depth = vp.min_depth;
		r.max.depth = vp.max_depth;

		float drxz = vp.max_ry - vp.min_ry;
		float dryz = vp.max_rx - vp.min_rx;

		vector2 px(0,0);
		for(px.y=0;px.y<angles.rows;px.y++) {
			for(px.x=0;px.x<angles.cols;px.x++) {
				float rxz, ryz;
				upright_angles(px,poly.camera,
						//output parameters:
						rxz,ryz);
				float min_depth = std::numeric_limits<float>::infinity();
				float max_depth = -std::numeric_limits<float>::infinity();
				poly.depth_range(px,
						//output parameters:
						min_depth,max_depth);

				im(px.y,px.x) = cv::Vec3b(0,0,0);
				if(vp.min_ry-drxz/2.f<=rxz && rxz<vp.max_ry+drxz/2.f) im(px.y,px.x)[0] = 255;
				if(vp.min_rx-dryz/2.f<=ryz && ryz<vp.max_rx+dryz/2.f) im(px.y,px.x)[1] = 255;
				if(min_depth!=std::numeric_limits<float>::infinity()) im(px.y,px.x)[2] = 255;
			}
		}
		return im;
	}
};

//class bounding_box_constraint {
//private:
//	bounding_box bbox;
//	float overlap_percent;
//	struct index_with_quality_of_match {
//		int index;
//		float quality_of_match;
//		static bool compare(const index_with_quality_of_match& iq1, const index_with_quality_of_match& iq2) {
//			return iq1.quality_of_match < iq2.quality_of_match;
//		}
//	};
//public:
//	bounding_box_constraint(const bounding_box& bbox, float overlap_percent=0.5) : bbox(bbox), overlap_percent(overlap_percent) {}
//	inline void order_views(const std::vector<view>& views,
//			//output parameter:
//			std::vector<int>& sorted_indexes) const {
//		float bbox_ratio = (float)(bbox.y1-bbox.y0)/(float)(bbox.x1-bbox.x0);
//		std::vector<index_with_quality_of_match> iqs;
//		iqs.reserve(views.size());
//		for(int j=0;j<(int)views.size();j++) {
//			index_with_quality_of_match iq;
//			iq.index = j;
//			float x0, x1, y0, y1;
//			point p;
//			p.x = 0; p.y = 0; p.depth = 1;
//			const viewpoint& vp = views[j].get_viewpoint();
//			p.rx = (vp.min_rx+vp.max_rx)*.5f;
//			p.ry = (vp.min_ry+vp.max_ry)*.5f;
//			p.rz = (vp.min_rz+vp.max_rz)*.5f;
//			get_bounding_box(views[j],p,x0,x1,y0,y1);
//			float view_ratio = (y1-y0)/(x1-x0);
//			iq.quality_of_match = fabs(view_ratio-bbox_ratio);
//			iqs.push_back(iq);
//		}
//		std::sort(iqs.begin(),iqs.end(),index_with_quality_of_match::compare);
//		sorted_indexes.clear();
//		sorted_indexes.reserve(iqs.size());
//		for(int j=0;j<(int)iqs.size();j++) {
//			sorted_indexes.push_back(iqs[j].index);
//		}
//	}
//	inline region bounding_region(const view& v) const {
//		float expand_search_region = 0.5;
//		float expand_x = ((float)bbox.x1 - (float)bbox.x0)*expand_search_region;
//		float expand_y = ((float)bbox.y1 - (float)bbox.y0)*expand_search_region;
//		float expanded_x0, expanded_x1, expanded_y0, expanded_y1;
//		expanded_x0 = bbox.x0 - expand_x;
//		expanded_x1 = bbox.x1 + expand_x;
//		expanded_y0 = bbox.y0 - expand_y;
//		expanded_y1 = bbox.y1 + expand_y;
//		const viewpoint& vp = v.get_viewpoint();
//		return region(
//				point(expanded_x0,expanded_y0,vp.min_depth,vp.min_rx,vp.min_ry,vp.min_rz),
//				point(expanded_x1,expanded_y1,vp.max_depth,vp.max_rx,vp.max_ry,vp.max_rz));
//	}
//	inline bool is_satisfied(const localization& loc) const {
////		!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!return bounding_boxes_overlap(loc,bbox.x0,bbox.x1,bbox.y0,bbox.y1,overlap_percent);
//		return true;
//	}
//	inline cv::Mat_<cv::Vec3b> draw() const {
//		cv::Mat_<cv::Vec3b> im(cv::Size(640,480));
//		set_to(im,cv::Vec3b(0,0,0));
//		cv::line(im,cv::Point(bbox.x0,bbox.y0),cv::Point(bbox.x1,bbox.y0),cv::Scalar(255,255,255));
//		cv::line(im,cv::Point(bbox.x1,bbox.y0),cv::Point(bbox.x1,bbox.y1),cv::Scalar(255,255,255));
//		cv::line(im,cv::Point(bbox.x1,bbox.y1),cv::Point(bbox.x0,bbox.y1),cv::Scalar(255,255,255));
//		cv::line(im,cv::Point(bbox.x0,bbox.y1),cv::Point(bbox.x0,bbox.y0),cv::Scalar(255,255,255));
//		return im;
//	}
//
//	inline cv::Mat_<cv::Vec3b> draw(const view& v) const { return draw(); }
//};

} //namespace objrec

#endif /* CONSTRAINTS_H_ */

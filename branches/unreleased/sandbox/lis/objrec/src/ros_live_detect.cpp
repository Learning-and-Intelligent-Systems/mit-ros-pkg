/*
 * ros_live_detect.cpp
 *
 *  Created on: Mar 22, 2011
 */

#include "ros_kinect.h"
#include "gui.h"

//this is no longer used, but may be brought back to give accurate pose information for a detection
//detection detection::transform(const tf::Transform& c0Tc1, const sensor_msgs::CameraInfo& camera_info) {
//	//Goal: Find the new location of the particle given the change in the camera's pose
//	//
//	// 1. Calculate the ray of possible positions of the object in the optical frame (using PinholeCameraModel::projectPixelTo3dRay)
//	// 2. Transform the ray into the wide_stereo_link frame
//	// 3. Find the intersection of the ray and the sphere with the correct x-distance to find the object's position in the wide_stereo_link frame
//	// 4. Incorporate the rotation information
//	// 5. Transform the object pose (position+orientation) back to the optical frame
//	// 6. Transform the object pose in the optical frame based on how the camera has moved
//	// 7. Use PinholeCameraModel::project3dToPixel to find the new pixel location of the particle
//	// 8. transform the object pose in the moved camera frame to the wide stereo frame
//	// 9. determine the x-distance of the object in the wide_stereo_frame
//	// 10. get the new Euler angles using btMatrix3x3::getEulerYPR (after constructing a btMatrix3x3 from a quaternion)
//
//
//	// 1. Calculate the ray of possible positions of the object in the optical frame (using PinholeCameraModel::projectPixelTo3dRay)
//	image_geometry::PinholeCameraModel phc;
//	phc.fromCameraInfo(camera_info);
//	cv::Point3d ray_opt = phc.projectPixelTo3dRay(cv::Point2d(x, y));
//
//	// 2. Find the intersection of the ray and the sphere with the correct x-distance to find the object's position in the wide_stereo_link frame
//	double len = sqrt(ray_opt.x*ray_opt.x+ray_opt.y*ray_opt.y+ray_opt.z*ray_opt.z);
//	//		double dist = (max_x+min_x)/2.0;
//	tf::Vector3 ray_opt_v(
//			(ray_opt.x+phc.Tx()/phc.fx())*dist/len-phc.Tx()/phc.fx(),
//			(ray_opt.y+phc.Ty()/phc.fy())*dist/len-phc.Ty()/phc.fy(),
//			ray_opt.z*dist/len);
//
//	// 3. Transform the ray into the wide_stereo_link frame
//	tf::Transform from_optical(tf::Quaternion(-0.5,0.5,-0.5,0.5));
//	tf::Transform to_optical(tf::Quaternion(0.5,-0.5,0.5,0.5));
//	tf::Vector3 ray = from_optical*ray_opt_v;
//
//
//	tf::Transform rotation(tf::Quaternion(qx,qy,qz,qw));
//	tf::Transform position(tf::Quaternion(0.0,0.0,0.0,1.0),ray);
//	tf::Transform c0To_non_optical = position*rotation;
//
//	// 5. Transform the object pose (position+orientation) back to the optical frame
//
//	tf::Transform c0To = to_optical*c0To_non_optical;
//
//	// 6. Transform the object pose in the optical frame based on how the camera has moved
//
//	//particle locations for a moving camera:
//	// When the camera pose has moved, the particles should also move in a corresponding way.
//	//
//	// Given the following transforms:
//	//  c0To (where the object was in the camera's frame)
//	//  c0Tc1 (where the camera is in the frame of where it was)
//	//
//	// Find the following transform:
//	//  c1To (where the object is in the camera's frame)
//	//
//	// c1To = c0Tc1.inverse()*c0To
//
//	tf::Transform c1To = c0Tc1.inverse()*c0To;
//	tf::Vector3 obj_pos_in_camera = c1To.getOrigin();
//	tf::Quaternion obj_rot_in_camera = c1To.getRotation();
//	obj_rot_in_camera.normalize();//todo: remove this
//
//	// 7. Use PinholeCameraModel::project3dToPixel to find the new pixel location of the particle
//	cv::Point3d obj_pos_in_camera_p(obj_pos_in_camera.x(),obj_pos_in_camera.y(),obj_pos_in_camera.z());
//	cv::Point2d new_location = phc.project3dToPixel(obj_pos_in_camera_p);
//
//
//	// 8. transform the object pose in the moved camera frame to the wide stereo frame
//	tf::Transform c1To_non_optical = from_optical*c1To;
//
//	// 9. determine the x-distance of the object in the wide_stereo_frame
//	tf::Vector3 new_location_non_optical = c1To_non_optical.getOrigin();
//	double new_dist = sqrt(
//			new_location_non_optical.x()*new_location_non_optical.x()+
//			new_location_non_optical.y()*new_location_non_optical.y()+
//			new_location_non_optical.z()*new_location_non_optical.z()
//	);
//	// 10. get the new Euler angles using btMatrix3x3::getEulerYPR (after constructing a btMatrix3x3 from a quaternion)
//
//
//	//		btMatrix3x3 c1To_non_optical_basis = c1To_non_optical.getBasis();
//
//	detection d;
//	d.logP = logP;
//	tf::Quaternion new_rotation = c1To_non_optical.getRotation();
//	d.qx = new_rotation.x();
//	d.qy = new_rotation.y();
//	d.qz = new_rotation.z();
//	d.qw = new_rotation.w();
//	d.x = new_location.x;
//	d.y = new_location.y;
//	d.dist = new_dist;
//	//don't worry about updating the Euler angles, since they won't be used after transformation
//	return d;
//}




int main(int argc, char** argv) {
	if(argc<2) {
		std::cout << "usage:" << std::endl;
		std::cout << " " << std::string(argv[0]) << " object_file" << std::endl;
		std::cout << std::endl;
		std::cout << " Detects objects in live images from a PR2." << std::endl;
		return 0;
	}
	ros::init(argc, argv, "ros_live_detect");

	std::ifstream in(argv[1]);
	objrec::object obj;
	in >> obj;
	std::cout << "Loaded " << obj.name << " with " << obj.views.size() << " views." << std::endl;

	objrec::ros_kinect camera;

	bool visualize_search = false;
	while(true) {
		cv::Mat_<cv::Vec3b> image;
		cv::Mat_<float> depth;
		objrec::camera_info info;
		camera.get(image,depth,info);

		cv::Mat_<cv::Vec3b> depth_image = objrec::mat_bgr(depth);
		cv::Mat_<cv::Vec3b> edges = objrec::edges_bgr(image,depth,obj.visual_words.get_edges());
		cv::imshow("depth",depth_image);
		cv::imshow("edges",edges);
		cv::imshow("image",image);
		int key = cv::waitKey(100);
		switch(key) {
		case 113: //q
		case 1048689: //q with numlock
		case 27: //esc
		case 1048603: //esc with numlock
			return 0;
		case 32: //space
		case 1048608: //space with numlock
			visualize_search = !visualize_search;
			if(visualize_search) {
				std::cout << "Search visualization: ON" << std::endl;
			}
			else {
				std::cout << "Search visualization: OFF" << std::endl;
			}
		default:
			break;
		}

		objrec::image im(obj.visual_words,image,depth);
		bool print_progress = true;

		objrec::level_constraint constr(info);
		std::vector<objrec::localization> locs;
		float lower_bound = -std::numeric_limits<float>::infinity();
		bool only_top_detection = true;
		objrec::localizations<objrec::level_constraint>(obj,im,lower_bound,locs,only_top_detection,
				print_progress,visualize_search,0,obj.views.size(),constr);

		std::cout << "Localization: log probability: " << locs[0].log_probability;
		const objrec::viewpoint& vp = locs[0].v->get_viewpoint();
		std::cout << " pixel: (" << locs[0].location.x << "," << locs[0].location.y << ")";
		std::cout << " depth: " << locs[0].location.depth;
		std::cout << " Rx: " << (vp.min_rx+vp.max_rx)/2.0f;
		std::cout << " Ry: " << (vp.min_ry+vp.max_ry)/2.0f;
		std::cout << " Rz: " << (vp.min_rz+vp.max_rz)/2.0f;
		std::cout << std::endl;
		objrec::draw(locs[0],im,image);
		cv::imshow("detection",image);
	}
}

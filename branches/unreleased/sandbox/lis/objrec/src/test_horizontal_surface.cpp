/*
 * test_horizontal_surface.cpp
 *
 *  Created on: Jul 24, 2012
 */

#include "constraints.h"
#include "gui.h"

int main(int argc, char** argv) {
	std::string image_file = "tmp/mat_background/0002.png";
	objrec::object obj;
	std::ifstream objin("campbells_can.txt");
	objin >> obj;
	std::string depth_file = objrec::corresponding_depth_file(image_file);
	std::string camera_info_file = objrec::corresponding_camera_info_file(image_file);

	cv::Mat_<cv::Vec3b> picture = cv::imread(image_file);
	cv::Mat_<float> depth = objrec::read_depth(depth_file);
	objrec::camera_info info;
	std::ifstream in(camera_info_file.c_str());
	objrec::read_camera_info(in,info);

	objrec::vector2 p(320,240);

	std::cout << "2D point: " << p.x << "," << p.y << std::endl;
	float dpth = depth(p.y,p.x);
	std::cout << "depth at 2d point: " << dpth << std::endl;
	objrec::vector3 v3d = info.project_pixel_to_a_3d_vector(p);
	double len = v3d.length();
	v3d.x *= dpth/len;
	v3d.y *= dpth/len;
	v3d.z *= dpth/len;

	objrec::vector3 p3d = info.pos+v3d;

	std::cout << "3D point: " << p3d.x << "," << p3d.y << "," << p3d.z << std::endl;


	double w = 1.2065;
	double h = .5969;

	std::vector<objrec::vector2> vertices;
	vertices.push_back(objrec::vector2(.7+h/2.,-w/2.));
	vertices.push_back(objrec::vector2(.7+h/2., w/2.));
	vertices.push_back(objrec::vector2(.7-h/2., w/2.));
	vertices.push_back(objrec::vector2(.7-h/2.,-w/2.));
	float min_z = .677573+.01;
	float max_z = .677573+.04;
	objrec::level_polygon poly(vertices,min_z,max_z,info);


//	objrec::upright_constraint constr(info);
	objrec::level_polygon_constraint constr(poly);
	float lower_bound = -410;//-std::numeric_limits<float>::infinity();
	objrec::feature_library library;
	objrec::image im(library,picture,depth);

	poly.draw(picture);
	cv::imshow("picture",picture);

	std::vector<objrec::localization> locs;
	bool only_top_detection = true, print_progress = true, visualize_search = false;
	objrec::localizations<objrec::level_polygon_constraint>(obj,im,lower_bound,locs,only_top_detection,
			print_progress,visualize_search,0,obj.views.size(),constr);

	std::cout << "localization: " << locs[0] << std::endl;
	objrec::draw(locs[0],im,picture);

	float min_depth, max_depth;
	poly.depth_range(p,min_depth,max_depth);
	std::cout << "depth at " << p.x << "," << p.y << " ranges from " << min_depth << " to " << max_depth << std::endl;

	cv::Mat_<cv::Vec3b> depth_image = objrec::mat_bgr(depth);
	cv::Mat_<cv::Vec3b> edges = objrec::edges_bgr(picture,depth,obj.visual_words.get_edges());
	cv::line(depth_image,cv::Point(322,240),cv::Point(318,240),cv::Scalar(0,255,0));
	cv::line(depth_image,cv::Point(320,242),cv::Point(320,238),cv::Scalar(0,255,0));
	cv::imshow("depth",depth_image);
	cv::imshow("edges",edges);
	cv::imshow("picture",picture);

	int j = 100;
	cv::imshow("view",objrec::view_image(obj.views[j]));
	cv::imshow("upright",constr.draw(obj.views[j]));
	cv::waitKey(0);
}

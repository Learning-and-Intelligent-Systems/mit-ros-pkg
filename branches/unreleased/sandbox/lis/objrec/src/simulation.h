/*
 * simulation.h
 *
 *  Created on: Sep 4, 2012
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <GL/glx.h>

namespace objrec {

class OpenGL_X_window {
	Window win;
	Display *dpy;
	GLXContext ctx;
public:
	OpenGL_X_window(int width=640, int height=480);
	void swap_buffers();
};

struct mesh {
	GLuint display_list_number_with_textures;
	GLuint display_list_number_no_textures;
	double radius;
	double origin_height_above_table;
	bool render(
			//where to render the mesh:
			OpenGL_X_window& w,
			//textures
			bool use_textures,
			//object position:
			double rx, double ry, double rz, double distance, double screen_updown, double screen_leftright,
			//light position:
			double lx, double ly, double lz,
			//ambient light ranges from 0.0 to 1.0
			double ambient,
			//output parameters:
			cv::Mat_<cv::Vec3b>& im, cv::Mat_<float>& depth, double& xc, double& yc) const;
};

mesh load_obj(const std::string& obj_file_name);
mesh load_off(const std::string& off_file_name);


//class mesh_model {
//private:
//	std::string mesh_file_name;
//	GLint id_without_table, id_with_table; //two display lists, one does not include the table under the object, and the other does.
//	GLdouble bounding_radius, bottom_z_value;
//	Window win;
//	GLXContext ctx;
//	Display *dpy;
//
//	static const GLint width = 640, height = 480;
//	static const GLdouble near = 0.5, far = 5.0;
//	GLdouble left, bottom;
//
//public:
//	mesh_model(const std::string& off_file);
//	~mesh_model();
//	double half_FOV_angle_up_down() const { return atan(-left/near)*180./M_PI; }
//	double half_FOV_angle_left_right() const { return atan(-bottom/near)*180./M_PI; }
//	const std::string& get_name() const {return mesh_file_name;}
//	double get_radius() const {return bounding_radius;}
//	bool generate_synthetic_image(double rx, double ry, double rz, double distance, double screen_updown, double screen_leftright,
//				      double lx, double ly, double lz,
//			//output parameters:
//			cv::Mat_<cv::Vec3b>& cropped_im, cv::Mat_<float>& cropped_depth,
//			cv::Mat_<cv::Vec3b>& cropped_im_table, cv::Mat_<float>& cropped_depth_table) const;
//};

} //namespace objrec

#endif /* SIMULATION_H_ */

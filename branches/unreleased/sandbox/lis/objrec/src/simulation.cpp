/*
 * simulation.cpp
 *
 *  Created on: Aug 28, 2012
 *  Largely based on glxgears.c
 */

#include "simulation.h"
#include "io.h"
#include <stdexcept>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>

namespace objrec {

/*
 * Create an RGB, double-buffered window.
 * Return the window and context handles.
 */
void make_window(const char *name, int width, int height, Window& win, GLXContext& ctx, Display*& dpy) {
	char *dpyName = NULL;
	dpy = XOpenDisplay(dpyName);
	if (!dpy) {
		std::ostringstream err;
		err << "Error: couldn't open X Windows display " << dpyName << ". Try logging out and logging back in.";
		throw std::runtime_error(err.str());
	}

	int attrib[] = { GLX_RGBA,
			GLX_RED_SIZE, 1,
			GLX_GREEN_SIZE, 1,
			GLX_BLUE_SIZE, 1,
			GLX_DOUBLEBUFFER,
			GLX_DEPTH_SIZE, 1,
			None };
	int scrnum;
	XSetWindowAttributes attr;
	unsigned long mask;
	Window root;
	XVisualInfo *visinfo;

	scrnum = DefaultScreen( dpy );
	root = RootWindow( dpy, scrnum );

	visinfo = glXChooseVisual( dpy, scrnum, attrib );
	if (!visinfo) {
		throw std::runtime_error("Error: couldn't get an RGB, Double-buffered visual. If this machine has no GPU, maybe try running with 'xvfb-run -a'?");
	}

	/* window attributes */
	attr.background_pixel = 0;
	attr.border_pixel = 0;
	attr.colormap = XCreateColormap( dpy, root, visinfo->visual, AllocNone);
	attr.event_mask = StructureNotifyMask | ExposureMask | KeyPressMask;
	mask = CWBackPixel | CWBorderPixel | CWColormap | CWEventMask;

	win = XCreateWindow( dpy, root, 0, 0, width, height,
			0, visinfo->depth, InputOutput,
			visinfo->visual, mask, &attr );

	/* set hints and properties */
	{
		XSizeHints sizehints;
		sizehints.x = 0;
		sizehints.y = 0;
		sizehints.width  = width;
		sizehints.height = height;
		sizehints.flags = USSize | USPosition;
		XSetNormalHints(dpy, win, &sizehints);
		XSetStandardProperties(dpy, win, name, name,
				None, (char **)NULL, 0, &sizehints);
	}

	ctx = glXCreateContext( dpy, visinfo, NULL, True );
	if (!ctx) {
		throw std::runtime_error("Error: glXCreateContext failed");
	}

	XFree(visinfo);
	XMapWindow(dpy, win);
	glXMakeCurrent(dpy, win, ctx);
}

OpenGL_X_window::OpenGL_X_window(int width, int height) {
	make_window("OpenGL / X windows (GLX) drawn by local graphics card where viewed",width,height,win,ctx,dpy);
	glViewport(0, 0, (GLint) width, (GLint) height);

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
}
void OpenGL_X_window::swap_buffers() { glXSwapBuffers(dpy, win); }

struct vertex {
	float x,y,z,r,g,b;
	vertex(const std::string& parameters) {
		std::istringstream in(parameters.c_str());
		in >> x >> y >> z >> r >> g >> b;
		if(in.fail()) {
			r = g = b = std::numeric_limits<float>::quiet_NaN();
		}
	}
	inline vertex operator-(const vertex& other) const { return vertex(x-other.x,y-other.y,z-other.z); }
	inline vertex cross(const vertex& other) const {
		return vertex(
				y*other.z-z*other.y,
				z*other.x-x*other.z,
				x*other.y-y*other.x
				);
	}
private:
	vertex(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct texture_coordinate {
	float s,t;
	texture_coordinate(const std::string& parameters) {
		std::istringstream in(parameters.c_str());
		in >> s >> t;
	}
};


void parse_line(std::istream& in, int& line_number,
		//output arguments:
		std::string& command, std::string& parameters) {
	std::string line = nextline(in,line_number);

	size_t command_begin = line.find_first_not_of(" \t\r");
	size_t command_end = line.find_first_of(" \t\r",command_begin);
	if(command_end==std::string::npos) {
		command = line.substr(command_begin);
		parameters = "";
	} else {
		command = line.substr(command_begin,command_end-command_begin);
		parameters = line.substr(command_end);
	}
}

std::string trim(const std::string& s) {
	size_t begin = s.find_first_not_of(" \r\t\n");
	size_t end = s.find_last_of(" \r\t\n");
	if(end==std::string::npos) {
		return s.substr(begin);
	} else {
		return s.substr(begin,end-begin);
	}
}


struct index { int vertex_ind, texture_coordinate_ind; };

typedef GLuint texture;

struct face {
	std::vector<index> ind;
	texture t;
};

class obj_file_loader {
	std::vector<vertex> vertices;
	std::vector<texture_coordinate> texture_coordinates;
	std::vector<face> faces;
	std::map<std::string,texture> materials;
	bool has_textures;
	mesh m;

	texture load_mtl(const std::string& path, const std::string& name, std::istream& in,
			int& line_number) {
		texture tex;
		while(in.good()) {
			std::streampos position = in.tellg();
			int last_line_number = line_number;
			std::string command_token, parameters;
			parse_line(in, line_number, command_token, parameters);

			if(command_token=="map_Kd") {
				size_t token_begin = parameters.find_first_not_of(" \r\t");
				size_t token_end = parameters.find_first_of(" \r\t",token_begin);
				std::string image_file_name;
				if(token_end==std::string::npos) {
					image_file_name = parameters.substr(token_begin);
				} else {
					image_file_name = parameters.substr(token_begin,token_end-token_begin);
				}
				cv::Mat_<cv::Vec3b> im = cv::imread(path+"/"+image_file_name);
				cv::flip(im,im,0);
//				cv::imshow("texture",im);
//				cv::waitKey(0);
				if(!has_textures) {
					has_textures = true;
					glEnable(GL_TEXTURE_2D);
					glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
//					glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
				}
				glGenTextures(1, &tex);
				glBindTexture(GL_TEXTURE_2D, tex);
			    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
			    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
			    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, im.cols, im.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, im.data);
			} else if(command_token=="newmtl") {
				//back up before this line so this material next, and finish this material
				in.seekg((long)position,in.beg);
				line_number = last_line_number;
				break;
			} else if(command_token=="Ka" || command_token=="Kd" || command_token=="Ks"
					|| command_token=="Tr" || command_token=="illum" || command_token=="Ns") {
				//ignore these commands
			} else {
				std::ostringstream err;
				err << "unrecognized command '" << command_token << "' on line " << line_number;
				throw std::runtime_error(err.str());
			}
		}
		return tex;
	}

	void parse_mtl_file(const std::string& path, const std::string& mtl_file) {
		std::string full_mtl_file = path + '/' + mtl_file;
		std::ifstream in(full_mtl_file.c_str());
		if(!in) throw std::runtime_error("Could not access material template library file '"+mtl_file+"'.");
		int line_number = 0;
		while(in.good()) {
			try {
				std::string command_token, parameters;
				parse_line(in, line_number, command_token, parameters);
				if(command_token=="newmtl") {
					std::string name = trim(parameters);
					texture t = load_mtl(path,name,in,line_number);
					materials.insert(std::pair<std::string,texture>(name,t));
				}
			} catch(std::runtime_error& e) {
				std::ostringstream err;
				err << e.what() << " of " << mtl_file;
				throw std::runtime_error(err.str());
			}
		}
	}
	index parse_index(const std::string& parameters, size_t& token_begin) {
		index ind;
		size_t token_end = parameters.find_first_of(" \r\t",token_begin);
		size_t slash1 = parameters.find_first_of('/',token_begin);
		if(slash1==std::string::npos) { //just a vertex index
			std::istringstream in(parameters.substr(token_begin).c_str());
			in >> ind.vertex_ind;
			if(ind.vertex_ind<0) {
				std::ostringstream err;
				err << "negative vertex indexing (" << ind.vertex_ind << ") is not supported";
				throw std::runtime_error(err.str());
			} else if((unsigned int)ind.vertex_ind-1>=this->vertices.size()) {
				std::ostringstream err;
				err << "vertex index (" << ind.vertex_ind << ") out of bounds, maximum index: " << this->vertices.size();
				throw std::runtime_error(err.str());
			}
			ind.texture_coordinate_ind = 0;
		} else {
			size_t slash2 = parameters.find_first_of('/',slash1+1);
			if(slash2==std::string::npos || slash2>token_end) { // vertex/texture coordinate
				std::istringstream in1(parameters.substr(token_begin).c_str());
				in1 >> ind.vertex_ind;
				if(ind.vertex_ind<0) {
					std::ostringstream err;
					err << "negative vertex indexing (" << ind.vertex_ind << ") is not supported";
					throw std::runtime_error(err.str());
				} else if((unsigned int)ind.vertex_ind-1>=this->vertices.size()) {
					std::ostringstream err;
					err << "vertex index (" << ind.vertex_ind << ") out of bounds, maximum index: " << this->vertices.size();
					throw std::runtime_error(err.str());
				}
				std::istringstream in2(parameters.substr(slash1+1).c_str());
				in2 >> ind.texture_coordinate_ind;
				if(ind.texture_coordinate_ind<0) {
					std::ostringstream err;
					err << "negative vertex texture indexing (" << ind.texture_coordinate_ind << ") is not supported";
					throw std::runtime_error(err.str());
				} else if((unsigned int)ind.texture_coordinate_ind-1>=this->texture_coordinates.size()) {
					std::ostringstream err;
					err << "vertex texture index (" << ind.texture_coordinate_ind << ") out of bounds, maximum index: "
							<< this->texture_coordinates.size();
					throw std::runtime_error(err.str());
				}
			}
			else throw std::runtime_error("unsupported normal index in obj file");
		}

		token_begin = parameters.find_first_not_of(" \r\t",token_end);
		return ind;
	}
	inline face parse_face(const std::string& parameters, const texture& current_texture) {
		face f;
		f.t = current_texture;
		size_t token_begin = parameters.find_first_not_of(" \r\t");
		f.ind.push_back(parse_index(parameters,token_begin));
		if(token_begin==std::string::npos) throw std::runtime_error("face has only 1 vertex");
		f.ind.push_back(parse_index(parameters,token_begin));
		if(token_begin==std::string::npos) throw std::runtime_error("face has only 2 vertices");
		while(token_begin!=std::string::npos) {
			f.ind.push_back(parse_index(parameters,token_begin));
		}
		return f;
	}
	void draw_face(const face& f, bool use_textures) {
		glBegin(GL_POLYGON);
		vertex t = vertices[f.ind[0].vertex_ind-1];
		vertex u = vertices[f.ind[1].vertex_ind-1];
		vertex v = vertices[f.ind[2].vertex_ind-1];
		vertex normal = (u-t).cross(v-u);
		glNormal3f(normal.x,normal.y,normal.z);
		for(std::vector<index>::const_iterator j=f.ind.begin();j!=f.ind.end();j++) {
			vertex& v = vertices[j->vertex_ind-1];
			if(use_textures) {
				int vt_ind = j->texture_coordinate_ind;
				if(vt_ind!=0) {
					texture_coordinate& vt = texture_coordinates[vt_ind];
					glTexCoord2d(vt.s,vt.t);
				}
				if(v.r==v.r) {
					glColor3f(v.r,v.g,v.b);
				}
			}
			glVertex3f(v.x/100.,v.y/100.,v.z/100.);
			m.radius = std::max(m.radius,sqrt(v.x*v.x+v.y*v.y+v.z*v.z)/100.);
			m.origin_height_above_table = std::max(m.origin_height_above_table,-v.z/100.);
		}
		glEnd(); //GL_POLYGON
	}
public:
	obj_file_loader(const std::string& obj_file) : has_textures(false) {
		m.radius = 0.0;
		m.origin_height_above_table = -std::numeric_limits<double>::infinity();
		std::ifstream in(obj_file.c_str());
		if(!in) throw std::runtime_error("Could not access mesh file '"+obj_file+"'.");
		int line_number = 0;

		texture current_texture;
		current_texture = std::numeric_limits<GLuint>::max();
		while(in.good()) {
			try {
				std::string command_token, parameters;
				parse_line(in, line_number, command_token, parameters);
				if(command_token=="v") vertices.push_back(vertex(parameters));
				else if(command_token=="vt") texture_coordinates.push_back(texture_coordinate(parameters));
				else if(command_token=="f") faces.push_back(parse_face(parameters,current_texture));
				else if(command_token=="g") {} //begin a new group... ignore this?
				else if(command_token=="mtllib") {
					std::string mtl_file = trim(parameters);
					std::string path;
					if(mtl_file[0]!='/') {
						path = dirname(obj_file);
					} else {
						path = ".";
					}
					parse_mtl_file(path,mtl_file);
				} else if(command_token=="usemtl") {
					//use a particular material
					std::string mtl = trim(parameters);
					std::map<std::string,texture>::const_iterator m = materials.find(mtl);
					if(m==materials.end()) {
						std::ostringstream err;
						err << "unrecognized material name '" << mtl << "'";
						throw std::runtime_error(err.str());
					}
					current_texture = m->second;
				} else {
					std::ostringstream err;
					err << "unsupported command token '" << command_token << "'";
					throw std::runtime_error(err.str());
				}
			} catch(std::runtime_error& e) {
				std::ostringstream err;
				err << e.what() << " on line " << line_number << " of " << obj_file;
				throw std::runtime_error(err.str());
			}
		}


		//with textures
		m.display_list_number_with_textures = glGenLists(1);
		glNewList(m.display_list_number_with_textures, GL_COMPILE);
		GLfloat color[4] = {1.0,1.0,1.0,1.0};
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color); //todo: test with and without this.
//		glColor3f(1,1,1); // set global color to white, otherwise this color will be (somehow) added to the texture

		current_texture = std::numeric_limits<unsigned int>::max();
		for(std::vector<face>::const_iterator f=faces.begin();f!=faces.end();f++) {
			texture tex = f->t;
			if(tex!=current_texture) {
				current_texture = tex;
				glBindTexture(GL_TEXTURE_2D, tex);
			}
			draw_face(*f,true);
		}
		glEndList();

		//no textures
		m.display_list_number_no_textures = glGenLists(1);
		glNewList(m.display_list_number_no_textures, GL_COMPILE);
//		GLfloat color[4] = {1.0,1.0,1.0,1.0};
//		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
		glColor3f(1,1,1); // set global color to white, otherwise this color will be (somehow) added to the texture

		for(std::vector<face>::const_iterator f=faces.begin();f!=faces.end();f++) {
			draw_face(*f,false);
		}
		glEndList();

	}
	mesh get_mesh() {return m;}
};

mesh load_obj(const std::string& obj_file_name) { return obj_file_loader(obj_file_name).get_mesh(); }

class off_file_loader {
	std::vector<vertex> vertices;
	std::vector<std::vector<int> > faces;
	mesh m;
public:
	off_file_loader(const std::string& off_file_name) {
		std::ifstream in(off_file_name.c_str());
		if(!in) throw std::runtime_error("Could not access mesh file '"+off_file_name+"'.");
		int line_number = 0;

		std::string header = nextline(in,line_number,off_file_name);
		if(header!="OFF") throw std::runtime_error("Header line 'OFF' should appear by itself on the first line, but not found.");

		int n_vertices, n_faces, n_edges;
		std::istringstream vfe(nextline(in,line_number,off_file_name));
		vfe >> n_vertices >> n_faces >> n_edges;

		std::vector<vertex> vertices;
		vertices.reserve(n_vertices+4); //extra 4 for table added below
		m.radius = -std::numeric_limits<double>::infinity();
		m.origin_height_above_table = -std::numeric_limits<double>::infinity();

		for(int j=0;j<n_vertices;j++) {
//			std::istringstream v(nextline(in,line_number,off_file_name));
//			double x, y, z;
//			v >> x >> y >> z;
			vertex p(nextline(in,line_number,off_file_name));
			p.x /= 100.; //cm to m
			p.y /= 100.; //cm to m
			p.z /= 100.; //cm to m
			m.radius = std::max(m.radius,sqrt(p.x*p.x+p.y*p.y+p.z*p.z));
			m.origin_height_above_table = std::max(m.origin_height_above_table,-(GLdouble)p.z);
			vertices.push_back(p);
		}

		std::vector<std::vector<int> > faces;
		faces.reserve(n_faces+1); //extra 1 for table added below
		for(int j=0;j<n_faces;j++) {
			std::istringstream line(nextline(in,line_number,off_file_name));
			int n_points;
			line >> n_points;

			faces.push_back(std::vector<int>());
			std::vector<int>& indices = faces.back();
			for(int k=0;k<n_points;k++) {
				int index;
				line >> index;
				if(index<0 || index>(int)vertices.size()-1) {
					std::ostringstream err;
					err << "Unknown vertex index '" << index << "' on line " << line_number << " of " << off_file_name << ".";
					throw std::runtime_error(err.str().c_str());
				}
				indices.push_back(index);
			}
		}

		m.display_list_number_no_textures = m.display_list_number_with_textures = glGenLists(1);
		glNewList(m.display_list_number_no_textures, GL_COMPILE);
		GLfloat color[4] = {1.0,1.0,1.0,1.0};
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
		for(int j=0;j<(int)faces.size();j++) {
			const std::vector<int>& indices = faces[j];

			const vertex& t = vertices[indices[0]];
			const vertex& u = vertices[indices[1]];
			const vertex& v = vertices[indices[2]];

//			vertex v0;
//			v0.x = u.x-t.x;
//			v0.y = u.y-t.y;
//			v0.z = u.z-t.z;
//
//			vertex v1;
//			v1.x = v.x-u.x;
//			v1.y = v.y-u.y;
//			v1.z = v.z-u.z;

			vertex normal = (u-t).cross(v-u);
			//cross(u-t,v-u)
//			normal.x = (v0.y*v1.z-v0.z*v1.y);
//			normal.y = (v0.z*v1.x-v0.x*v1.z);
//			normal.z = (v0.x*v1.y-v0.y*v1.x);

			glBegin(GL_POLYGON);
			glNormal3f(normal.x,normal.y,normal.z);
			for(int k=0;k<(int)indices.size();k++) {
				const vertex& v = vertices[indices[k]];
				glVertex3f(v.x,v.y,v.z);
			}
			glEnd(); //GL_POLYGON
		}
		glEndList();
	}
	mesh get_mesh() {return m;}
};

mesh load_off(const std::string& off_file_name) { return off_file_loader(off_file_name).get_mesh(); }


inline void project(double X, double Y, double Z, int width, int height,
		//output parameters:
		double& x, double& y) {
	double projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projection);
	double input[4];
	input[0] = X;
	input[1] = Y;
	input[2] = Z;
	input[3] = 1;
	double result[4];
	for(int r=0;r<4;++r) {
		result[r] = 0;
		for(int c=0;c<4;++c) {
			result[r] += input[c]*projection[r+c*4];
		}
	}

	float rhw = 1./result[3];

	x = (1.-result[0]*rhw)*width/2.;
	y = (1.-result[1]*rhw)*height/2.;
}

inline void project_sphere(double x, double y, double z, double r, int width, int height,
		//output parameters:
		double& min_x, double& max_x, double& min_y, double& max_y, double& x_center, double& y_center) {
	double x_pixel,y_pixel;
	project(x+r,y,z,width,height,min_x,y_pixel);
	project(x-r,y,z,width,height,max_x,y_pixel);
	project(x,y+r,z,width,height,x_pixel,min_y);
	project(x,y-r,z,width,height,x_pixel,max_y);
	project(x,y,z,  width,height,x_center,y_center);
}


bool sphere_in_FOV(int min_x, int max_x, int min_y, int max_y, int width, int height) {
	return
			0<=min_x && max_x<width &&
			0<=min_y && max_y<height;
}


void simulated_kinect_snapshot(int min_x, int max_x, int min_y, int max_y,
		int width, int height, double near, double far, double left, double bottom,
		//output parameters
		cv::Mat_<cv::Vec3b>& cropped_im, cv::Mat_<float>& cropped_depth) {
	int cropped_width  = max_x - min_x + 1;
	int cropped_height = max_y - min_y + 1;

	//thanks to: http://stackoverflow.com/questions/9097756/converting-data-from-glreadpixels-to-opencvmat
	cropped_im.create(cropped_height,cropped_width);
	//use fast 4-byte alignment (default anyway) if possible
	glPixelStorei(GL_PACK_ALIGNMENT, (cropped_im.step & 3) ? 1 : 4);
	//set length of one complete row in destination data (doesn't need to equal cropped_im.cols)
	glPixelStorei(GL_PACK_ROW_LENGTH, cropped_im.step/cropped_im.elemSize());
	glReadPixels(min_x,min_y,cropped_width,cropped_height,GL_BGR,GL_UNSIGNED_BYTE,cropped_im.data);
	cv::flip(cropped_im,cropped_im,0);

	cropped_depth.create(cropped_height,cropped_width);
	glReadPixels(min_x,min_y,cropped_width,cropped_height,GL_DEPTH_COMPONENT,GL_FLOAT,cropped_depth.data);
	cv::flip(cropped_depth,cropped_depth,0);
	double f = far, n = near;
	for(int y=0;y<cropped_height;y++) {
		for(int x=0;x<cropped_width;x++) {
			if(cropped_depth(y,x)==1) {
				cropped_depth(y,x) = std::numeric_limits<float>::quiet_NaN();
			} else {
				double zn = cropped_depth(y,x);
				double ze = 2.*f*n/(f+n-(f-n)*(2*zn-1));// + z_correction;
				double xe = -ze*(x+min_x - width/2.)*2./width*left/n;
				double ye = -ze*(y+min_y - height/2.)*2./height*bottom/n;
				cropped_depth(y,x) = sqrt(xe*xe+ye*ye+ze*ze);

			}
		}
	}

	//surfaces that have a high angle with respect to the image plane are often missed by the kinect
	//only use dx because we are working with a stereo camera that has a horizontal disparity (in x, not y)
//	cv::Mat_<float> ddx(cropped_depth.rows,cropped_depth.cols);
//	int depth_aperture_size = 7;
//	cv::Sobel(cropped_depth,ddx,CV_32F,1,0,depth_aperture_size);
//	for(int y=0;y<cropped_depth.rows;y++) {
//		for(int x=0;x<cropped_depth.cols;x++) {
//			float dx = ddx(y,x);
//			if(fabs(dx)>15) {
//				cropped_depth(y,x) = std::numeric_limits<float>::quiet_NaN();
//			}
//		}
//	}
}

bool mesh::render(
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
		cv::Mat_<cv::Vec3b>& im, cv::Mat_<float>& depth, double& x_center, double& y_center) const {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const double fx = 525, fy = 525, cx = 319.5, cy = 239.5;
	const double near = 0.5, far = 5.0;
	const double left = -cx*near/fx*.892121;
	const double bottom = -cy*near/fy*.892121;
	const int width = 640, height = 480;
	glFrustum(-left,left,-bottom,bottom,near,far);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRotatef(screen_updown,1,0,0);
	glRotatef(screen_leftright,0,1,0);
	glTranslatef(0.0, 0.0, -distance);

	glRotatef(rx,0,0,1);
	glRotatef(ry,1,0,0);
	glRotatef(rz,0,1,0);

	glRotatef(90,1,0,0);

	double mv[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,mv);

	double xc = mv[0+3*4];
	double yc = mv[1+3*4];
	double zc = -mv[2+3*4];

	if(zc-radius<near || zc+radius>far) {
		return false;
	}

	double max_x, min_x, max_y, min_y;
	project_sphere(xc,yc,zc,radius*1.2,width,height,
			//output parameters:
			min_x,max_x,min_y,max_y,x_center,y_center);
//	x_center = x_center + 320;
//	y_center = y_center + 240;

	if(!sphere_in_FOV(min_x, max_x, min_y, max_y, width, height)) {
		glPopMatrix();
		return false;
	}

	GLfloat pos[4] = { lx, ly, lz, 0.0 };
	GLfloat light_ambient[] = {ambient, ambient, ambient, 1.0};
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_POSITION, pos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glEnable(GL_LIGHT0);

	if(use_textures) {
		glCallList(display_list_number_with_textures);
	} else {
		glCallList(display_list_number_no_textures);
	}

	simulated_kinect_snapshot(min_x, max_x, min_y, max_y, width, height, near, far, left, bottom,
			//output parameters
			im, depth);

	w.swap_buffers();

	glPopMatrix();

	for(int y=0;y<im.rows;y++) {
		for(int x=0;x<im.cols;x++) {
			if(im(y,x)!=cv::Vec3b(0,0,0)) {
				return true;
			}
		}
	}
	return false;
}

} //namespace OBJREC

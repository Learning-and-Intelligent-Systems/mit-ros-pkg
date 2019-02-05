/*
 * visualize_object.cpp
 *
 *  Created on: Sep 21, 2011
 */

//#include "image.h"
//#include "detection.h"
#include "constraints.h"
#include <map>

int main(int argc, char** argv) {

	if(argc<2) {
		std::cerr << "usage:" << std::endl;
		std::cerr << " " << std::string(argv[0]) << " object_file" << std::endl;
		std::cerr << std::endl;
		std::cerr << " OR:" << std::endl;
		std::cerr << std::endl;
		std::cerr << " " << std::string(argv[0]) << " object_file image_file1 image_file2 ..." << std::endl;
		std::cerr << std::endl;
		std::cerr << " Visualizes the object model in object_file. Use the number pad to change viewpoints." << std::endl;
		std::cerr << " Optionally also finds the best detection for the displayed viewpoint in the image file(s)." << std::endl;
		return -1;
	}

	std::ifstream in(argv[1]);
	if(!in) throw std::runtime_error(std::string("Could not open object file '")+argv[1]+"'.");
	objrec::object obj;
	in >> obj;
	char* image_file = 0;
	std::string depth_file;
	cv::Mat_<cv::Vec3b> detection_picture, edges_bgr, depth_bgr, textons_bgr;
	cv::Mat_<float> detection_depth;
//	objrec::bounding_box bbox;
//	bool has_bounding_box;
	objrec::image* detection_image = 0;
	objrec::scene_info info;
	info.camera_tilt_angle = std::numeric_limits<float>::quiet_NaN();
	if(argc>=3) {
		image_file = argv[2];
		detection_picture = objrec::read_image(image_file);
		depth_file = objrec::corresponding_depth_file(std::string(image_file));
		detection_depth = objrec::read_depth(depth_file);
//		std::string bounding_box_file = objrec::corresponding_bounding_box_file(image_file);
//		std::ifstream in(bounding_box_file.c_str());
//		if(in) {
//			has_bounding_box = true;
//			objrec::read_bounding_box(in,bbox);
//		}
//		edges_bgr = objrec::edges_bgr(detection_picture, detection_depth, obj.visual_words.get_edges());
		depth_bgr = objrec::mat_bgr((cv::Mat_<float>)-detection_depth);
		std::ifstream scene_file(objrec::corresponding_scene_file(image_file).c_str());
		if(scene_file) {
			objrec::read_scene_info(scene_file,info);
		} else {
			std::cout << "could not find scene file " << scene_file << std::endl;
			return -1;
		}
//		textons_bgr = objrec::textons_bgr(detection_picture,obj.visual_words.get_textons());
//		cv::imshow("edges",edges_bgr);
		cv::imshow("depth",depth_bgr);
//		cv::imshow("textons",textons_bgr);
		cv::imshow("picture",detection_picture);
		cv::waitKey(100);
		detection_image = new objrec::image(obj.library,detection_picture,detection_depth);
	}
	objrec::table_constraint tc(info,obj.object_center_height_above_table);

	objrec::viewpoint vp0 = obj.views[0].get_viewpoint();
	float depth_step = vp0.max_depth-vp0.min_depth;
	float rx_step = (vp0.max_rx-vp0.min_rx)/10.f;
	float ry_step = (vp0.max_ry-vp0.min_ry)/10.f;
	float rz_step = (vp0.max_rz-vp0.min_rz)/10.f;
	float x_step = 5;
	float y_step = 5;

	float rx_mid = (vp0.max_rx+vp0.min_rx)/2.;
	float ry_mid = (vp0.max_ry+vp0.min_ry)/2.;
	float rz_mid = (vp0.max_rz+vp0.min_rz)/2.;
	objrec::point p(320,240,1,rx_mid,ry_mid,rz_mid);
	if(info.camera_tilt_angle==info.camera_tilt_angle) { //if camera_tilt_angle is not a NaN, i.e. if a .scene file was found
		p.rx = tc.rx(p.x,p.y);
		p.ry = tc.ry(p.x,p.y);
		p.depth = tc.depth(p.x,p.y);
	}

	int image_count = 0;

//	std::map<objrec::view*,objrec::localization> localization_cache;

//	objrec::image *img = 0;
//	if(image_file) {
//		img = new objrec::image(obj.visual_words,detection_picture,detection_depth);
//	}

	bool viewing = true;
	bool detect_now = false;
	cv::Mat_<cv::Vec3b> im(cv::Size(640,480),cv::Vec3b(255,255,255));
	while(viewing) {
		objrec::view* v = find_view(obj,p.rx,p.ry,p.rz);
		if(detection_image!=0) {
			std::cout << detection_image->max_log_probability(*v,p)
					<< " visual: " << detection_image->max_log_visual_parts_probability(*v,p)
					<< " depth: " << detection_image->max_log_depth_parts_probability(*v,p);
		}
		objrec::viewpoint vp = objrec::find_view(obj,p.rx,p.ry,p.rz)->get_viewpoint();
		double rx = (vp.min_rx + vp.max_rx)/2.0;
		double ry = (vp.min_ry + vp.max_ry)/2.0;
		double rz = (vp.min_rz + vp.max_rz)/2.0;
		std::cout << "current view: " << p << " viewpoint: (rx=" << rx << ", ry=" << ry << ", rz=" << rz << ")" << std::endl;
//		if(image_file && detect_now) {
//			detect_now = false;
//			float lower_bound = -std::numeric_limits<float>::infinity();
//			objrec::localization l;
//			std::map<objrec::view*,objrec::localization>::iterator map_entry = localization_cache.find(v);
//			if(map_entry==localization_cache.end()) {
//				if(has_bounding_box) {
//					l = objrec::best_localization<objrec::bounding_box_constraint>(*v,*img,lower_bound,
//							objrec::bounding_box_constraint(bbox));
//				} else {
//					l = objrec::best_localization<objrec::no_constraint>(*v,*img,lower_bound);
//				}
//				localization_cache.insert(std::pair<objrec::view*,objrec::localization>(v,l));
//			} else {
//				l = map_entry->second;
//			}
//			if(l.v!=0) {
//				cv::Mat_<cv::Vec3b> modified_picture = detection_picture.clone();
//				cv::Mat_<cv::Vec3b> modified_edges_bgr = edges_bgr.clone();
//				cv::Mat_<cv::Vec3b> modified_textons_bgr = textons_bgr.clone();
//				cv::Mat_<cv::Vec3b> modified_depth_bgr = depth_bgr.clone();
//				objrec::draw(l,*img,
//						modified_picture,modified_edges_bgr,modified_textons_bgr,modified_depth_bgr);
//				cv::imshow("edges",modified_edges_bgr);
//				cv::imshow("depth",modified_depth_bgr);
//				cv::imshow("textons",modified_textons_bgr);
//				cv::imshow("picture",modified_picture);
//				cv::waitKey(100);
//				std::cerr << l << std::endl;
//			} else {
//				std::cerr << "no detection overlaps bounding box sufficiently." << std::endl;
//			}
//		}
		if(info.camera_tilt_angle==info.camera_tilt_angle) {  //if camera_tilt_angle is not a NaN, i.e. if a .scene file was found
			im = detection_picture.clone();
		} else {
			objrec::set_to(im,cv::Vec3b(255,255,255)); //else white background
		}
		if(detection_image!=0) {
			detection_image->draw(*v,p,im);
		} else {
			v->draw(p,
					//output argument:
					im);
		}
//		cv::Mat im = objrec::view_image(*v,rx,ry,rz,depth);

		cv::imshow(obj.name, im);
		int key = cv::waitKey(0);
		switch(key) {
		case 1114041: // numpad 9 with numlock
		case 65434: // numpad 9
		case 114: // r
			p.rx -= rx_step;
			break;
		case 1114039: // numpad 7 with numlock
		case 65429: // numpad 7
		case 119: // w
			p.rx += rx_step;
			break;
		case 1114040: // numpad 8 with numlock
		case 65431: // numpad 8
		case 101: // e
			p.ry += ry_step;
			break;
		case 1114034: // numpad 2 with numlock
		case 65433: // numpad 2
		case 99: // c
			p.ry -= ry_step;
			break;
		case 1114036: // numpad 4 with numlock
		case 65430: // numpad 4
		case 115: // s
			p.rz += rz_step;
			break;
		case 1114038: // numpad 6 with numlock
		case 65432: // numpad 6
		case 102: // f
			p.rz -= rz_step;
			break;
		case 1113937: // left arrow with numlock
		case 65361: // left arrow
			p.x -= x_step;
			break;
		case 1113939: // right arrow with numlock
		case 65363: // right arrow
			p.x += x_step;
			break;
		case 1113938: // up arrow with numlock
		case 65362: // up arrow
			p.y -= y_step;
			break;
		case 1113940: // down arrow with numlock
		case 65364: // down arrow
			p.y += y_step;
			break;

//		case 1114033: //numpad 1 with numlock
//		case 65436: //numpad 1
//			depth -= depth_step;
//			break;
//		case 1114035: //numpad 3 with numlock
//		case 65435: //numpad 3
//			depth += depth_step;
//			break;
		case 1114027: // numpad + with numlock
		case 65451: // numpad +
		case 61: // =
			p.depth /= 1.01f;
			break;
		case 1114029: // numpad - with numlock
		case 65453: // numpad -
		case 45: // -
			p.depth *= 1.01f;
			break;
		case 32: //space
		case 1048608: //space with numlock
		{
			std::ostringstream im_name;
			im_name << obj.name << image_count << ".png";
			image_count++;
			cv::imwrite(im_name.str(),im);
			std::cerr << "Wrote image file " << im_name.str() << std::endl;
		}
			break;
		case 10: //enter
		case 65421: //numpad enter
		case 1048586: //enter with numlock
			std::cerr << "Viewpoint: " << v->get_viewpoint() << std::endl;
			if(image_file) {
				std::cerr << "About to detect viewpoint..." << std::endl;
			}
			detect_now = true;
			break;
		case 113: //q
		case 1048689: //q with numlock
		case 27: //esc
		case 1048603: //esc with numlock
			viewing = false;
			break;
		default:
			std::cerr << "unrecognized key: " << key << std::endl;
			break;
		}
		p.rx = fmod(p.rx,360);
		p.ry = fmod(p.ry,360);
		p.rz = fmod(p.rz,360);

		if(info.camera_tilt_angle==info.camera_tilt_angle) { //if camera_tilt_angle is not a NaN, i.e. if a .cam file was found
			p.rx = tc.rx(p.x,p.y);
			p.ry = tc.ry(p.x,p.y);
			p.depth = tc.depth(p.x,p.y);
		}
	}

	//todo: uncomment this when visualizing the new representation works
//	if(img!=0) {
//		delete img;
//		std::cout << "#" << objrec::viewpoint_description << std::endl;
//		std::cout << vp << std::endl;
//	}
	if(detection_image!=0) {
		delete detection_image;
	}
}

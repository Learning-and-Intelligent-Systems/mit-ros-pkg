/*
 * label_pose.cpp
 *
 *  Created on: May 19, 2014
 */

#include "constraints.h"
#include "simulation.h"
#include "draw.h"

objrec::view* best_view(const objrec::image& im, objrec::object& obj, const objrec::point& p) {
	float best_log_probability = -std::numeric_limits<float>::infinity();
	int best_view = -1;
	for(int j=0;j<obj.views.size();j++) {
		float log_probability = im.max_log_probability(obj.views[j],p);
		if(log_probability>best_log_probability) {
			best_view = j;
			best_log_probability = log_probability;
		}
	}
	return &obj.views[best_view];
}

void clip_point(const objrec::object& obj, objrec::point& p) {
	float inf = std::numeric_limits<float>::infinity();
	objrec::point min_p(inf,inf,inf,inf,inf,inf);
	objrec::point max_p(-inf,-inf,-inf,-inf,-inf,-inf);
	for(int j=0;j<obj.views.size();j++) {
		const objrec::viewpoint& v = obj.views[j].get_viewpoint();
		if(v.min_depth<min_p.depth) min_p.depth = v.min_depth;
		if(v.max_depth>max_p.depth) max_p.depth = v.max_depth;
		if(v.min_rx<min_p.rx) min_p.rx = v.min_rx;
		if(v.max_rx>max_p.rx) max_p.rx = v.max_rx;
		if(v.min_ry<min_p.ry) min_p.ry = v.min_ry;
		if(v.max_ry>max_p.ry) max_p.ry = v.max_ry;
		if(v.min_rz<min_p.rz) min_p.rz = v.min_rz;
		if(v.max_rz>max_p.rz) max_p.rz = v.max_rz;
	}
	if(p.depth<min_p.depth) p.depth = min_p.depth;
	if(p.depth>max_p.depth) p.depth = max_p.depth;
	if(p.rx<min_p.rx) p.rx = min_p.rx;
	if(p.rx>max_p.rx) p.rx = max_p.rx;
	if(p.ry<min_p.ry) p.ry = min_p.ry;
	if(p.ry>max_p.ry) p.ry = max_p.ry;
	if(p.rz<min_p.rz) {
		p.rz = max_p.rz;
	}
	if(p.rz>max_p.rz) {
		p.rz = min_p.rz;
	}
}

int main(int argc, char** argv) {
	if(argc<3) {
		std::cerr << "usage:" << std::endl;
		std::cerr << " " << std::string(argv[0]) << " mesh_file object_file image_file1 image_file2 ..." << std::endl;
		std::cerr << std::endl;
		std::cerr << " Labels the pose of an object on a table top with turn-table rotation (rotation about the z axis)." << std::endl;
		return -1;
	}

	std::string mesh_file = argv[1];
	std::string object_file = argv[2];
	float x_step = 1;
	float y_step = 1;
	float depth_step = .002;
	float rx_step = 1;
	float ry_step = 1;
	float rz_step = 1;

	std::ifstream in(object_file.c_str());
	if(!in) throw std::runtime_error("Could not open object file '"+object_file+"'.");
	objrec::object obj;
	in >> obj;


	std::string depth_file;
	cv::Mat_<cv::Vec3b> detection_picture, edges_bgr, depth_bgr, textons_bgr;
	cv::Mat_<float> detection_depth;
//	objrec::bounding_box bbox;
//	bool has_bounding_box;
	objrec::image* detection_image = 0;
	objrec::scene_info info;
	info.camera_tilt_angle = std::numeric_limits<float>::quiet_NaN();

	objrec::OpenGL_X_window w;
	objrec::mesh m = objrec::load_off(mesh_file);

	int display_mode = 2;

	for(int image_file_index = 3;image_file_index<argc;image_file_index++) {
		std::string image_file(argv[image_file_index]);
		detection_picture = objrec::read_image(image_file);
		depth_file = objrec::corresponding_depth_file(std::string(image_file));
		detection_depth = objrec::read_depth(depth_file);
		std::string precomputed_edges_file = objrec::corresponding_edges_file(image_file);
		std::vector<cv::Mat_<float> > precomputed_edges;
		if(precomputed_edges_file!="") {
			objrec::read_edges(precomputed_edges_file,
					//output parameter:
					precomputed_edges);
			edges_bgr = objrec::edges_bgr(precomputed_edges, obj.library.get_edges());
		} else {
			edges_bgr = objrec::edges_bgr(detection_picture, detection_depth, obj.library.get_edges());
		}
		depth_bgr = objrec::mat_bgr((cv::Mat_<float>)-detection_depth);
		std::ifstream scene_file(objrec::corresponding_scene_file(image_file).c_str());
		if(scene_file) {
			objrec::read_scene_info(scene_file,info);
		} else {
			std::cerr << "could not find scene file " << objrec::corresponding_scene_file(image_file) << std::endl;
			return -1;
		}
		//	textons_bgr = objrec::textons_bgr(detection_picture,obj.visual_words.get_textons());
		cv::imshow("edges",edges_bgr);
		cv::imshow("depth",depth_bgr);
		cv::waitKey(100);
		detection_image = new objrec::image(obj.library,detection_picture,detection_depth,precomputed_edges,1.0f);
		objrec::table_constraint tc(info,obj.object_center_height_above_table);
		objrec::viewpoint vp = obj.views[0].get_viewpoint();
		int x = detection_picture.cols/2;
		int y = detection_picture.rows/2;
		float ddepth = 0, drx = 0, dry = 0;
		objrec::point p(x,y,tc.depth(x,y),tc.rx(x,y),tc.ry(x,y),(vp.min_rz+vp.max_rz)/2.0);
		size_t dot = image_file.find_last_of('.');
		std::string pose_file = image_file.substr(0,dot)+".pose";
		{
			std::ifstream in(pose_file.c_str());
			if(in) {
				in >> p;
				ddepth = p.depth - tc.depth(p.x,p.y);
				drx = p.rx - tc.rx(p.x,p.y);
				dry = p.ry - tc.ry(p.x,p.y);
				std::cerr << "Read existing pose file from " << pose_file << "." << std::endl;
			}
		}
//		cv::imwrite("/tmp/label_pose_output/edges.png",edges_bgr);
//		cv::imwrite("/tmp/label_pose_output/depth.png",depth_bgr);
//		cv::imwrite("/tmp/label_pose_output/rgb.png",detection_picture);
//		objrec::view* v = best_view(*detection_image,obj,p);
//		cv::imwrite("/tmp/label_pose_output/model.png",draw_model(detection_picture,detection_image,p,v));
//		cv::imwrite("/tmp/label_pose_output/combined_mesh.png",draw_combined_mesh(detection_picture,detection_depth,m,p,w));
//		cv::imwrite("/tmp/label_pose_output/superimposed_mesh.png",draw_superimposed_mesh(detection_picture,m,p,w));
//		return 0;

		//	cv::imshow("textons",textons_bgr);
		bool done = false;
		while(!done) {
			objrec::view* v = best_view(*detection_image,obj,p);

			std::cerr << detection_image->max_log_probability(*v,p)
									<< " visual: " << detection_image->max_log_visual_parts_probability(*v,p)
									<< " depth: " << detection_image->max_log_depth_parts_probability(*v,p) << " ";
			std::cerr << "current view: " << p << " ddepth: " << ddepth << " drx: " << drx << " dry: " << dry << std::endl;
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
			cv::Mat_<cv::Vec3b> im;
			switch(display_mode) {
			case 0:
				im = objrec::draw_model(detection_picture,detection_image,p,v);
				break;
			case 1:
				im = objrec::draw_combined_mesh(detection_picture,detection_depth,m,p,w);
				break;
			case 2:
				im = objrec::draw_superimposed_mesh(detection_picture,m,p,w);
				break;
			default:
				im = detection_picture.clone();
				break;
			}

			cv::Mat_<cv::Vec3b> big_im;
			cv::resize(im,big_im,cv::Size(640*2,480*2),0,0,cv::INTER_NEAREST);
			cv::imshow(obj.name, big_im);
			int key = cv::waitKey(0);
			switch(key) {
			case 1114041: // numpad 9 with numlock
			case 65434: // numpad 9
			case 114: // r
				drx -= rx_step;
				break;
			case 1114039: // numpad 7 with numlock
			case 65429: // numpad 7
			case 119: // w
				drx += rx_step;
				break;
			case 1114040: // numpad 8 with numlock
			case 65431: // numpad 8
			case 101: // e
				dry += ry_step;
				break;
			case 1114034: // numpad 2 with numlock
			case 65433: // numpad 2
			case 99: // c
				dry -= ry_step;
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
				ddepth -= depth_step;
				break;
			case 1114029: // numpad - with numlock
			case 65453: // numpad -
			case 45: // -
				ddepth += depth_step;
				break;
			case 32: //space
			case 1048608: //space with numlock
				display_mode = (display_mode+1)%4;
				break;
			case 10: //enter
			case 65421: //numpad enter
			case 1048586: //enter with numlock
			{
				std::ofstream out(pose_file.c_str());
				out << p << std::endl;
				std::cerr << "Wrote changes to " << pose_file << "." << std::endl;
				done = true;
			}
				break;
//			case 113: //q
//			case 1048689: //q with numlock
			case 65288: //backspace
			case 27: //esc
			case 1048603: //esc with numlock
				std::cerr << "Did NOT write changes to " << pose_file << "." << std::endl;
				done = true;
				break;
			case 9: //tab
			{
				objrec::point best_p = p;
				objrec::view* best_v = v;
				float best_max_log_probability;
				float improvement = 1;
//				while(improvement>0) {
					best_max_log_probability = detection_image->max_log_probability(*v,best_p);
					improvement = -std::numeric_limits<float>::infinity();
#define TRY_P(x,y,depth,rx,ry,rz) { \
	objrec::point try_p(x,y,depth,rx,ry,rz); \
	clip_point(obj,try_p); \
	objrec::view* try_view = best_view(*detection_image,obj,try_p);\
	float try_max_log_probability = detection_image->max_log_probability(*try_view,try_p); \
	float try_improvement = try_max_log_probability-best_max_log_probability; \
	if(try_improvement>0 && try_improvement>improvement) { \
		improvement = try_improvement; \
		best_p = try_p; \
		best_v = try_view; \
	}\
}
					TRY_P(best_p.x+x_step,best_p.y,best_p.depth,best_p.rx,best_p.ry,best_p.rz);
					TRY_P(best_p.x-x_step,best_p.y,best_p.depth,best_p.rx,best_p.ry,best_p.rz);
					TRY_P(best_p.x,best_p.y+y_step,best_p.depth,best_p.rx,best_p.ry,best_p.rz);
					TRY_P(best_p.x,best_p.y-y_step,best_p.depth,best_p.rx,best_p.ry,best_p.rz);
					TRY_P(best_p.x,best_p.y,best_p.depth+depth_step,best_p.rx,best_p.ry,best_p.rz);
					TRY_P(best_p.x,best_p.y,best_p.depth-depth_step,best_p.rx,best_p.ry,best_p.rz);
					TRY_P(best_p.x,best_p.y,best_p.depth,best_p.rx+rx_step,best_p.ry,best_p.rz);
					TRY_P(best_p.x,best_p.y,best_p.depth,best_p.rx-rx_step,best_p.ry,best_p.rz);
					TRY_P(best_p.x,best_p.y,best_p.depth,best_p.rx,best_p.ry+ry_step,best_p.rz);
					TRY_P(best_p.x,best_p.y,best_p.depth,best_p.rx,best_p.ry-ry_step,best_p.rz);
					TRY_P(best_p.x,best_p.y,best_p.depth,best_p.rx,best_p.ry,best_p.rz+rz_step);
					TRY_P(best_p.x,best_p.y,best_p.depth,best_p.rx,best_p.ry,best_p.rz-rz_step);
//				}
				if(detection_image->max_log_probability(*best_v,best_p)>detection_image->max_log_probability(*v,p)) {
					v = best_v;
					p.x = best_p.x;
					p.y = best_p.y;
					ddepth = best_p.depth - tc.depth(best_p.x,best_p.y);
					drx = best_p.rx - tc.rx(best_p.x,best_p.y);
					dry = best_p.ry - tc.ry(best_p.x,best_p.y);
					p.rz = best_p.rz;
				}
			}
				break;
			default:
				std::cerr << "unrecognized key: " << key << std::endl;
				break;
			}

			p.rx = tc.rx(p.x,p.y)+drx;
			p.ry = tc.ry(p.x,p.y)+dry;
			p.depth = tc.depth(p.x,p.y)+ddepth;
			//		p.rx = p.rx - 360.0*floor(p.rx/360.0);
			//		p.ry = p.ry - 360.0*floor(p.ry/360.0);
			//		p.rz = p.rz - 360.0*floor(p.rz/360.0);
			clip_point(obj,p);
//			p.rx = fmod(p.rx,360.0);
//			p.ry = fmod(p.ry,360.0);
//			p.rz = fmod(p.rz,360.0);
		}

		delete detection_image;
	}
}

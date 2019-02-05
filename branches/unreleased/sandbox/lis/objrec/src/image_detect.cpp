/*
 * image_detect.cpp
 *
 *  Created on: Feb 27, 2012
 */

#include "constraints.h"
#include "gui.h"
#include "args.h"

int main(int argc, char** argv) {
	INIT_PARAMS();
	help += "usage:";
	help += "\n";
	help += "  " + std::string(argv[0]) + " -object_file <object_file> -image_file <image_file>\n";
	help += "\n";
	help += " Searches in an image (with depth) for high-probability detections of an object model.\n";
	help += "\n";
	help += "Required Parameters\n";
	help += "===================\n";
	ADD_STRING_PARAM(object_file,"","The object file to load.");
	ADD_STRING_PARAM(image_file,"", "An image file to load.");
	help += "\n";
	help += "Optional Parameters\n";
	help += "===================\n";
	ADD_STRING_PARAM(output_file,"","The file in which to store the detections.  Detections are appended to the end of this file, which is useful if you are running multiple instances of this program in parallel (see below).  Delete this file before running this program if you want to make sure you are only looking at the most recent results. If this file is not passed, the results are printed to stdout.");
//	ADD_INT_PARAM(read_output_file,0,"If non-zero and if the output_file exists, the best detection in the output_file is used as a lower_bound.  This option is useful to improve speed if you are running several instances of this program in parallel with different ranges of views (using '-start_view_index' and '-stop_view_index') and if you are only interested in the top detection ('-only_top_detection 1').");
	ADD_STRING_PARAM(depth_file,"", "The corresponding depth file to load.  If depth_file is not specified, it is assumed to have the same base name as the image, with the extension .yml.");
	ADD_STRING_PARAM(bounding_box_file,"","The corresponding bounding box file to load.  The bounding box file defines a rectangular region of pixels in the image.  Only detections whose bounding boxes have sufficient overlap (as defined by the bounding_box_overlap parameter) with this bounding box are considered. If bounding_box_file is not specified, it is assumed to have the same base name as the image, with the extension .bbox. If it does not exist, no bounding box constraint is used.  If bounding box and a camera info files are both available, only the bounding box constraint will be used.")
	ADD_PARAM(bounding_box_overlap,0.5,"The minimum overlap between the ground truth bounding box and the detected bounding box required to consider the detection.  This is calculated using by dividing the area of the intersection by the area of the union of the two bounding boxes.");
	ADD_INT_PARAM(ignore_bounding_box,0,"If non-zero, the bounding box file (if any) is ignored when considering detections.");
	ADD_STRING_PARAM(camera_info_file,"","The corresponding camera info file to load.  If camera_info_file is not specified, it is assumed to have the same base name as the image, with the extension .cam. If it does not exist, no level constraints are used. If bounding box and a camera info files are both available, only the bounding box constraint will be used.");
	ADD_INT_PARAM(ignore_camera_info,0,"If non-zero, the camera info file (if any) is ignored when considering detections.");
	float inf = std::numeric_limits<float>::infinity();
	ADD_PARAM(lower_bound,-inf,"The the lowest log probability value to search before terminating.");
	ADD_INT_PARAM(only_top_detection,0,"If non-zero, the search will terminate immediately after the first (highest probability) detection in the image.");
//	ADD_INT_PARAM(non_maximum_suppression,1,"If non-zero, detections that overlap more than nms_overlap_amount will be greedily ignored.");
	ADD_PARAM(nms_overlap_percent,0.5,"The amount of overlap required to reject a detection by non maximum suppression.  The overlap is calculuated as with the bounding_box_overlap parameter.");
	ADD_INT_PARAM(verbose,0,"If non-zero, status information will be printed to stderr.");
	ADD_INT_PARAM(view_detections,0,"If non-zero, the detections will be displayed in an image window.");
	ADD_INT_PARAM(visualize_search,0,"If non-zero and a camera info file is provided, the progress through search regions of the image will be displayed.");
	ADD_INT_PARAM(start_view_index,0,"The object file consists of a list of views.  This parameter specifies the first index in a range of views within that list (the first index being 0).  Only views within this range are used for detection.  This is useful for parallelizing long detection jobs across multiple CPUs.");
	ADD_INT_PARAM(stop_view_index,-1,"The object file consists of a list of views.  This parameter specifies the last index (non-inclusive) in a range of views within that list.  Only views within this range are used for detection.  This is useful for parallelizing long detection jobs across multiple CPUs. A value of -1 means that the range will go to the end of the list of views.  If the initial list is I and the final list is F, then views I, I+1, ..., F-1 will be searched for.");

	CHECK_PARAMS();

	if(!object_file_is_passed || !image_file_is_passed) {
		std::cerr << "A required parameter was not passed." << std::endl;
		std::cerr << std::endl;
		std::cerr << help << std::endl;
		return 0;
	}

	if(lower_bound==-inf && only_top_detection==0) {
		std::cerr << "WARNING! Searching for all detections with a lower bound of -inf will yield a very large output";
		if(output_file_is_passed) std::cerr << " file (" << output_file << ")";
		std::cerr << "." << std::endl;
		std::cerr << "  use '-only_top_detection 1' or set the '-lower_bound' parameter." << std::endl;
	}

	if(!depth_file_is_passed) depth_file = objrec::corresponding_depth_file(image_file);
	if(!camera_info_file_is_passed) camera_info_file = objrec::corresponding_camera_info_file(image_file);
	if(!bounding_box_file_is_passed) bounding_box_file = objrec::corresponding_bounding_box_file(image_file);

	objrec::object obj;
	std::ifstream in(object_file.c_str());
	if(!in) throw std::runtime_error("Could not open object file '"+object_file+"'.");
    long long start, end;

    if(verbose!=0) std::cerr << "Loading object file '" << object_file << "'... " << std::flush;
    start = objrec::current_time();
	in >> obj;
	end = objrec::current_time();
	if(verbose!=0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;


	if(verbose!=0) std::cerr << "Loading picture file '" << image_file << "'... " << std::flush;
	start = objrec::current_time();
	cv::Mat_<cv::Vec3b> picture = cv::imread(image_file);
	end = objrec::current_time();
	if(verbose!=0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	if(verbose!=0) std::cerr << "Loading depth file '" << depth_file << "'... " << std::flush;
	start = objrec::current_time();
	cv::Mat_<float> depth = objrec::read_depth(depth_file);
	end = objrec::current_time();
	if(verbose!=0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	objrec::camera_info info;
	if(ignore_camera_info==0 && camera_info_file!="") {
		if(verbose!=0) std::cerr << "Loading camera info file '" << camera_info_file << "'... " << std::flush;
		start = objrec::current_time();
		std::ifstream in(camera_info_file.c_str());
		objrec::read_camera_info(in,info);
		end = objrec::current_time();
		if(verbose!=0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}

	objrec::bounding_box bbox;
	if(ignore_bounding_box==0 && bounding_box_file!="") {
		if(verbose!=0) std::cerr << "Loading bounding box file '" << bounding_box_file << "'... " << std::flush;
		start = objrec::current_time();
		std::ifstream in(bounding_box_file.c_str());
		in.exceptions(std::istringstream::failbit);
		objrec::read_bounding_box(in,bbox);
		end = objrec::current_time();
		if(verbose!=0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}

//	if(read_output_file!=0) {
//		std::ifstream in(output_file.c_str());
//		if(in) {
//			if(verbose!=0) std::cerr << "Loading output_file '" << output_file << "'..." << std::flush;
//			bool new_lower_bound = false;
//			int line_number = 0;
//			while(in.good()) {
//				try {
//					std::istringstream line(objrec::nextline(in, line_number));
//					double log_probability;
//					objrec::point location;
//					objrec::viewpoint vp;
//					std::string detection_object_file, detection_image_file;
//					line >> log_probability >> location >> vp >> detection_object_file >> detection_image_file;
//					if(detection_object_file==object_file && detection_image_file==image_file) {
//						if(log_probability>lower_bound) {
//							new_lower_bound = true;
//							lower_bound = log_probability;
//						}
//					}
//				}
//				catch(...) {
//					break;
//				}
//			}
//			if(verbose!=0) {
//				std::cerr << " done. (";
//				if(new_lower_bound) {
//					std::cerr << "new";
//				} else {
//					std::cerr << "unchanged";
//				}
//				std::cerr << " lower_bound: " << lower_bound << ")" << std::endl;
//			}
//		}
//	}

	cv::Mat_<cv::Vec3b> edges_bgr, depth_bgr, textons_bgr;
	if(view_detections!=0) {
		edges_bgr = objrec::edges_bgr(picture, depth, obj.library.get_edges());
		depth_bgr = objrec::mat_bgr((cv::Mat_<float>)-depth);
		textons_bgr = objrec::textons_bgr(picture,obj.library.get_textons());
		cv::imshow("edges",edges_bgr);
		cv::imshow("depth",depth_bgr);
		cv::imshow("textons",textons_bgr);
		cv::imshow("picture",picture);
		cv::waitKey(100);
	}

	if(verbose!=0) std::cerr << "Preprocessing... " << std::flush;
	start = objrec::current_time();
	objrec::image im(obj.library,picture,depth);
	end = objrec::current_time();
	if(verbose!=0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	if(verbose!=0) std::cerr << "Detecting... " << std::flush;
	start = objrec::current_time();
	std::vector<objrec::localization> localizations;
	if(bounding_box_file!="" && ignore_bounding_box==0) {
		objrec::bounding_box_constraint constr(bbox,bounding_box_overlap);
		objrec::localizations<objrec::bounding_box_constraint>(obj,im,lower_bound,localizations,only_top_detection!=0,
				verbose!=0,visualize_search!=0,start_view_index,stop_view_index,constr,nms_overlap_percent);
	} else if(camera_info_file!="" && ignore_camera_info==0) {
		objrec::level_constraint constr(info);
		objrec::localizations<objrec::level_constraint>(obj,im,lower_bound,localizations,only_top_detection!=0,
				verbose!=0,visualize_search!=0,start_view_index,stop_view_index,constr,nms_overlap_percent);
	} else {
		objrec::localizations<objrec::no_constraint_old>(obj,im,lower_bound,localizations,only_top_detection!=0,
				verbose!=0,visualize_search!=0,start_view_index,stop_view_index);
	}
	end = objrec::current_time();
	if(verbose!=0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	if(output_file_is_passed) {
		std::ofstream out(output_file.c_str(),std::ios_base::app);
		if(!out) throw std::runtime_error("Could not open output file '"+output_file+"'.");
		for(std::vector<objrec::localization>::iterator l=localizations.begin();l!=localizations.end();++l) {
			out << *l << " " << object_file << " " << image_file << std::endl;
		}
	} else {
		for(std::vector<objrec::localization>::iterator l=localizations.begin();l!=localizations.end();++l) {
			std::cout << *l << " " << object_file << " " << image_file << std::endl;
		}
	}

	if(view_detections!=0) {
		cv::imshow("picture",picture);

		if(only_top_detection==0) {
			if(verbose!=0) std::cerr << localizations.size() << " localizations above the log probability: " << lower_bound << "." << std::endl;
			if(localizations.size()==0) {
				cv::waitKey(0);
				return 0;
			}
			std::cerr << "Press up/down to see different localizations." << std::endl;
			int current_loc = 0;
			std::cerr << localizations[current_loc] << std::endl;
			bool quit = false;
			while(!quit) {
				cv::Mat_<cv::Vec3b> modified_picture = picture.clone();
//				cv::Mat_<cv::Vec3b> modified_edges_bgr = edges_bgr.clone();
//				cv::Mat_<cv::Vec3b> modified_textons_bgr = textons_bgr.clone();
//				cv::Mat_<cv::Vec3b> modified_depth_bgr = depth_bgr.clone();
//				objrec::draw(localizations[current_loc],im,
//						modified_picture,modified_edges_bgr,modified_textons_bgr,modified_depth_bgr);
//				cv::imshow("edges",modified_edges_bgr);
//				cv::imshow("depth",modified_depth_bgr);
//				cv::imshow("textons",modified_textons_bgr);
//				cv::imshow("picture",modified_picture);
				localizations[current_loc].v->draw(localizations[current_loc].location,modified_picture);

				int key = cv::waitKey(0);
				switch(key) {
				case 65364: //down arrow
				case 1113940: //down arrow with num lock
					if(current_loc<(int)localizations.size()-1) {
						current_loc++;
						std::cerr << localizations[current_loc] << std::endl;
					}
					break;
				case 65362: //up arrow
				case 1113938: //up arrow with numlock
					if(current_loc>0) {
						current_loc--;
						std::cerr << localizations[current_loc] << std::endl;
					}
					break;
				case 113: //q
				case 1048689: //q with numlock
				case 27: //esc
				case 1048603: //esc with numlock
					quit = true;
					break;
				default:
					std::cerr << "Unrecognized key pressed: " << key << std::endl;
					break;
				}
			}
		} else {

			if(localizations.size()==0) {
				std::cerr << "No localizations above the lower bound were found. Maybe try decreasing the lower bound." << std::endl;
			} else {
				std::cerr << "Top detection:" << std::endl;
				std::cerr << localizations[0] << std::endl;
				localizations[0].v->draw(localizations[0].location,picture);
//				objrec::draw(localizations[0],im,picture,edges_bgr,textons_bgr,depth_bgr);
				cv::imshow("picture",picture);
				cv::waitKey(0);
			}
		}
	}
}

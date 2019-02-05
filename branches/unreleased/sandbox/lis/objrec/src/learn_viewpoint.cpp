/*
 * learn_viewpoint.cpp
 *
 *  Created on: Jun 29, 2012
 */

#include "learning.h"
#include "args.h"

namespace objrec {
void check_finished() {}
}

int main(int argc, char** argv) {
	INIT_PARAMS();
	help += "usage:";
	help += "\n";
	help += "  " + std::string(argv[0]) + " -mesh_file_name <OFF_file>\n";
	help += "\n";
	help += " Learns a visual model of an viewpoint of an object from synthetic generated images.\n";
	help += "\n";
	help += "Required Parameter\n";
	help += "==================\n";
	ADD_STRING_PARAM(mesh_file_name,"","The '.off' file to load, specified in units of centimeters.");
	help += "\n";
	help += "Optional Parameters\n";
	help += "===================\n";
	ADD_STRING_PARAM(view_output_file,"","The file in which to save the model. If unspecified, the view will be printed to stdout.");
	ADD_INT_PARAM(n_training_images,100,"The number of synthetic training images to generate.");
	help += "\n";
	help += "  Viewpoint Bin Parameters\n";
	help += "  ------------------------\n";
	ADD_PARAM(angle_bin_width,5,"The default width of a bin, in degrees, centered at one of the mean angles of rotation rx, ry or rz.");
	ADD_PARAM(rx,0,"The mean rotation of the object's viewpoint about the x axis, in the image plane (i.e. roll), in degrees.");
	ADD_PARAM(rx_bin_width,angle_bin_width,"The viewpoint rx will be sampled from the range [rx-rx_bin_width/2,rx+rx_bin_width/2], in degrees.");
	ADD_PARAM(ry,0,"The mean rotation of the object's viewpoint about the y axis, simulating the object translating vertically up and down (i.e. pitch), in degrees.");
	ADD_PARAM(ry_bin_width,angle_bin_width,"The viewpoint ry will be sampled from the range [ry-ry_bin_width/2,ry+ry_bin_width/2], in degrees.");
	ADD_PARAM(rz,0,"The mean rotation of the object's viewpoint about the z axis, as if on a horizontal turn-table (i.e. yaw), in degrees.");
	ADD_PARAM(rz_bin_width,angle_bin_width,"The viewpoint rz will be sampled from the range [rz-rz_bin_width/2,rz+rz_bin_width/2], in degrees. Setting this value to 360 is useful when the mesh is rotationally symmetric as it sits upright on a table.");
	ADD_INT_PARAM(rz_symmetry_order,0,"The number of times an object can be rotated on to itself around the z axis. A can has an infinite order of rotational symmetry (to specify this, use 0). A cube has rz_symmetry_order of 4.  A box of cereal has rz_symmetry_order of 2.  An asymmetric object has rz_symmetry of 1.  A larger value for rz_symmetry_order means that fewer viewpoints need to be learned, so learning and detection is more efficient.");
	ADD_PARAM(min_depth,0.5,"The minimum distance from the focal point of the camera to the nearest point of the bounding sphere of the object, in meters.");
	ADD_PARAM(max_depth,3,"The maximum distance from the focal point of the camera to the nearest point of the bounding sphere of the object, in meters.");
	help += "\n";
	help += "  Edge Feature Parameters\n";
	help += "  -----------------------\n";
	ADD_INT_PARAM(n_edge_features,100,"The desired number of edge features in the view.");
	ADD_PARAM(edge_receptive_field_radius,6.0,"The maximum distance from the mean edge feature location where edges in the image can improve the overall detection probability (measured in pixels scaled as when the object is 1 meter away).");
	ADD_PARAM(max_edge_position_variance,250,"The maximum allowable variance used when selecting edge features.");
	ADD_PARAM(min_edge_position_variance,0.0,"Used for regularization so that features with low variance will have their variance increased to this value.");
#ifdef TWO_THRESHOLD_FEATURES
	ADD_PARAM(high_threshold_log_probability_shift,2.0,"This value is added to those edge features that are stronger than the high threshold.");
#endif

	//todo: choose a best setting for the following four parameters and remove them
	ADD_PARAM(edge_low_threshold,800,"The low threshold used in the edge detector.");
	ADD_PARAM(edge_high_threshold,2400,"The high threshold used in the edge detector.");
	ADD_INT_PARAM(n_edge_directions,8,"The number of edge angle ranges to consider as separate edge directions.");
	ADD_PARAM(edge_bin_overlap,1.0,"The amount of overlap between edge bins, in the range of 0 and 1.  An overlap of 1.0 means, for example, that an edge bin from 5 to 15 degrees will overlap with a bin from 0 to 10 degrees and another from 10 to 20 degrees.");
	ADD_INT_PARAM(use_textured_mesh_for_edges,0,"If non-zero, any texture maps or vertex colors from the mesh file will be used while choosing edge features.");

	help += "\n";
	help += "  Texton Feature Parameters\n";
	help += "  -------------------------\n";
	ADD_STRING_PARAM(textons_file,"","A file defining the k texton clusters to be used in calculating texton features.");
	ADD_INT_PARAM(n_texton_features,0,"The desired number of edge features in the view.");
	ADD_PARAM(texton_receptive_field_radius,6.0,"The maximum distance from the mean edge feature location where edges in the image can improve the overall detection probability (measured in pixels scaled as when the object is 1 meter away).");
	ADD_PARAM(max_texton_position_variance,250,"The maximum allowable variance used when selecting edge features.");
	ADD_PARAM(min_texton_position_variance,0.0,"Used for regularization so that features with low variance will have their variance increased to this value.");

	help += "\n";
	help += "  Depth Feature Parameters\n";
	help += "  ------------------------\n";
	ADD_INT_PARAM(n_depth_features,100,"The desired number of depth features in the view.");
	ADD_PARAM(depth_receptive_field_radius,0.02,"The maximum difference in depth (either positive or negative) that a pixel's depth can deviate from the mean while still improving the overall detection probability (measured in meters).");
	ADD_PARAM(max_depth_feature_variance,0.02,"The maximum allowable variance of the depth (in meters^2) used when selecting edge features.");
	ADD_PARAM(min_depth_position_variance,0.0,"Used for regularization so that features with low variance will have their variance increased to this value.");
	help += "\n";
	help += "\n";
	help += "  Rendering Parameters\n";
	help += "  --------------------\n";
	ADD_PARAM(ambient_light,0.5,"The amount of ambient light in the rendered scene.  Should be in the range from 0.0 to 1.0.");
	//todo: add ambient textons parameter for the rendering when using texture
	help += "\n";
	help += "  Debugging/Other Parameters\n";
	help += "  --------------------------\n";
	ADD_INT_PARAM(random_seed,0,"The random seed used for drawing object pose samples.");
	ADD_INT_PARAM(verbose,0,"If non-zero, status information will be printed to stderr.");
	ADD_INT_PARAM(show_debug_images,0,"If non-zero, visualizations of the aggregate information for edge and depth features will be displayed (for debugging purposes).");
	ADD_STRING_PARAM(image_output_directory,"","If provided, scaled aligned image files will be saved in this directory. Used for generating training data for other algorithms to use.");

	CHECK_PARAMS();

	if(!mesh_file_name_is_passed) {
		std::cerr << "The mesh_file_name parameter is not passed." << std::endl;
		std::cerr << std::endl;
		std::cerr << help << std::endl;
		return 0;
	}

	if(textons_file=="" && n_texton_features>0) {
		std::cerr << "Use the -textons_file parameter to use texton features." << std::endl;
		return -1;
	}

	objrec::edge_detection_parameters edge_parameters(n_edge_directions, edge_bin_overlap, edge_low_threshold, edge_high_threshold);
	if(verbose!=0) {
		const std::vector<objrec::edge_direction>& edge_directions = edge_parameters.get_edge_directions();
		std::cerr << "Available edge directions (in degrees): ";
		for(int j=0;j<(int)edge_directions.size();j++) {
			std::cerr << "(" << edge_directions[j] << ") ";
		}
		std::cerr << std::endl;
	}

    long long start, end;

	objrec::textons textons_parameters;
	if(textons_file!="") {
		std::ifstream in(textons_file.c_str());
		if(!in) throw std::runtime_error("Could not open textons file '"+textons_file+"'.");
		if(verbose!=0) std::cerr << "Loading textons file " << textons_file << "... " << std::flush;
		start = objrec::current_time();
		int line_number = 0;
		textons_parameters = objrec::textons(in,line_number);
		end = objrec::current_time();
		if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}
	objrec::feature_library features(edge_parameters, textons_parameters);

	objrec::OpenGL_X_window win; //creates an OpenGL X window for rendering synthetic images
	objrec::mesh m;
	std::string mesh_name = objrec::basename(mesh_file_name);
	std::string mesh_extension = objrec::file_extension(mesh_file_name);

	if(verbose!=0) std::cerr << "Loading mesh " << mesh_file_name << "... " << std::flush;
	start = objrec::current_time();
	if(mesh_extension=="obj") m = objrec::load_obj(mesh_file_name);
	else if(mesh_extension=="off") m = objrec::load_off(mesh_file_name);
	else throw std::runtime_error("unrecognized mesh file extension '"+mesh_extension+"'. must be 'obj' or 'off'.");
	end = objrec::current_time();
	if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	objrec::object obj;
	obj.library = features;
	obj.name = mesh_name;
	obj.object_center_height_above_table = m.origin_height_above_table;

	srand(abs(random_seed)+2); //something above changes the random seed on some systems

	objrec::view v;
	objrec::learn_viewpoint(rx, ry, rz, rx_bin_width, ry_bin_width, rz_bin_width, rz_symmetry_order,
			min_depth, max_depth, features, m, win, mesh_name,
			ambient_light, use_textured_mesh_for_edges, n_training_images, image_output_directory, verbose,
			n_edge_features, edge_receptive_field_radius, max_edge_position_variance, min_edge_position_variance,
#ifdef TWO_THRESHOLD_FEATURES
			high_threshold_log_probability_shift,
#endif
			n_texton_features, texton_receptive_field_radius, max_texton_position_variance, min_texton_position_variance,
			n_depth_features, depth_receptive_field_radius, max_depth_feature_variance, min_depth_position_variance,
			//output parameter:
			v,
			//debugging:
			show_debug_images);

	obj.views.push_back(v);

	//output the object
	if(view_output_file_is_passed) {
		std::ofstream out(view_output_file.c_str());
		out << obj;
	} else {
		std::cout << obj;
	}

//	if(!mesh_file_name_is_passed) {
//		std::cerr << "The mesh_file_name parameter is not passed." << std::endl;
//		std::cerr << std::endl;
//		std::cerr << help << std::endl;
//		return 0;
//	}
//
//	if(textons_file=="" && n_texton_features>0) {
//		std::cerr << "Use the -textons_file parameter to use texton features." << std::endl;
//		return -1;
//	}
//
//	objrec::edge_detection_parameters edge_parameters(n_edge_directions, edge_bin_overlap, edge_low_threshold, edge_high_threshold);
//	if(verbose!=0) {
//		const std::vector<objrec::edge_direction>& edge_directions = edge_parameters.get_edge_directions();
//		std::cerr << "Available edge directions (in degrees): ";
//		for(int j=0;j<(int)edge_directions.size();j++) {
//			std::cerr << "(" << edge_directions[j] << ") ";
//		}
//		std::cerr << std::endl;
//	}
//
//    long long start, end;
//
//	objrec::textons textons_parameters;
//	if(textons_file!="") {
//		std::ifstream in(textons_file.c_str());
//		if(!in) throw std::runtime_error("Could not open textons file '"+textons_file+"'.");
//		if(verbose!=0) std::cerr << "Loading textons file " << textons_file << "... " << std::flush;
//		start = objrec::current_time();
//		int line_number = 0;
//		textons_parameters = objrec::textons(in,line_number);
//		end = objrec::current_time();
//		if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
//	}
//	objrec::feature_library features(edge_parameters, textons_parameters);
//
//	objrec::OpenGL_X_window win; //creates an OpenGL X window for rendering synthetic images
//	objrec::mesh m;
//	std::string mesh_name = objrec::basename(mesh_file_name);
//	std::string mesh_extension = objrec::file_extension(mesh_file_name);
//
//	if(verbose!=0) std::cerr << "Loading mesh " << mesh_file_name << "... " << std::flush;
//	start = objrec::current_time();
//	if(mesh_extension=="obj") m = objrec::load_obj(mesh_file_name);
//	else if(mesh_extension=="off") m = objrec::load_off(mesh_file_name);
//	else throw std::runtime_error("unrecognized mesh file extension '"+mesh_extension+"'. must be 'obj' or 'off'.");
//	end = objrec::current_time();
//	if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
//
//	objrec::viewpoint vp;
//	if(rz_symmetry_order==0) {
//		vp.min_rz = rz; vp.max_rz = rz+.1;
//	} else {
//		vp.min_rz = rz - rz_bin_width/2.0; vp.max_rz = rz + rz_bin_width/2.0;
//	}
//	vp.min_ry = ry - ry_bin_width/2.0; vp.max_ry = ry + ry_bin_width/2.0;
//	vp.min_rx = rx - rx_bin_width/2.0; vp.max_rx = rx + rx_bin_width/2.0;
//	vp.min_depth = min_depth+m.radius; vp.max_depth = max_depth+m.radius;
//
//	srand(abs(random_seed)+2); //something above changes the random seed on some systems
//	if(verbose!=0) std::cerr << "Gathering statistics from synthetic training data... " << std::flush;
//    start = objrec::current_time();
//
//	objrec::enumerated_features f(features,m,mesh_name,win,ambient_light,use_textured_mesh_for_edges!=0,
//			vp,rz_symmetry_order,n_training_images,image_output_directory);
//
//	end = objrec::current_time();
//    if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
//
//	if(verbose!=0) std::cerr << "Calculating feature parameters... " << std::flush;
//    start = objrec::current_time();
//    f.calculate_enumerated_features();
//    end = objrec::current_time();
//	if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
//
//    if(show_debug_images!=0) f.show();
//
//	if(verbose!=0) std::cerr << "Sorting features by variance... " << std::flush;
//    start = objrec::current_time();
//    f.sort_enumerated_features();
//    end = objrec::current_time();
//	if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
//
//
//	if(verbose!=0) std::cerr << "Selecting features... " << std::flush;
//	start = objrec::current_time();
//
//	objrec::object obj;
//	obj.object_center_height_above_table = m.origin_height_above_table;
//	f.select_features(
//			n_edge_features,  edge_receptive_field_radius,  max_edge_position_variance, min_edge_position_variance,
//#ifdef TWO_THRESHOLD_FEATURES
//			high_threshold_log_probability_shift,
//#endif
//			n_texton_features,  texton_receptive_field_radius,  max_texton_position_variance, min_texton_position_variance,
//			n_depth_features, depth_receptive_field_radius, max_depth_feature_variance, min_depth_position_variance,
//			//output parameter:
//			obj);
//
//	obj.library = objrec::feature_library(edge_parameters,textons_parameters); //todo: fix this hack. included so that simulated edges are not affected by precomputed edges
//	end = objrec::current_time();
//    if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
//
//	//output the object
//	if(view_output_file_is_passed) {
//		std::ofstream out(view_output_file.c_str());
//		out << obj;
//	} else {
//		std::cout << obj;
//	}
}

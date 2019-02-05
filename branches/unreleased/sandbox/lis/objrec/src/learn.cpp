/*
 * learn.cpp
 *
 *  Created on: Aug 25, 2014
 */

#include <mpi.h>
#include "learning.h"
#include "args.h"

namespace objrec {

#define INITIALIZED_TAG 1
#define ASSIGN_VIEW_JOB_TAG 2
struct view_job {
	double rx, ry, rz;
	int job_number;
	view_job(double rx, double ry, double rz, int job_number) : rx(rx), ry(ry), rz(rz), job_number(job_number) {}
	view_job() {}
};
#define RESULT_INFO_TAG 3
struct result_info_message {
	int job_number;
	double time_seconds;
	int string_length;
};
#define RESULT_VIEW_TAG 4
#define ALL_FINISHED_TAG 5


void check_finished() {
	int message_received;
	MPI_Status status;
	MPI_Iprobe(0,ALL_FINISHED_TAG,MPI_COMM_WORLD,&message_received,&status);
	if(message_received) {
		int place_holder;
		MPI_Recv(&place_holder,0,MPI_INT,0,ALL_FINISHED_TAG,MPI_COMM_WORLD,&status);
		MPI_Finalize();
		exit(0);
	}
}

void learn(int rank, double rx_bin_width, double ry_bin_width, double rz_bin_width, int rz_symmetry_order,
		double min_depth, double max_depth, feature_library& features, const mesh& m, OpenGL_X_window& win,
		double ambient_light, int use_textured_mesh_for_edges, int n_training_images, const std::string& image_output_directory,
		int n_edge_features,  double edge_receptive_field_radius,  double max_edge_position_variance, double min_edge_position_variance,
#ifdef TWO_THRESHOLD_FEATURES
		double high_threshold_log_probability_shift,
#endif
		int n_texton_features,  double texton_receptive_field_radius,  double max_texton_position_variance, double min_texton_position_variance,
		int n_depth_features, double depth_receptive_field_radius, double max_depth_feature_variance, double min_depth_position_variance) {

	int place_holder = 0;
	MPI_Send(&place_holder,0,MPI_INT,0,INITIALIZED_TAG,MPI_COMM_WORLD);
	int verbose = 0;
	std::string mesh_name = "";

	while(true) {
		MPI_Status status;
		MPI_Probe(0,MPI_ANY_TAG,MPI_COMM_WORLD,&status);
		check_finished();
		{ //status.MPI_TAG==ASSIGN_VIEW_JOB_TAG
			view_job j;
			MPI_Recv(&j,sizeof(view_job)/sizeof(char),MPI_CHAR,0,ASSIGN_VIEW_JOB_TAG,MPI_COMM_WORLD,&status);
			view v;
			long long start, end;
			start = objrec::current_time();
			objrec::learn_viewpoint(j.rx, j.ry, j.rz, rx_bin_width, ry_bin_width, rz_bin_width, rz_symmetry_order,
					min_depth, max_depth, features, m, win, mesh_name,
					ambient_light, use_textured_mesh_for_edges, n_training_images, image_output_directory, verbose,
					n_edge_features, edge_receptive_field_radius, max_edge_position_variance, min_edge_position_variance,
#ifdef TWO_THRESHOLD_FEATURES
					high_threshold_log_probability_shift,
#endif
					n_texton_features, texton_receptive_field_radius, max_texton_position_variance, min_texton_position_variance,
					n_depth_features, depth_receptive_field_radius, max_depth_feature_variance, min_depth_position_variance,
					//output parameter:
					v);
			end = objrec::current_time();
			std::ostringstream out;
			out << v << std::flush;
			int len = out.str().size()+1; //include null terminator
			char* buffer = new char[len];
			strcpy(buffer,out.str().c_str());
			result_info_message r;
			r.job_number = j.job_number;
			r.string_length = len;
			r.time_seconds = (double)(end-start)/1000000.0;
			check_finished();
			MPI_Send(&r,sizeof(result_info_message)/sizeof(char),MPI_CHAR,0,RESULT_INFO_TAG,MPI_COMM_WORLD);
			check_finished();
			MPI_Send(buffer,len,MPI_CHAR,0,RESULT_VIEW_TAG,MPI_COMM_WORLD);
			check_finished();
			delete[] buffer;
		}
	}
}

struct CPU_info {
	double last_completed_job_time_seconds;
	bool has_initialized;
	CPU_info() : last_completed_job_time_seconds(std::numeric_limits<double>::quiet_NaN()), has_initialized(false) {};
};

struct job_info {
	view_job j;
	std::vector<int> assigned_to;
	bool completed;
	job_info(double rx, double ry, double rz, int job_number) : j(rx,ry,rz,job_number), completed(false) {}
};

int n_unfinished_jobs(const std::vector<job_info>& to_do) {
	int n = 0;
	for(int j=0;j<to_do.size();j++) {
		if(to_do[j].completed!=true) {
			n++;
		}
	}
	return n;
}

void assign_next_job(int rank,std::vector<job_info>& to_do, const std::vector<CPU_info>& cpus) {
	int fewest_assignments_of_a_job = std::numeric_limits<int>::max();
	for(int j=0;j<to_do.size();j++) {
		if(!to_do[j].completed) {
			fewest_assignments_of_a_job = std::min(fewest_assignments_of_a_job,(int)to_do[j].assigned_to.size());
		}
	}
	double longest_estimated_time_to_complete = 0;
	std::vector<int> jobs_with_longest_estimated_time_to_complete;
	for(int j=0;j<to_do.size();j++) {
		if(!to_do[j].completed && to_do[j].assigned_to.size()==fewest_assignments_of_a_job) {
			double est_j = std::numeric_limits<double>::infinity();
			for(int k=0;k<to_do[j].assigned_to.size();k++) {
				const CPU_info& cpu = cpus[to_do[j].assigned_to[k]];
				if(cpu.has_initialized) {
					est_j = std::min(est_j,cpu.last_completed_job_time_seconds);
				} else {
					est_j = std::min(est_j,std::numeric_limits<double>::infinity());
				}
			}
			if(est_j==longest_estimated_time_to_complete) {
				jobs_with_longest_estimated_time_to_complete.push_back(j);
			} else if(est_j>longest_estimated_time_to_complete) {
				longest_estimated_time_to_complete = est_j;
				jobs_with_longest_estimated_time_to_complete.clear();
				jobs_with_longest_estimated_time_to_complete.push_back(j);
			}
		}
	}
	if(jobs_with_longest_estimated_time_to_complete.size()!=0) {
		int j = rand()%jobs_with_longest_estimated_time_to_complete.size();
		job_info& job = to_do[jobs_with_longest_estimated_time_to_complete[j]];
		job.assigned_to.push_back(rank);
		MPI_Send(&job,sizeof(view_job)/sizeof(char),MPI_CHAR,rank,ASSIGN_VIEW_JOB_TAG,MPI_COMM_WORLD);
	}
}

void progress(long long start, int j, int n) {
    struct timeval tv;
    gettimeofday(&tv, 0);
    long long now = ((long long)tv.tv_sec)*1000000 + tv.tv_usec;
    double elapsed = (double)(now-start)/1000000.0;
    if(j!=0) {
    	double remaining = elapsed/(double)j*(n-j);

    	std::cerr << "learned " << j << " of " << n
    			<<" views. (" << round(100.0f*(float)(j)/(float)n) << "%) ";
    	std::cerr << time_string(elapsed) << " elapsed, " << time_string(remaining) << " left.    \r" << std::flush;
    } else {
    	std::cerr << "learned 0 of " << n << " views.\r" << std::flush;
    }
}

void coordinate_learning(double rx_bin_width, double ry_bin_width, double rz_bin_width, int rz_symmetry_order, int verbose,
		//output parameter:
		object& obj) {
	std::vector<job_info> to_do;
	int job_number = 0;
	double rx_start = -30;
	double rx_stop = 30;
	for(double rx=rx_bin_width*floor(rx_start/rx_bin_width);rx<rx_stop;rx+=rx_bin_width) {
		double ry_start = -90;
		double ry_stop = 0;
		for(double ry=ry_start;ry<ry_stop;ry+=ry_bin_width) {
			double rz_start = 0;
			double rz_stop;
			if(rz_symmetry_order==0) {
				rz_stop = 1;
			} else {
				rz_stop = 360.0/rz_symmetry_order;
			}
			for(double rz=rz_start;rz<rz_stop;rz+=rz_bin_width) {
				to_do.push_back(job_info(rx,ry,rz,job_number));
				job_number++;
			}
		}
	}

	int n_cpus;
	MPI_Comm_size(MPI_COMM_WORLD,&n_cpus);
	std::vector<CPU_info> cpus(n_cpus);

	long long start, end;
	start = objrec::current_time();

	while(n_unfinished_jobs(to_do)>0) {
		MPI_Status status;
		int message_received=0;
		int poll_count = 0;
		while(!message_received) {
			MPI_Iprobe(MPI_ANY_SOURCE,MPI_ANY_TAG,MPI_COMM_WORLD,&message_received,&status);
			poll_count++;
			if(poll_count%10000==0) {
				if(verbose) progress(start,to_do.size()-n_unfinished_jobs(to_do),to_do.size());
			}
		}
		switch(status.MPI_TAG) {
		case INITIALIZED_TAG: {
			int place_holder;
			MPI_Recv(&place_holder,0,MPI_INT,status.MPI_SOURCE,INITIALIZED_TAG,MPI_COMM_WORLD,&status);
			cpus[status.MPI_SOURCE].has_initialized = true;
			assign_next_job(status.MPI_SOURCE,to_do,cpus);
			break;
		}
		case RESULT_INFO_TAG: {
			result_info_message r;
			MPI_Recv(&r,sizeof(result_info_message)/sizeof(char),MPI_CHAR,status.MPI_SOURCE,RESULT_INFO_TAG,MPI_COMM_WORLD,&status);
			char* view_string = new char[r.string_length];
			MPI_Recv(view_string,r.string_length,MPI_CHAR,status.MPI_SOURCE,RESULT_VIEW_TAG,MPI_COMM_WORLD,&status);
			if(to_do[r.job_number].completed==false) {
				to_do[r.job_number].completed = true;
				std::string view_str(view_string);
				std::istringstream in(view_str);
				int line_number = 0;
				obj.views.push_back(view(&obj,in,line_number));
			}
			delete[] view_string;
			assign_next_job(status.MPI_SOURCE,to_do,cpus);
			break;
		}
		}
	}
	if(verbose) progress(start,to_do.size()-n_unfinished_jobs(to_do),to_do.size());
	std::cerr << std::endl;
	int place_holder = 0;
	MPI_Request request;
	for(int j=1;j<n_cpus;j++) {
		MPI_Isend(&place_holder,0,MPI_INT,j,ALL_FINISHED_TAG,MPI_COMM_WORLD,&request);
	}
}

} //namespace objrec

int main(int argc, char** argv) {
	MPI_Init(&argc,&argv);
	int rank;
	MPI_Comm_rank(MPI_COMM_WORLD,&rank);
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
	ADD_STRING_PARAM(output_file,"","The file in which to save the model. If unspecified, the view will be printed to stdout.");
	ADD_INT_PARAM(n_training_images,100,"The number of synthetic training images to generate.");
	help += "\n";
	help += "  Viewpoint Bin Parameters\n";
	help += "  ------------------------\n";
	ADD_PARAM(angle_bin_width,20,"The default width of a bin, in degrees, centered at one of the mean angles of rotation rx, ry or rz.");
	ADD_PARAM(rx_bin_width,angle_bin_width,"The viewpoint rx will be sampled from the range [rx-rx_bin_width/2,rx+rx_bin_width/2], in degrees.");
	ADD_PARAM(ry_bin_width,angle_bin_width,"The viewpoint ry will be sampled from the range [ry-ry_bin_width/2,ry+ry_bin_width/2], in degrees.");
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
	ADD_INT_PARAM(verbose,1,"If non-zero, status information will be printed to stderr.");
	ADD_INT_PARAM(show_debug_images,0,"If non-zero, visualizations of the aggregate information for edge and depth features will be displayed (for debugging purposes).");
	ADD_STRING_PARAM(image_output_directory,"","If provided, scaled aligned image files will be saved in this directory. Used for generating training data for other algorithms to use.");

	CHECK_PARAMS();

	if(rank==0 && !mesh_file_name_is_passed) {
		std::cerr << "The mesh_file_name parameter is not passed." << std::endl;
		std::cerr << std::endl;
		std::cerr << help << std::endl;
		return -1;
	}

	if(rank==0 && textons_file=="" && n_texton_features>0) {
		std::cerr << "Use the -textons_file parameter to use texton features." << std::endl;
		return -1;
	}

	objrec::edge_detection_parameters edge_parameters(n_edge_directions, edge_bin_overlap, edge_low_threshold, edge_high_threshold);
	if(rank==0 && verbose!=0) {
		const std::vector<objrec::edge_direction>& edge_directions = edge_parameters.get_edge_directions();
		std::cerr << "Available edge directions (in degrees): ";
		for(int j=0;j<(int)edge_directions.size();j++) {
			std::cerr << "(" << edge_directions[j] << ") ";
		}
		std::cerr << std::endl;
	}

	objrec::check_finished();
    long long start, end;


	objrec::textons textons_parameters;
	if(textons_file!="") {
		std::ifstream in(textons_file.c_str());
		if(!in) throw std::runtime_error("Could not open textons file '"+textons_file+"'.");
		if(rank==0 && verbose!=0) std::cerr << "Loading textons file " << textons_file << "... " << std::flush;
		start = objrec::current_time();
		int line_number = 0;
		textons_parameters = objrec::textons(in,line_number);
		end = objrec::current_time();
		if(rank==0 && verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}

	objrec::check_finished();
	objrec::feature_library features(edge_parameters, textons_parameters);

	objrec::OpenGL_X_window win; //creates an OpenGL X window for rendering synthetic images
	objrec::mesh m;
	std::string mesh_extension = objrec::file_extension(mesh_file_name);
	if(rank==0 && verbose!=0) std::cerr << "Loading mesh " << mesh_file_name << "... " << std::flush;
	start = objrec::current_time();
	if(mesh_extension=="obj") m = objrec::load_obj(mesh_file_name);
	else if(mesh_extension=="off") m = objrec::load_off(mesh_file_name);
	else throw std::runtime_error("unrecognized mesh file extension '"+mesh_extension+"'. must be 'obj' or 'off'.");
	end = objrec::current_time();
	if(rank==0 && verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	if(rank==0) {
		objrec::object obj;
		obj.library = features;
		obj.name = objrec::basename(mesh_file_name);
		obj.object_center_height_above_table = m.origin_height_above_table;
		objrec::coordinate_learning(rx_bin_width, ry_bin_width, rz_bin_width, rz_symmetry_order, verbose,
				//output parameter:
				obj);
		if(output_file_is_passed) {
			std::ofstream out(output_file.c_str());
			out << obj;
		} else {
			std::cout << obj;
		}
//		MPI_Abort(MPI_COMM_WORLD,0);
	} else {

		srand(abs(random_seed)+2); //something above changes the random seed on some systems

		objrec::learn(rank, rx_bin_width, ry_bin_width, rz_bin_width, rz_symmetry_order,
				min_depth, max_depth, features, m, win,
				ambient_light, use_textured_mesh_for_edges, n_training_images, image_output_directory,
				n_edge_features, edge_receptive_field_radius, max_edge_position_variance, min_edge_position_variance,
#ifdef TWO_THRESHOLD_FEATURES
				high_threshold_log_probability_shift,
#endif
				n_texton_features, texton_receptive_field_radius, max_texton_position_variance, min_texton_position_variance,
				n_depth_features, depth_receptive_field_radius, max_depth_feature_variance, min_depth_position_variance);
	}
	MPI_Finalize();
}

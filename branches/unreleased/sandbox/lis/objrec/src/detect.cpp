/*
 * detect.cpp
 *
 *  Created on: Mar 5, 2014
 */

#include "constraints.h"
#include "args.h"
#include <mpi.h>

namespace objrec {


int find_view_index(const hypothesis& h) {
	const std::vector<view>& views = h.get_view()->get_parent_object()->views;
	for(int view_index=0;view_index<views.size();view_index++) {
		if(h.get_view()==&views[view_index]) {
			return view_index;
		}
	}
	throw std::runtime_error("Parent object of a view does not contain that view!");
}



#define N_HYPOTHESES_TAG 0
struct n_hypotheses_message {
	int n_remaining;
	int n_evaluated;
	double n_hypotheses_per_second;
	int delegate_from;
	n_hypotheses_message(int n_remaining, int n_evaluated, double n_hypotheses_per_second, int delegate_from = -1)
	: n_remaining(n_remaining), n_evaluated(n_evaluated), n_hypotheses_per_second(n_hypotheses_per_second),
	  delegate_from(delegate_from) {}
	n_hypotheses_message() {}
};

#define LEAF_HYPOTHESIS_TAG 1
struct leaf_hypothesis_message {
	int view_index;
	point p;
	float log_probability;

	leaf_hypothesis_message(const hypothesis& h) : view_index(find_view_index(h)),
			p(h.get_region().min), log_probability(h.get_bound()) {}
	leaf_hypothesis_message() {}
};

#define DELEGATE_HYPOTHESES_TAG 2
struct delegate_hypothesis_message {
	int view_index;
	region r;
	float log_probability; //todo: remove this member

	delegate_hypothesis_message(const hypothesis& h) : view_index(find_view_index(h)),
			r(h.get_region()), log_probability(h.get_bound()) {}
	delegate_hypothesis_message() {}
	hypothesis to_hypothesis(const object& obj, const image& im) {
		if(r.min==r.max) {
			return hypothesis(&obj.views[view_index],im,r.min);
		} else {
			return hypothesis(&obj.views[view_index],im,r);
		}
	}
};

#define START_DELEGATION_TAG 3
struct start_delegation_message {
	int delegate_to_rank;
	double n_hypotheses_per_second;
};


//#define HOSTNAME_TAG 4

#define FINISHED_TAG 5
bool is_finished() {
	MPI_Status status;
	int message_arrived;
	MPI_Iprobe(MPI_ANY_SOURCE,FINISHED_TAG,MPI_COMM_WORLD,&message_arrived,&status);
	if(message_arrived) {
		int place_holder;
		MPI_Recv(&place_holder,0,MPI_INT,MPI_ANY_SOURCE,FINISHED_TAG,MPI_COMM_WORLD,&status);
		return true;
	} else {
		return false;
	}
}


template <class search_space_constraint>
void check_delegate_to(double n_hypotheses_per_second, int n_hypotheses_evaluated, detector<search_space_constraint>& d) {
	MPI_Status status;
	int message_arrived;
	MPI_Iprobe(MPI_ANY_SOURCE,START_DELEGATION_TAG,MPI_COMM_WORLD,&message_arrived,&status);
	if(message_arrived) {
		start_delegation_message sdm;
		MPI_Recv(&sdm,sizeof(start_delegation_message)/sizeof(float),MPI_FLOAT,0,START_DELEGATION_TAG,MPI_COMM_WORLD,&status);

		int n_hypotheses_to_delegate =
				sdm.n_hypotheses_per_second/(n_hypotheses_per_second+sdm.n_hypotheses_per_second)*d.n_remaining_hypotheses();
		try {
		delegate_hypothesis_message* dhm = new delegate_hypothesis_message[n_hypotheses_to_delegate];

		for(int j=0;j<n_hypotheses_to_delegate;j++) {
			dhm[j] = delegate_hypothesis_message(d.best_hypothesis());
			d.remove_best_hypothesis();
		}
		MPI_Send(&n_hypotheses_to_delegate,1,MPI_INT,sdm.delegate_to_rank,DELEGATE_HYPOTHESES_TAG,MPI_COMM_WORLD);
		MPI_Send(dhm,sizeof(delegate_hypothesis_message)/sizeof(float)*n_hypotheses_to_delegate,
				MPI_FLOAT,sdm.delegate_to_rank,DELEGATE_HYPOTHESES_TAG,MPI_COMM_WORLD);
		delete[] dhm;
		}catch(std::bad_alloc& e) {
			std::cerr << "sdm.n_hypotheses_per_second: " << sdm.n_hypotheses_per_second << std::endl;
			std::cerr << "n_hypotheses_per_second: " << sdm.n_hypotheses_per_second << std::endl;
			std::cerr << "d.n_remaining_hypotheses(): " << d.n_remaining_hypotheses() << std::endl;
			std::cerr << "sdm.n_hypotheses_per_second==std::numeric_limits<double>::infinity(): " << (sdm.n_hypotheses_per_second==std::numeric_limits<double>::infinity()) << std::endl;
			std::cerr << "n_hypotheses_per_second==std::numeric_limits<double>::infinity(): " << (n_hypotheses_per_second==std::numeric_limits<double>::infinity()) << std::endl;
		}
	}
}
template <class search_space_constraint>
int check_delegate_from(detector<search_space_constraint>& d, const objrec::object& obj, const objrec::image& im) {
	if(is_finished()) {
		return std::numeric_limits<int>::max();
	}
	MPI_Status status;
	int message_arrived;

	MPI_Iprobe(MPI_ANY_SOURCE,DELEGATE_HYPOTHESES_TAG,MPI_COMM_WORLD,&message_arrived,&status);
	if(message_arrived) {
		int n_hypotheses_delegated;
		MPI_Recv(&n_hypotheses_delegated,1,MPI_INT,MPI_ANY_SOURCE,DELEGATE_HYPOTHESES_TAG,MPI_COMM_WORLD,&status);
		delegate_hypothesis_message* dhm = new delegate_hypothesis_message[n_hypotheses_delegated];
		MPI_Recv(dhm,sizeof(delegate_hypothesis_message)/sizeof(float)*n_hypotheses_delegated,
				MPI_FLOAT,status.MPI_SOURCE,DELEGATE_HYPOTHESES_TAG,MPI_COMM_WORLD,&status);
		for(int j=0;j<n_hypotheses_delegated;j++) {
			hypothesis h = dhm[j].to_hypothesis(obj,im);
			d.add_hypothesis(h);
		}
		delete[] dhm;
		return status.MPI_SOURCE;
	} else {
		return -1;
	}
}

const double delegate_percent_of_queue_size = 0.1;
const int max_queue_size = 3000000;
template <class search_space_constraint>
void detect(const object& obj, const image& im, const search_space_constraint& constraint, float lower_bound, int rank) {
	int n_cpus;
	MPI_Comm_size(MPI_COMM_WORLD,&n_cpus);

	detector<search_space_constraint> d(&obj,&im,constraint);
	hypothesis::priority_queue q;//(q_initial);
	int n_hypotheses_evaluated = 0;

	int delegated_from = -1;

	double n_hypotheses_per_second = 50;//std::numeric_limits<double>::quiet_NaN();
	while(delegated_from!=std::numeric_limits<int>::max()) {
		MPI_Status status;
		int n_remaining_hypotheses = d.n_remaining_hypotheses();
		//send queue size, receive current lower bound
		n_hypotheses_message nhm(n_remaining_hypotheses,n_hypotheses_evaluated,n_hypotheses_per_second,delegated_from);
		MPI_Send(&nhm,sizeof(n_hypotheses_message)/sizeof(int),MPI_INT,0,N_HYPOTHESES_TAG,MPI_COMM_WORLD);
		int lower_bound_message_received = 0, finished_message_received = 0;
		while(!lower_bound_message_received && !finished_message_received) {
			MPI_Iprobe(0,N_HYPOTHESES_TAG,MPI_COMM_WORLD,&lower_bound_message_received,&status);
			MPI_Iprobe(0,FINISHED_TAG,MPI_COMM_WORLD,&finished_message_received,&status);
		}
		if(finished_message_received) {
			break;
		}
		MPI_Recv(&lower_bound,1,MPI_FLOAT,0,N_HYPOTHESES_TAG,MPI_COMM_WORLD,&status);
		delegated_from = -1;
		while(d.n_remaining_hypotheses()==0 && delegated_from<0) { //if the queue is empty, wait for a delegation or the end of the search
			check_delegate_to(n_hypotheses_per_second,n_hypotheses_evaluated,d);
			delegated_from = check_delegate_from(d,obj,im);
		}
		if(delegated_from>=0) continue;
		check_delegate_to(n_hypotheses_per_second,n_hypotheses_evaluated,d);
		delegated_from = check_delegate_from(d,obj,im);
		if(delegated_from>=0) continue;

		if(d.n_remaining_hypotheses() > max_queue_size) {
			//may receive delegation even if the queue is not empty if another CPU's queue is over full
			continue;
		}

		long long start = objrec::current_time();
		int start_n_hypotheses_evaluated = n_hypotheses_evaluated;
		for(int j=0;j<1000;j++) {
			hypothesis h = d.best_hypothesis();
			d.remove_best_hypothesis();
			n_hypotheses_evaluated++;
			if(h.is_leaf() && h.get_bound()>=lower_bound) {
				//			if(d.add_child_hypotheses(h,lower_bound)) {
				//send h to the coordinator
				leaf_hypothesis_message lhm(h);
				MPI_Send(&lhm,sizeof(leaf_hypothesis_message)/sizeof(float),MPI_FLOAT,0,LEAF_HYPOTHESIS_TAG,MPI_COMM_WORLD);
			} else {
				try {
					d.add_child_hypotheses(h,lower_bound);
				} catch(std::bad_alloc& e) {
					std::cerr << "rank " << rank << " out of memory with queue size of " << d.n_remaining_hypotheses() << std::endl;
					throw e;
				}
			}

			long long end = objrec::current_time();
			int end_n_hypotheses_evaluated = n_hypotheses_evaluated;
			double seconds = (double)(end-start)/1000000.0;
			n_hypotheses_per_second = (double)(end_n_hypotheses_evaluated - start_n_hypotheses_evaluated)/seconds;
			if(n_hypotheses_per_second==std::numeric_limits<double>::infinity()) {
				n_hypotheses_per_second = 100;
			}
			check_delegate_to(n_hypotheses_per_second,n_hypotheses_evaluated,d);

			if(d.n_remaining_hypotheses()==0) {
				break;
			}
		}
	}
}

class CPUs {
	struct cnpair {
		cnpair(unsigned int cpu, unsigned int n_remaining_hypotheses)
		: cpu(cpu), n_remaining_hypotheses(n_remaining_hypotheses) {}
		unsigned int cpu;
		unsigned int n_remaining_hypotheses;
		inline operator unsigned long() const {
			return (((unsigned long)n_remaining_hypotheses)<<(sizeof(unsigned int)*8)) | (unsigned long) cpu;
		}
	};
	struct compare : public std::binary_function<const cnpair&,const cnpair &,bool> {
		bool operator()(const cnpair& c1, const cnpair& c2) {
			const unsigned int n_cpus = 1;//cpus.cnvec.size();
			return (unsigned long)c1 > (unsigned long)c2;
		}
	};
	std::vector<unsigned int> cnvec;
	std::vector<unsigned int> n_evaluated_vec;
	std::set<cnpair, compare> cnset;
	std::vector<bool> first_message_received;
	std::vector<double> n_hypotheses_per_second;
	std::vector<bool> is_delegating;
	bool show_monitor;
	cv::Mat_<cv::Vec3b> display;
	int current_rank;
public:
	CPUs(int n_cpus, unsigned int n_views) : cnvec(n_cpus), n_evaluated_vec(n_cpus,0),
	first_message_received(n_cpus,false),
	n_hypotheses_per_second(n_cpus,std::numeric_limits<double>::quiet_NaN()), is_delegating(n_cpus,false), current_rank(1) {
		for(unsigned int j=1;j<cnvec.size();j++) { //the first cpu is this coordinator
			unsigned int n;
			if(j<=n_views) {
				n = 1;
			} else {
				n = 0;
			}
			cnvec[j] = n;
			cnset.insert(cnpair(j,n));
		}
		show_monitor = false;
	}
	void set_n_hypotheses_per_second(unsigned int rank, double n_hypotheses_per_second) {
		this->n_hypotheses_per_second[rank] = n_hypotheses_per_second;
	}
	const double get_n_hypotheses_per_second(unsigned int rank) const { return n_hypotheses_per_second[rank]; }

	void update_n_hypotheses(unsigned int rank, unsigned int n_remaining, unsigned int n_evaluated) {
		first_message_received[rank] = true;
		unsigned int old_n_remaining = cnvec[rank];
		cnvec[rank] = n_remaining;
		n_evaluated_vec[rank] = n_evaluated;
		int success = cnset.erase(cnpair(rank,old_n_remaining));
		if(!success) {
			throw std::runtime_error("problem with internal representation of cpus");
		}
		cnset.insert(cnpair(rank,n_remaining));
	}
	bool any_first_messages_received() const {
		for(int j=1;j<cnvec.size();j++) {
			if(first_message_received[j]) {
				return true;
			}
		}
		return false;
	}
	unsigned int get_n_evaluated(int rank) const {return n_evaluated_vec[rank]; }
	unsigned int get_n_remaining(int rank) const {return cnvec[rank]; }
	unsigned int rank_with_most_remaining_hypotheses() const { return cnset.begin()->cpu; }
	void set_is_delegating(int rank, bool value) {
		is_delegating[rank] = value;
	}
	int get_n_delegating() const {
		int n_delegating = 0;
		for(int j=0;j<cnvec.size();j++) {
			if(is_delegating[j]) {
				n_delegating++;
			}
		}
		return n_delegating;
	}
	int next_delegation_to() const {
		int best_rank = -1;
		double most_hypotheses_per_second = -std::numeric_limits<double>::infinity();
		for(int j=1;j<cnvec.size();j++) {
			if(!is_delegating[j]
			                  && first_message_received[j]
			                  && get_n_remaining(j)==0
			                  && n_hypotheses_per_second[j]>most_hypotheses_per_second) {
				best_rank = j;
				most_hypotheses_per_second = n_hypotheses_per_second[j];
			}
		}
		return best_rank;
	}
	int next_delegation_from() const {
		int best_rank = -1;
		unsigned int most_remaining_hypotheses = 0;
		for(int j=1;j<cnvec.size();j++) {
			int n_remaining = get_n_remaining(j);
			if(!is_delegating[j]
			                  && n_remaining>most_remaining_hypotheses
			                  && n_remaining>5) {
				best_rank = j;
				most_remaining_hypotheses = n_remaining;
			}
		}
		return best_rank;
	}
	void print() {
		for(int j=1;j<cnvec.size();j++) {
			std::cerr << cnvec[j] << " "; // << "/" << n_evaluated_vec[j] << " ";
		}
		std::cerr << "    " << std::flush;
	}
	int write_next_cell(const std::string& text,
			//output parameters
			int& current_x, int& current_y, int& max_column_width) {
		int font_face = cv::FONT_HERSHEY_PLAIN;
		cv::Scalar color(0,0,0);
		double scale = 1;
		int thickness = 1;
		int line_type = 8;
		int base_line;
		cv::Size sz = cv::getTextSize(text,font_face,scale,thickness,&base_line);
		max_column_width = std::max(max_column_width,sz.width);
		current_y += sz.height+base_line;
		cv::Point p(current_x,current_y);
		cv::putText(display,text,p,font_face,scale,color,thickness,line_type);
		return sz.width;
	}
	int write_next_cell(int n,
			//output parameters
			int& current_x, int& current_y, int& max_column_width) {
		std::ostringstream out;
		out << n;
		return write_next_cell(out.str(),
				//output parameters
				current_x,current_y,max_column_width);
	}
	void write_status(const std::string& text) {
		int font_face = cv::FONT_HERSHEY_PLAIN;
		cv::Scalar color(0,0,0);
		double scale = 1;
		int thickness = 1;
		int line_type = 8;
		int base_line;
		cv::Point p(0,display.rows);
		cv::putText(display,text,p,font_face,scale,color,thickness,line_type);
	}
	void draw(const std::string& status_text) {
		const int width = 1800;
		const int height = 100;
		if(!show_monitor) {
			display.create(height,width);
			cv::imshow("monitor",display);
			cv::createTrackbar("rank","monitor",&current_rank,cnvec.size());
			show_monitor = true;
		}
		set_to(display,cv::Vec3b(255,255,255));
		int current_x = 0;
		int current_y = 0;
		int max_column_width = -1;
		write_next_cell("rank",current_x,current_y,max_column_width);
		write_next_cell("remaining hypotheses",current_x,current_y,max_column_width);
		write_next_cell("n evaluated",current_x,current_y,max_column_width);
		write_next_cell("n hypotheses/sec",current_x,current_y,max_column_width);
		current_x += max_column_width+2;
		current_y = 0;
		for(int j=current_rank;j<cnvec.size();j++) {
			int max_column_width = -1;
			write_next_cell(j,current_x,current_y,max_column_width);
			write_next_cell(get_n_remaining(j),current_x,current_y,max_column_width);
			write_next_cell(get_n_evaluated(j),current_x,current_y,max_column_width);
			write_next_cell(get_n_hypotheses_per_second(j),current_x,current_y,max_column_width);
			current_x += max_column_width+2;
			current_y = 0;
			if(current_x>width) {
				break;
			}
		}
		write_status(status_text);
		cv::imshow("monitor",display);
	}
};

void check_n_hypotheses_message(CPUs& cpus, float lower_bound) {
	MPI_Status status;
	int message_arrived;
	MPI_Iprobe(MPI_ANY_SOURCE,N_HYPOTHESES_TAG,MPI_COMM_WORLD,&message_arrived,&status);
	if(message_arrived) {
		n_hypotheses_message nhm;
		MPI_Recv(&nhm,sizeof(n_hypotheses_message)/sizeof(int),MPI_INT,status.MPI_SOURCE,N_HYPOTHESES_TAG,
				MPI_COMM_WORLD,&status);
		MPI_Send(&lower_bound,1,MPI_FLOAT,status.MPI_SOURCE,N_HYPOTHESES_TAG,MPI_COMM_WORLD);

		cpus.set_n_hypotheses_per_second(status.MPI_SOURCE,nhm.n_hypotheses_per_second);
		cpus.update_n_hypotheses(status.MPI_SOURCE,nhm.n_remaining,nhm.n_evaluated);
		if(nhm.delegate_from!=-1) {
			cpus.set_is_delegating(status.MPI_SOURCE,false);
			cpus.set_is_delegating(nhm.delegate_from,false);
		}
	}
}
bool check_leaf_message(localization::set& localizations, const object& obj) {
	MPI_Status status;
	int message_arrived;
	MPI_Iprobe(MPI_ANY_SOURCE,LEAF_HYPOTHESIS_TAG,MPI_COMM_WORLD,&message_arrived,&status);
	if(message_arrived) {
		leaf_hypothesis_message lhm;
		MPI_Recv(&lhm,sizeof(leaf_hypothesis_message)/sizeof(float),MPI_FLOAT,MPI_ANY_SOURCE,LEAF_HYPOTHESIS_TAG,
				MPI_COMM_WORLD,&status);
		localization loc(&obj.views[lhm.view_index],lhm.p,lhm.log_probability);
		localizations.insert(loc);
		return true;
	} else {
		return false;
	}
}
template <class search_space_constraint>
void coordinate_detection(const object& obj, const image& im, int n_best_detections, float lower_bound, const cv::Mat_<cv::Vec3b>& picture,
		const search_space_constraint& constraint,
		bool visualize_search, int n_chunks, const std::string& CPU_productivity_log_file,
		//output parameter:
		localization::set& localizations) {
	int n_cpus;
	MPI_Comm_size(MPI_COMM_WORLD,&n_cpus);

	CPUs cpus(n_cpus,obj.views.size());

	hypothesis::priority_queue q_initial;
	for(int j=0;j<obj.views.size();j++) {
		const view* v = &obj.views[j];
		q_initial.push(hypothesis(v,im,entire_image(v,&im)));
	}

	while(q_initial.size()<n_chunks) {
		hypothesis h = q_initial.top();

		q_initial.pop();
		if(h.is_leaf() && h.get_bound()>=lower_bound) {
			throw std::runtime_error("leaf message found earlier than expected");
		} else {
			h.branch(q_initial,im,lower_bound,constraint);
		}
		if(q_initial.size()==0) {
			throw std::runtime_error("queue empty before search starts");
		}
	}
	std::vector<hypothesis> chunk_hypothesis_roots;
	while(q_initial.size()>0) {
		chunk_hypothesis_roots.push_back(q_initial.top());
		q_initial.pop();
	}
	cv::Mat_<cv::Vec3b> modified_picture = picture.clone();

	for(int current_chunk=0;current_chunk<n_chunks;current_chunk++) {
		bool sent_chunk_hypothesis_root = false;
		int iterations_since_redraw = 0;
		while(true) {
			MPI_Status status;
			int message_arrived;
			MPI_Iprobe(MPI_ANY_SOURCE,MPI_ANY_TAG,MPI_COMM_WORLD,&message_arrived,&status);
			if(visualize_search && (!message_arrived || iterations_since_redraw>1000)) {
				std::ostringstream status_text;
				int n_too_full = 0;
				for(int j=1;j<n_cpus;j++) {
					if(cpus.get_n_remaining(j)>max_queue_size) {
						n_too_full++;
					}
				}
				int n_empty = 0;
				for(int j=1;j<n_cpus;j++) {
					if(cpus.get_n_remaining(j)==0) {
						n_empty++;
					}
				}
				status_text << "lower bound: " << lower_bound << ", " << localizations.size() << " localizations. ";
				status_text << n_too_full << " full, " << n_empty << " empty, " << cpus.get_n_delegating() << " delegating.";
				if(localizations.size()>0) {
					status_text << " best localization: " << *localizations.begin();
				}
				cpus.draw(status_text.str());
				cv::waitKey(100); //allow the windows to redraw
				iterations_since_redraw=0;
			} else {
				iterations_since_redraw++;
			}

			localization::set::const_iterator best_before = localizations.begin();
			for(int j=0;j<1000;j++) {
				if(check_leaf_message(localizations,obj)) {
					if(n_best_detections>0 && localizations.size()>=n_best_detections) {
						localization::set::const_iterator j = localizations.begin();
						std::advance(j,n_best_detections-1);
						if(j->log_probability>lower_bound) {
							lower_bound = j->log_probability;
						}
					}
				} else {
					break;
				}
			}
			if(visualize_search && best_before!=localizations.begin()) {
				localization best_localization = *localizations.begin();
				modified_picture = picture.clone();
				im.draw(*best_localization.v,best_localization.location,
						//output parameter:
						modified_picture);
				cv::imshow("image",modified_picture);
			}
			check_n_hypotheses_message(cpus,lower_bound);

			if(!sent_chunk_hypothesis_root) {
				int delegate_to = cpus.next_delegation_to();
				if(delegate_to != -1) {
					int n_hypotheses_to_delegate = 0;
					for(int j=0;j<chunk_hypothesis_roots.size();j++) {
						if(j%n_chunks==current_chunk) {
							n_hypotheses_to_delegate++;
						}
					}
					MPI_Send(&n_hypotheses_to_delegate,1,MPI_INT,delegate_to,DELEGATE_HYPOTHESES_TAG,MPI_COMM_WORLD);
					delegate_hypothesis_message *dhm = new delegate_hypothesis_message[n_hypotheses_to_delegate];
					int current_hypothesis = 0;
					for(int j=0;j<chunk_hypothesis_roots.size();j++) {
						if(j%n_chunks==current_chunk) {
							dhm[current_hypothesis] = delegate_hypothesis_message(chunk_hypothesis_roots[j]);
							current_hypothesis++;
						}
					}

					MPI_Send(dhm,sizeof(delegate_hypothesis_message)/sizeof(float)*n_hypotheses_to_delegate,
							MPI_FLOAT,delegate_to,DELEGATE_HYPOTHESES_TAG,MPI_COMM_WORLD);
					delete[] dhm;
					cpus.set_is_delegating(0,true);
					cpus.set_is_delegating(delegate_to,true);
					sent_chunk_hypothesis_root = true;
				}
			} else {
				while(true) {
					int delegate_to = cpus.next_delegation_to();
					int delegate_from = cpus.next_delegation_from();
					if(delegate_to!=-1 && delegate_from != -1) {
						start_delegation_message sdm;
						sdm.delegate_to_rank = delegate_to;
						sdm.n_hypotheses_per_second = cpus.get_n_hypotheses_per_second(delegate_to);
						MPI_Send(&sdm,sizeof(start_delegation_message)/sizeof(float),MPI_FLOAT,delegate_from,START_DELEGATION_TAG,MPI_COMM_WORLD);
						cpus.set_is_delegating(delegate_from,true);
						cpus.set_is_delegating(delegate_to,true);
					} else {
						break;
					}
				}
			}
			if(cpus.get_n_delegating()==0 && cpus.any_first_messages_received() && cpus.get_n_remaining(cpus.rank_with_most_remaining_hypotheses())==0) {
				//then everything is finished
				//  DELEGATE_HYPOTHESIS_TAG from rank 0 means "done"
				int place_holder = 0;
				for(int j=1;j<n_cpus;j++) {
					MPI_Send(&place_holder,0,MPI_INT,j,FINISHED_TAG,MPI_COMM_WORLD);
				}
				if(CPU_productivity_log_file!="") {
					std::ofstream out(CPU_productivity_log_file.c_str());
					for(int j=1;j<n_cpus;j++) {
						out << cpus.get_n_evaluated(j) << " " << cpus.get_n_hypotheses_per_second(j) << " " << j << std::endl;
					}
				}
				break;
			}
		}
	}
}

template <class search_space_constraint>
void start_work(const object& obj, const image& im, int n_best_detections, float lower_bound, cv::Mat_<cv::Vec3b> picture,
		const search_space_constraint& constraint, int rank, int visualize_search, int n_chunks, const std::string& CPU_productivity_log_file,
		//output parameter:
		localization::set& localizations) {
	if(rank==0) {
		coordinate_detection(obj,im,n_best_detections,lower_bound,picture,constraint,visualize_search!=0,n_chunks,CPU_productivity_log_file,
				//output parameter:
				localizations);
	} else {
		detect(obj,im,constraint,lower_bound,rank);
		MPI_Finalize();
	}
}

} //namespace objrec
void distribute_file(const std::string& file_name, int rank,
		//output parameter:
		std::vector<char>& content) {
	int file_size;
	std::ifstream ifs;
	if(rank==0) {
		ifs.open(file_name.c_str(), std::ios::binary);
		ifs.seekg(0, std::ios::end);
		file_size = ifs.tellg();
		ifs.seekg(0, std::ios::beg);
	}
	MPI_Bcast(&file_size,1,MPI_INT,0,MPI_COMM_WORLD);
	char* file_bytes = new char[file_size];
	if(rank==0) {
		if (!ifs.read(file_bytes, file_size)) {
			throw std::runtime_error("Could not read file '"+file_name+"'");
		}
	}
	MPI_Bcast(file_bytes,file_size,MPI_CHAR,0,MPI_COMM_WORLD);
	content.insert(content.end(), file_bytes, &file_bytes[file_size]);
	delete[] file_bytes;
}



int main(int argc, char** argv) {
	MPI_Init(&argc,&argv);
	int rank;
	MPI_Comm_rank(MPI_COMM_WORLD,&rank);
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
	ADD_STRING_PARAM(depth_file,"", "The corresponding depth file to load.  If depth_file is not specified, it is assumed to have the same base name as the image, with the extension .yml.");
	ADD_STRING_PARAM(edges_file,"","The corresponding file containing precomputed edges. If edges_file is not specified, it is assumed to have the same base name as the image, with the extension .edges.");
	ADD_INT_PARAM(ignore_edges_file,0,"If non-zero, the edges file (if it exists) is ignored.");
	ADD_STRING_PARAM(bounding_box_file,"","The corresponding bounding box file to load.  The bounding box file defines a rectangular region of pixels in the image.  Only detections whose bounding boxes have sufficient overlap (as defined by the bounding_box_overlap parameter) with this bounding box are considered. If bounding_box_file is not specified, it is assumed to have the same base name as the image, with the extension .bbox. If it does not exist, no bounding box constraint is used.  If bounding box and a camera info files are both available, only the bounding box constraint will be used.")
	ADD_PARAM(bounding_box_overlap,0.5,"The minimum overlap between the ground truth bounding box and the detected bounding box required to consider the detection.  This is calculated using by dividing the area of the intersection by the area of the union of the two bounding boxes.");
	ADD_INT_PARAM(ignore_bounding_box,0,"If non-zero, the bounding box file (if it exists) is ignored.");
	ADD_STRING_PARAM(scene_file,"","The corresponding scene file to load.  If scene_file is not specified, it is assumed to have the same base name as the image, with the extension \".scene\".");
	ADD_INT_PARAM(ignore_scene_file,0,"If non-zero, the scene file (if it exists) is ignored.");
	float inf = std::numeric_limits<float>::infinity();
	ADD_PARAM(lower_bound,-inf,"The the lowest log probability value to search before terminating.");
	ADD_INT_PARAM(n_best_detections,0,"If positive, the search will terminate immediately after finding this number of the highest probability detections in the image.");
//	ADD_INT_PARAM(non_maximum_suppression,1,"If non-zero, detections that overlap more than nms_overlap_amount will be greedily ignored.");
	ADD_PARAM(camera_pitch_angle_tolerance,12,"If using a table constraint, detections up to +/- angle_tolerance degrees from upright are within the search space.");
	ADD_PARAM(camera_height_tolerance,0.0135,"If using a table constraint, detections up to +/- camera_height_tolerance meters from the table are within the search space.");
	ADD_PARAM(nms_overlap_percent,0.5,"The amount of overlap required to reject a detection by non maximum suppression.  The overlap is calculuated as with the bounding_box_overlap parameter.");
	ADD_STRING_PARAM(output_file,"","If specified, results will be written to this file.  Otherwise, they will be sent to stdout.");
	ADD_INT_PARAM(verbose,0,"If non-zero, status information will be printed to stderr.");
	ADD_INT_PARAM(visualize_search,0,"If non-zero and a camera info file is provided, the progress through search regions of the image will be displayed.");
	ADD_INT_PARAM(n_chunks,1,"If the search space is too big (for example, if there are no scene file or bounding box constraints), the search may fill all of the available memory on all CPUs. So it may be necessary to break the search up into more than one 'chunks' that will be searched sequentially, completing the search through one chunk before the next. If the original search is like a breadth-first search, then dividing it into chunks makes it more like a depth-first search.");
	ADD_STRING_PARAM(CPU_productivity_log_file,"","For checking performance of MPI nodes. Write a list of the number of search nodes evaluated by each CPU to this file.");

	CHECK_PARAMS();

	if(!object_file_is_passed || !image_file_is_passed) {
		if(rank==0) {
			std::cerr << "A required parameter was not passed." << std::endl;
			std::cerr << std::endl;
			std::cerr << help << std::endl;
		}
		MPI_Finalize();
		return 0;
	}

	if(rank==0 && lower_bound==-inf && n_best_detections<=0) {
		std::cerr << "WARNING! Searching for all detections with a lower bound of -inf will yield a very large output." << std::endl;
		std::cerr << "  use the '-n_best_detections' or the '-lower_bound' parameter." << std::endl;
	}

	if(objrec::is_finished()) { MPI_Finalize(); return 0; }
	if(!depth_file_is_passed) depth_file = objrec::corresponding_depth_file(image_file);
	if(!edges_file_is_passed) edges_file = objrec::corresponding_edges_file(image_file);
	if(!scene_file_is_passed) scene_file = objrec::corresponding_scene_file(image_file);
	if(!bounding_box_file_is_passed) bounding_box_file = objrec::corresponding_bounding_box_file(image_file);
	if(objrec::is_finished()) { MPI_Finalize(); return 0; }

	long long start, end;
	objrec::object obj;
	{
		if(verbose!=0 && rank==0) std::cerr << "Loading object file '" << object_file << "'... " << std::flush;
		start = objrec::current_time();
//		std::vector<char> contents;
//		distribute_file(object_file,rank,contents);
//		std::istringstream in(std::string(contents.begin(),contents.end()));
		std::ifstream in(object_file.c_str());
		in >> obj;
//		MPI_Barrier(MPI_COMM_WORLD);
		end = objrec::current_time();
		if(verbose!=0 && rank==0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}
	if(objrec::is_finished()) { MPI_Finalize(); return 0; }

	if(verbose!=0 && rank==0) std::cerr << "Loading picture file '" << image_file << "'... " << std::flush;
	start = objrec::current_time();
	cv::Mat_<cv::Vec3b> picture;
//	if(rank==0) {
		picture = cv::imread(image_file);
//	}
//	distribute_mat(picture,rank);
//	MPI_Barrier(MPI_COMM_WORLD);
	end = objrec::current_time();
	if(verbose!=0 && rank==0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	if(objrec::is_finished()) { MPI_Finalize(); return 0; }

	if(verbose!=0 && rank==0) std::cerr << "Loading depth file '" << depth_file << "'... " << std::flush;
	start = objrec::current_time();
	cv::Mat_<float> depth;
//	if(rank==0) {
		depth = objrec::read_depth(depth_file);
//	}
//	distribute_mat(depth,rank);
//	MPI_Barrier(MPI_COMM_WORLD);
	end = objrec::current_time();
	if(verbose!=0 && rank==0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	if(objrec::is_finished()) { MPI_Finalize(); return 0; }

	objrec::scene_info info;
	int load_scene_file;
//	if(rank==0) {
		load_scene_file = ignore_scene_file==0 && scene_file!="";
//	}
//	MPI_Bcast(&load_scene_file,1,MPI_INT,0,MPI_COMM_WORLD);
	if(load_scene_file) {
		if(verbose!=0 && rank==0) std::cerr << "Loading scene file '" << scene_file << "'... " << std::flush;
		start = objrec::current_time();
//		std::vector<char> contents;
//		distribute_file(scene_file,rank,contents);
//		std::istringstream in(std::string(contents.begin(),contents.end()));
		std::ifstream in(scene_file.c_str());
		objrec::read_scene_info(in,info);
//		MPI_Barrier(MPI_COMM_WORLD);
		end = objrec::current_time();
		if(verbose!=0 && rank==0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}
	if(objrec::is_finished()) { MPI_Finalize(); return 0; }

	std::vector<cv::Mat_<float> > precomputed_edges;
	int load_precomputed_edges;
//	if(rank==0) {
		load_precomputed_edges = (!ignore_edges_file && edges_file!="");
//	}
//	MPI_Bcast(&load_precomputed_edges,1,MPI_INT,0,MPI_COMM_WORLD);
	if(load_precomputed_edges) {
		if(verbose!=0 && rank==0) std::cerr << "Loading edges file '" << edges_file << "'... " << std::flush;
		start = objrec::current_time();
//		std::vector<char> contents;
//		distribute_file(edges_file,rank,contents);
//		std::istringstream in(std::string(contents.begin(),contents.end()));
		std::ifstream in(edges_file.c_str());
		objrec::read_edges(in,precomputed_edges);
//		MPI_Barrier(MPI_COMM_WORLD);
		end = objrec::current_time();
		if(verbose!=0 && rank==0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}
	if(objrec::is_finished()) { MPI_Finalize(); return 0; }

	objrec::bounding_box bbox;
	int load_bounding_box;
//	if(rank==0) {
		load_bounding_box = ignore_bounding_box==0 && bounding_box_file!="";
//	}
//	MPI_Bcast(&load_bounding_box,1,MPI_INT,0,MPI_COMM_WORLD);
	if(load_bounding_box) {
		if(verbose!=0 && rank==0) std::cerr << "Loading bounding box file '" << bounding_box_file << "'... " << std::flush;
		start = objrec::current_time();
//		std::vector<char> contents;
//		distribute_file(bounding_box_file,rank,contents);
//		std::istringstream in(std::string(contents.begin(),contents.end()));
		std::ifstream in(bounding_box_file.c_str());
		in.exceptions(std::istringstream::failbit);
		objrec::read_bounding_box(in,bbox);
//		MPI_Barrier(MPI_COMM_WORLD);
		end = objrec::current_time();
		if(verbose!=0 && rank==0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}
	if(objrec::is_finished()) { MPI_Finalize(); return 0; }

	if(verbose!=0 && rank==0) std::cerr << "Preprocessing... " << std::flush;
	start = objrec::current_time();
	objrec::image im(obj.library,picture,depth,precomputed_edges);
//	MPI_Barrier(MPI_COMM_WORLD);
	end = objrec::current_time();
	if(verbose!=0 && rank==0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	if(objrec::is_finished()) { MPI_Finalize(); return 0; }

	objrec::localization::set localizations;

	if(rank==0) {
		if(visualize_search!=0) {
			cv::Mat_<cv::Vec3b> edges_bgr;
			if(load_precomputed_edges) {
				edges_bgr = objrec::edges_bgr(precomputed_edges,obj.library.get_edges());
			} else {
				edges_bgr = objrec::edges_bgr(picture, depth, obj.library.get_edges());
			}
			cv::imshow("edges",edges_bgr);

			cv::imshow("depth",objrec::mat_bgr(depth));
			if(load_bounding_box) {
				cv::line(picture,cv::Point(bbox.min_x,bbox.min_y),cv::Point(bbox.max_x,bbox.min_y),cv::Scalar(255,255,255),1);
				cv::line(picture,cv::Point(bbox.max_x,bbox.min_y),cv::Point(bbox.max_x,bbox.max_y),cv::Scalar(255,255,255),1);
				cv::line(picture,cv::Point(bbox.max_x,bbox.max_y),cv::Point(bbox.min_x,bbox.max_y),cv::Scalar(255,255,255),1);
				cv::line(picture,cv::Point(bbox.min_x,bbox.max_y),cv::Point(bbox.min_x,bbox.min_y),cv::Scalar(255,255,255),1);
			}
			cv::imshow("image",picture);
			cv::waitKey(500);
		}
		if(verbose!=0) std::cerr << "Detecting... " << std::flush;
	}
	start = objrec::current_time();

	if(load_scene_file) {
		objrec::table_constraint tconstraint(info,obj.object_center_height_above_table,camera_pitch_angle_tolerance,camera_height_tolerance);
		if(load_bounding_box) {
			objrec::bounding_box_constraint<objrec::table_constraint> bbconstraint(bbox,tconstraint);
			objrec::start_work(obj,im,n_best_detections,lower_bound,picture,bbconstraint,rank,visualize_search,n_chunks,CPU_productivity_log_file,
					//output parameter:
					localizations);
		} else {
			objrec::start_work(obj,im,n_best_detections,lower_bound,picture,tconstraint,rank,visualize_search,n_chunks,CPU_productivity_log_file,
					//output parameter:
					localizations);
		}
	} else {
		objrec::no_constraint noconstraint;
		if(load_bounding_box) {
			objrec::bounding_box_constraint<objrec::no_constraint> bbconstraint(bbox,noconstraint);
			objrec::start_work(obj,im,n_best_detections,lower_bound,picture,bbconstraint,rank,visualize_search,n_chunks,CPU_productivity_log_file,
					//output parameter:
					localizations);
		} else {
			objrec::start_work(obj,im,n_best_detections,lower_bound,picture,noconstraint,rank,visualize_search,n_chunks,CPU_productivity_log_file,
					//output parameter:
					localizations);
		}
	}
	end = objrec::current_time();
	if(rank==0 && verbose!=0) std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	if(rank>0) {
		return 0;
	}

	//non-maximum suppression of localizations
	std::vector<objrec::localization> nms_localizations;
	for(objrec::localization::set::const_iterator j=localizations.begin();j!=localizations.end();j++) {
		if(!objrec::is_non_maximum(nms_localizations,*j,nms_overlap_percent)) {
			nms_localizations.push_back(*j);
		}
	}

	int n_detections_to_print;
	if(n_best_detections<=0) {
		n_detections_to_print = nms_localizations.size();
	} else {
		n_detections_to_print = std::min(n_best_detections,(int)nms_localizations.size());
	}
	if(output_file_is_passed) {
		std::ofstream out(output_file.c_str());
		if(!out) {
			throw std::runtime_error("Could not open output file '" + output_file + "'");
		}
		for(int j=0;j<n_detections_to_print;j++) {
			out << nms_localizations[j] << " " << object_file << " " << image_file << std::endl;
//			out << im.max_log_depth_parts_probability(*loc->v,loc->location) << " from depth parts" << std::endl;
//			out << im.max_log_visual_parts_probability(*loc->v,loc->location) << " from visual parts" << std::endl;
		}
	} else {
		for(int j=0;j<n_detections_to_print;j++) {
			std::cout << nms_localizations[j] << " " << object_file << " " << image_file << std::endl;
//			std::cout << im.max_log_depth_parts_probability(*loc->v,loc->location) << " from depth parts" << std::endl;
//			std::cout << im.max_log_visual_parts_probability(*loc->v,loc->location) << " from visual parts" << std::endl;
		}
	}

	MPI_Finalize();
	return 0;


//	if(view_detections!=0) {
//		cv::imshow("picture",picture);
//
//		if(n_best_detections==0) {
//			if(verbose!=0) std::cerr << localizations.size() << " localizations above the log probability: " << lower_bound << "." << std::endl;
//			if(localizations.size()==0) {
//				cv::waitKey(0);
//				MPI_Finalize();
//				return 0;
//			}
//			std::cerr << "Press up/down to see different localizations." << std::endl;
//			int current_loc = 0;
////			std::cerr << localizations[current_loc] << std::endl;
//			bool quit = false;
//			while(!quit) {
//				cv::Mat_<cv::Vec3b> modified_picture = picture.clone();
////				cv::Mat_<cv::Vec3b> modified_edges_bgr = edges_bgr.clone();
////				cv::Mat_<cv::Vec3b> modified_textons_bgr = textons_bgr.clone();
////				cv::Mat_<cv::Vec3b> modified_depth_bgr = depth_bgr.clone();
////				objrec::draw(localizations[current_loc],im,
////						modified_picture,modified_edges_bgr,modified_textons_bgr,modified_depth_bgr);
////				cv::imshow("edges",modified_edges_bgr);
////				cv::imshow("depth",modified_depth_bgr);
////				cv::imshow("textons",modified_textons_bgr);
////				cv::imshow("picture",modified_picture);
////				localizations[current_loc].v->draw(localizations[current_loc].location,modified_picture);
//
//				int key = cv::waitKey(0);
//				switch(key) {
//				case 65364: //down arrow
//				case 1113940: //down arrow with num lock
//					if(current_loc<(int)localizations.size()-1) {
//						current_loc++;
////						std::cerr << localizations[current_loc] << std::endl;
//					}
//					break;
//				case 65362: //up arrow
//				case 1113938: //up arrow with numlock
//					if(current_loc>0) {
//						current_loc--;
////						std::cerr << localizations[current_loc] << std::endl;
//					}
//					break;
//				case 113: //q
//				case 1048689: //q with numlock
//				case 27: //esc
//				case 1048603: //esc with numlock
//					quit = true;
//					break;
//				default:
//					std::cerr << "Unrecognized key pressed: " << key << std::endl;
//					break;
//				}
//			}
//		} else {
//
//			if(localizations.size()==0) {
//				std::cerr << "No localizations above the lower bound were found. Maybe try decreasing the lower bound." << std::endl;
//			} else {
//				std::cerr << "Top detection:" << std::endl;
////				std::cerr << localizations[0] << std::endl;
////				localizations[0].v->draw(localizations[0].location,picture);
////				objrec::draw(localizations[0],im,picture,edges_bgr,textons_bgr,depth_bgr);
//				cv::imshow("picture",picture);
//				cv::waitKey(0);
//			}
//		}
//	}
}

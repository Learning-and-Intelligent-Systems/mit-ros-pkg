/*
 * distribute.cpp
 *
 *  Created on: Jul 10, 2014
 */
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <mpi.h>
#include <iomanip>
#include <sys/time.h>
#include <cmath>
#include <vector>
#include <mpi.h>
#include <ifaddrs.h>
#include <arpa/inet.h>

std::string get_IP_address() {
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa ->ifa_addr->sa_family==AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
//            printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
            if(std::string(ifa->ifa_name)=="eth0") {
            	std::string IP_address = addressBuffer;
                if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
            	return IP_address;
            }
//        } else if (ifa->ifa_addr->sa_family==AF_INET6) { // check it is IP6
//            // is a valid IP6 Address
//            tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
//            char addressBuffer[INET6_ADDRSTRLEN];
//            inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
//            printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer);
        }
    }
    if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
    return "";
}

void MPI_send_IP_address() {
	std::string IP_address = get_IP_address();
	int IP_address_length = IP_address.size();
	MPI_Send(&IP_address_length,1,MPI_INT,0,0,MPI_COMM_WORLD);
	char* IP = new char[IP_address_length+1];
	strcpy(IP,IP_address.c_str());
	MPI_Send(IP,IP_address.size(),MPI_CHAR,0,0,MPI_COMM_WORLD);
	delete[] IP;
}

void MPI_recv_IP_addresses(int n_cpus, std::vector<std::string>& IP_addresses) {
	IP_addresses.resize(n_cpus);
	for(int j=1;j<n_cpus;j++) {
		int IP_address_length;
		MPI_Status status;
		MPI_Recv(&IP_address_length,1,MPI_INT,MPI_ANY_SOURCE,0,MPI_COMM_WORLD,&status);
		char* IP = new char[IP_address_length+1];
		MPI_Recv(IP,IP_address_length,MPI_CHAR,status.MPI_SOURCE,0,MPI_COMM_WORLD,&status);
		IP[IP_address_length] = (char)0;
		IP_addresses[status.MPI_SOURCE] = IP;
	}
}

struct assignment_message {
	int len, id;
};
struct results_message {
	int id, exit_status, output_size;
};

//assign command
//receive results
//receive error
//kill job
// pipe (to create the pipe)
// fork (to create the subprocess)
// dup2 (to force the subprocess to use the pipe as its standard input or output channel), and
// exec (to execute the new program)

void send_result(const std::string& output, int id, int exit_status) {
	results_message results;
	results.id = id;
	results.exit_status = exit_status;
	results.output_size = output.size();
	MPI_Send(&results,sizeof(results)/sizeof(int),MPI_INT,0,0,MPI_COMM_WORLD);
	char* output_buffer = new char[output.size()+1];
	strcpy(output_buffer,output.c_str());
	MPI_Send(output_buffer,results.output_size,MPI_CHAR,0,0,MPI_COMM_WORLD);
	delete[] output_buffer;
}

//int popen2(const std::string& cmd, FILE** fp, pid_t& pid) {
//	const int READ = 0, WRITE = 1;
//	int pipe_stdout[2];
//	if(pipe(pipe_stdout)!=0) {
//		return -1;
//	}
//	pid = fork();
//	if(pid<0) {
//		return -1;
//	} else if(pid == 0) { //child process
//		close(pipe_stdout[READ]);
//		dup2(pipe_stdout[WRITE],WRITE);
//		execl("/bin/sh", "sh", "-c", cmd.c_str(), NULL);
//		perror("execl");
//		exit(1);
//	} else { //parent process
//		*fp = fdopen(pipe_stdout[READ],"r");
//		return 0;
//	}
//}

void worker() {
	MPI_send_IP_address();
	while(true) {
		MPI_Status status;
		assignment_message assignment;
		MPI_Recv(&assignment,sizeof(assignment_message)/sizeof(int),MPI_INT,0,0,MPI_COMM_WORLD,&status);
		if(assignment.len==-1) {
			return;
		} else {
			char* cmd = new char[assignment.len+1];
			MPI_Recv(cmd,assignment.len,MPI_CHAR,0,0,MPI_COMM_WORLD,&status);
			cmd[assignment.len] = (char)0;


			FILE* fp;
			fp = popen(cmd,"r");
//			pid_t pid;
//			if(popen2(cmd, &fp, pid)!=0) {
//				send_result("could not fork process on "+IP_address,assignment.id,-1);
//				continue;
//			}

			std::string output;
			char buffer[1024];
			while (fgets(buffer, sizeof(buffer), fp) != NULL) {
				output += buffer;
			}
			send_result(output,assignment.id,pclose(fp));
		}
	}
}

long long assign_job(const std::vector<std::string>& commands, const std::vector<std::string>& outfiles, int cpu,
		int& next_job_to_start, int& n_previously_completed) {
	while(next_job_to_start<commands.size()) {
		if(std::ifstream(outfiles[next_job_to_start].c_str()).good()) {
			n_previously_completed++;
			next_job_to_start++;
		} else {
			assignment_message assignment;
			assignment.len = commands[next_job_to_start].size();
			assignment.id = next_job_to_start;
			MPI_Send(&assignment,sizeof(assignment_message)/sizeof(int),MPI_INT,cpu,0,MPI_COMM_WORLD);
			char* cmd = new char[assignment.len+1];
			strcpy(cmd,commands[next_job_to_start].c_str());
			MPI_Send(cmd,commands[next_job_to_start].size(),MPI_CHAR,cpu,0,MPI_COMM_WORLD);
		    struct timeval tv;
		    gettimeofday(&tv, 0);
		    long long start = ((long long)tv.tv_sec)*1000000 + tv.tv_usec;
			delete[] cmd;
			next_job_to_start++;
			return start;
		}
	}
	return 0;
}

inline std::string time_string(double t) {
	std::ostringstream o;
	double minutes = floor(t/60.0);
	double seconds = (int)floor(t)%60;
	double milliseconds = round((t-floor(t))*1000.);
	o << std::setw(2) << std::setfill('0') << minutes << ":";
	o << std::setw(2) << std::setfill('0') << seconds << ".";
	o << std::setw(3) << std::setfill('0') << milliseconds;
	return o.str();
}
void progress(long long start, int j, int n) {
        struct timeval tv;
        gettimeofday(&tv, 0);
        long long now = ((long long)tv.tv_sec)*1000000 + tv.tv_usec;
        double elapsed = (double)(now-start)/1000000.0;
        if(j!=0) {
        	double remaining = elapsed/(double)j*(n-j);

        	std::cerr << "completed " << j << " of " << n
        			<<" jobs. (" << round(100.0f*(float)(j)/(float)n) << "%) ";
        	std::cerr << time_string(elapsed) << " elapsed, " << time_string(remaining) << " left.    \r" << std::flush;
        } else {
        	std::cerr << "completed 0 of " << n << " jobs.\r" << std::flush;
        }
}

void write_job_status_file(const std::string& job_status_file, const std::vector<std::string>& IP_addresses, const std::vector<long long>& job_start_times) {
	if(job_status_file=="") {
		return;
	}
	std::ofstream out(job_status_file.c_str());
    struct timeval tv;
    gettimeofday(&tv, 0);
    long long current_time = ((long long)tv.tv_sec)*1000000 + tv.tv_usec;
	for(int j=1;j<IP_addresses.size();j++) {
		if(job_start_times[j]!=0) {
			double elapsed_seconds = (double)(current_time - job_start_times[j])/1000000.0;
			out << elapsed_seconds << " " << IP_addresses[j] << std::endl;
		}
	}
}

void distribute(const std::vector<std::string>& commands, const std::vector<std::string>& outfiles, const std::string& job_status_file) {
	int n_cpus;
	MPI_Comm_size(MPI_COMM_WORLD,&n_cpus);

	std::vector<std::string> IP_addresses;
	MPI_recv_IP_addresses(n_cpus,IP_addresses);
	std::vector<long long> job_start_times(n_cpus,0);

    struct timeval tv;
    gettimeofday(&tv, 0);
    long long start = ((long long)tv.tv_sec)*1000000 + tv.tv_usec;
	int next_job_to_start = 0;
	int n_previously_completed = 0;
	for(int j=1;j<n_cpus;j++) {
		job_start_times[j] = assign_job(commands,outfiles,j,next_job_to_start,n_previously_completed);
		write_job_status_file(job_status_file,IP_addresses,job_start_times);
	}
	if(n_previously_completed>0) {
		std::cout << n_previously_completed << " jobs previously completed." << std::endl;
	}
	int n_done = 0;
	progress(start,n_done,commands.size()-n_previously_completed);
	while(n_done+n_previously_completed<commands.size()) {
		for(int j=0;j<100000;j++) {
			MPI_Status status;
			int message_arrived;
			MPI_Iprobe(MPI_ANY_SOURCE,0,MPI_COMM_WORLD,&message_arrived,&status);
			if(message_arrived) {
				results_message results;
				MPI_Recv(&results,sizeof(results_message)/sizeof(int),MPI_INT,MPI_ANY_SOURCE,0,MPI_COMM_WORLD,&status);
				char* output_buffer = new char[results.output_size+1];
				MPI_Recv(output_buffer,results.output_size,MPI_CHAR,status.MPI_SOURCE,0,MPI_COMM_WORLD,&status);
				output_buffer[results.output_size] = (char)0;
				if(results.exit_status==0) {
					if(outfiles[results.id]!="") {
						std::ofstream out(outfiles[results.id].c_str());
						out << output_buffer;
					}
					n_done++;
					job_start_times[status.MPI_SOURCE] = assign_job(commands,outfiles,status.MPI_SOURCE,next_job_to_start,n_previously_completed);
				}
				delete[] output_buffer;
				break;
			}
		}
		progress(start,n_done,commands.size()-n_previously_completed);
		write_job_status_file(job_status_file,IP_addresses,job_start_times);
	}
	std::cerr << std::endl;
	for(int j=1;j<n_cpus;j++) {
		assignment_message assignment;
		assignment.len = -1;
		assignment.id = next_job_to_start;
		MPI_Send(&assignment,2,MPI_INT,j,0,MPI_COMM_WORLD);
	}
}

std::string ltrim(const std::string& s) {
	size_t startpos = s.find_first_not_of(" \t");
	if(startpos!=std::string::npos) {
		return s.substr(startpos);
	} else {
		return s;
	}
}

int main(int argc, char** argv) {
	MPI_Init(&argc,&argv);
	int rank;
	MPI_Comm_rank(MPI_COMM_WORLD,&rank);
	int n_cpus;
	MPI_Comm_size(MPI_COMM_WORLD,&n_cpus);

	if(rank==0) {
		if(argc<2 || n_cpus <= 1) {
			std::cout << "usage:" << std::endl;
			std::cout << "mpirun " << argv[0] << " jobs [job_status_file]" << std::endl;
			std::cout << std::endl;
			std::cout << "the 'jobs' file is a text file with one line per job.  A line is a command, optionally followed by a '>' and a filename on the local machine in which to store the stdout from the command." << std::endl;
			std::cout << "the optional 'job_status_file' gives the time since the current job was started on each IP address." << std::endl;
			return -1;
		}

		std::vector<std::string> commands;
		std::vector<std::string> out_files;
		{
			std::ifstream in(argv[1]);
			while(in.good()) {
				std::string line;
				std::getline(in,line);
				line = ltrim(line);
				if(line=="") {
					break;
				}
				size_t gt = line.find_last_of('>');
				commands.push_back(line.substr(0,gt));
				if(gt!=std::string::npos) {
					std::string out_file = ltrim(line.substr(gt+1));
					out_files.push_back(out_file);
				} else {
					out_files.push_back("");
				}
			}
		}
		std::string job_status_file;
		if(argc>2) {
			job_status_file = argv[2];
		} else {
			job_status_file = "";
		}
		distribute(commands,out_files,job_status_file);
	} else {
		worker();
	}
	MPI_Finalize();
	return 0;
}



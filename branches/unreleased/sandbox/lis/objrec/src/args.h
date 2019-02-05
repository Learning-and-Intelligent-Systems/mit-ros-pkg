/*
 * args.h
 *
 *  Created on: Jan 19, 2011
 */

#ifndef ARGS_H_
#define ARGS_H_
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <fstream>
#include <algorithm>

/** \brief Macros to facilitate parsing command-line parameters.
 *
 * To use these parameters you need the following variables defined:
 *
 * - int argc; //from the arguments to main()
 * - char **argv; //from the arguments to main()
 * - std::string help; //the help string is appended to after each parameter macro is called
 * - std::ostringstream param_vals; //the parameter values are sent to this stream
 * - int num_passed=0; //counts the number of arguments passed
 *
 * a new variable with name <param> will be defined, as well as a
 * bool with name <param>_is_passed which indicates whether the
 * parameter was passed.
 *
 */

#define INIT_PARAMS() \
std::vector<bool> argument_recognized(argc,false); \
int num_passed = 0; \
std::string help; \
std::ostringstream param_vals;


#define LEFT_BUFFER 36
#define ADD_PARAM(param,default_val,description_str) \
double param = default_val; \
bool param ## _is_passed = false;\
{ \
  std::string param_string(#param); \
  if(LEFT_BUFFER<param_string.length()) {\
	  std::ostringstream err; \
	  err << "Please increase LEFT_BUFFER in args.h to at least " << param_string.length() << " for parameter " << param_string << "."; \
	  throw std::runtime_error(err.str());\
  } \
  help += std::string("-")+param_string + std::string(LEFT_BUFFER-param_string.length(),' ') \
          + description_str + " (default: double " + #default_val + ")\n"; \
  for(int argj=1; argj<argc; argj++) {\
    if(argv[argj][0]=='-') {\
	  if(std::string(&(argv[argj][1]))==param_string) {\
		argument_recognized[argj] = true;\
		argj++;\
		if(argj>=argc) {throw std::runtime_error("no value for parameter "+param_string);}\
		std::istringstream arg(argv[argj]); \
		arg >> param; \
		param_vals << #param << std::string(LEFT_BUFFER-param_string.length(),' ') << "= " << param << std::endl;\
		param ## _is_passed = true;\
		num_passed++;\
		break;\
	  }\
	  else argj++;\
	}\
  }\
}

#define ADD_INT_PARAM(param,default_val,description_str) \
int param = default_val; \
bool param ## _is_passed = false;\
{ \
  std::string param_string(#param); \
  if(LEFT_BUFFER<param_string.length()) {\
	  std::ostringstream err; \
	  err << "Please increase LEFT_BUFFER in args.h to at least " << param_string.length() << " for parameter " << param_string << "."; \
	  throw std::runtime_error(err.str());\
  } \
  help += std::string("-")+param_string + std::string(LEFT_BUFFER-param_string.length(),' ') \
          + description_str + " (default: int " + #default_val + ")\n"; \
  for(int argj=1; argj<argc; argj++) {\
    if(argv[argj][0]=='-') {\
	  if(std::string(&(argv[argj][1]))==param_string) {\
		argument_recognized[argj] = true;\
		argj++;\
		if(argj>=argc) {throw std::runtime_error("no value for parameter "+param_string);}\
		param = (int)atoi(argv[argj]);\
		param_vals << #param << std::string(LEFT_BUFFER-param_string.length(),' ') << "= " << param << std::endl;\
		param ## _is_passed = true;\
		num_passed++;\
		break;\
	  }\
	  else argj++;\
	}\
  }\
}

#define ADD_STRING_PARAM(param,default_val,description_str) \
bool param ## _is_passed = false;\
std::string param = default_val; \
{\
  std::string param_string(#param); \
  if(LEFT_BUFFER<param_string.length()) {\
	  std::ostringstream err; \
	  err << "Please increase LEFT_BUFFER in args.h to at least " << param_string.length() << " for parameter " << param_string << "."; \
	  throw std::runtime_error(err.str());\
  } \
  help += std::string("-")+param_string + std::string(LEFT_BUFFER-param_string.length(),' ') \
          + description_str + " (default: string " + #default_val + ")\n"; \
  for(int argj=1; argj<argc; argj++) {\
    if(argv[argj][0]=='-') {\
	  if(std::string(&(argv[argj][1]))==param_string) {\
		argument_recognized[argj] = true;\
		argj++;\
		if(argj>=argc) {throw std::runtime_error("no value for parameter "+param_string);}\
		param = std::string(argv[argj]);\
		param_vals << #param << std::string(LEFT_BUFFER-param_string.length(),' ') << "= " << param << std::endl;\
		param ## _is_passed = true;\
		num_passed++;\
		break;\
	  }\
	  else argj++;\
	}\
  }\
}

#define CHECK_PARAMS() \
for(int argj=1; argj<argc; argj+=2) {\
	if(!argument_recognized[argj]) {\
		throw std::runtime_error("Unrecognized argument '"+std::string(argv[argj])+"'.");\
	}\
}

//TODO improve this to parse '' and "" characters, to allow for spaces, etc.
inline void parse_command_line(const std::string & line,int & argc,char**& argv) {
	char* l = new char[line.size()+1];
	memcpy(l,line.c_str(),line.size());
	l[line.size()] = 0;
	char* tok = strtok(l," ");
	std::vector<std::string> args;
	while(tok!=0) {
		args.push_back(tok);
		tok = strtok(0," ");
	}
	delete[] l;

	argc = args.size();
	argv = new char*[args.size()];
	for(unsigned int j=0;j<args.size();j++) {
		argv[j] = new char[args[j].size()+1];
		memcpy(argv[j],args[j].c_str(),args[j].size());
		argv[j][args[j].size()] = 0;
	}
}



#endif /* ARGS_H_ */

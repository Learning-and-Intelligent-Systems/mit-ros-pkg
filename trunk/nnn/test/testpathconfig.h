
#ifndef IMAGEPATHCONFIG_H_
#define IMAGEPATHCONFIG_H_

//this file translates cmake variables into source variables

//static char log_path[] = "${PROJECT_SOURCE_DIR}/test_data/testlogs/";

static char *logfiles[] = {${filelist}};
static int num_log_files = ${numfiles};

#endif /* IMAGEPATHCONFIG_H_ */

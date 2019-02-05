#include "log.h"
#include <time.h>

#define MODULE_NAME "[LOGGER  ]"

void log_open(char *filename) {
	logfile = fopen(filename,"wt");
	time_t t = time(NULL);
	struct tm* tmp = localtime(&t);
	char tstr[128];
	strftime(tstr,sizeof(tstr),"%T %d/%m/%y",tmp);
	fprintf(logfile,"\n");
	LOG_INFO("Log opened at %s\n",tstr);
}

void log_close() {
	if (logfile)
		fclose(logfile);
}

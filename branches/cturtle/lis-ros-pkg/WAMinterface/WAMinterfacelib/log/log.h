#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

FILE *logfile;

void log_open(char *filename);
void log_close();

#define LOG(module, type, fmt, ... ) fprintf(logfile,module type fmt, ## __VA_ARGS__)

#define LOG_ERROR(fmt, ... ) LOG(MODULE_NAME," [ERR ] ",fmt, ## __VA_ARGS__)
#define LOG_INFO(fmt, ... ) LOG(MODULE_NAME," [INFO] ",fmt, ## __VA_ARGS__)


#ifdef __cplusplus
}
#endif

#endif

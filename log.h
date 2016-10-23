#ifndef __LOG_H__
#define __LOG_H__

#include <iostream>
#include <string>
#include <stdio.h>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sys/time.h>

#define TO_FILE     0
#define TO_CONSOLE  0

using namespace std;

/* Zones */
#ifndef ZONE_NONE
#define ZONE_NONE       (0U<<0)
#endif
#ifndef ZONE_WARNING
#define ZONE_WARNING    (1U<<0)
#endif
#ifndef ZONE_ERROR
#define ZONE_ERROR      (1U<<1)
#endif
#ifndef ZONE_INFO
#define ZONE_INFO       (1U<<2)
#endif
#ifndef ZONE_TIME
#define ZONE_TIME       (1U<<3)
#endif

// Level: INFO
#define info_print(...) \
    do { \
    if (logLevel & ZONE_INFO) { \
        char str[256]; \
        sprintf(str,"- %s[%s:%.30s()]", NowTime().c_str(), __FILE__, __func__); \
        fprintf(stdout, "%-50s \tINFO:   \t", str); \
        fprintf(stdout, __VA_ARGS__); \
        fflush(stdout); \
        if (logFile != NULL) { \
        fprintf(logFile, "%-50s \tINFO:   \t", str); \
        fprintf(logFile, __VA_ARGS__); \
        fflush(logFile); \
        } \
    }\
    } while (0)

// Level: WARNING
#define warning_print(...) \
    do { \
    if (logLevel & ZONE_WARNING) { \
        char str[256]; \
        sprintf(str,"- %s[%s:%.30s()]", NowTime().c_str(), __FILE__, __func__); \
        fprintf(stdout, "%-50s \tWARNING:\t", str); \
        fprintf(stdout, __VA_ARGS__); \
        fflush(stdout); \
        if (logFile != NULL) { \
        fprintf(logFile, "%-50s \tWARNING:\t", str); \
        fprintf(logFile, __VA_ARGS__); \
        fflush(logFile); \
        } \
    }\
    } while (0)

// Level: ERROR
#define error_print(...) \
    do { \
    if (logLevel & ZONE_ERROR) { \
        char str[256]; \
        sprintf(str,"- %s[%s:%.30s()]", NowTime().c_str(), __FILE__, __func__); \
        fprintf(stderr, "%-50s \tERROR:   \t", str); \
        fprintf(stderr, __VA_ARGS__); \
        fflush(stderr); \
        if (logFile != NULL) { \
        fprintf(logFile, "%-50s \tERROR:   \t", str); \
        fprintf(logFile, __VA_ARGS__); \
        fflush(logFile); \
        } \
    }\
    } while (0)

// Level: TIMING
#define timing_print(...) \
    do { \
    if (logLevel & ZONE_TIME) { \
        char str[256]; \
        sprintf(str,"- %s[%s:%.30s()]", NowTime().c_str(), __FILE__, __func__); \
        fprintf(stdout, "%-50s \tTIME:   \t", str); \
        fprintf(stdout, __VA_ARGS__); \
        fflush(stdout); \
        if (logFile != NULL) { \
        fprintf(logFile, "%-50s \tTIME:   \t", str); \
        fprintf(logFile, __VA_ARGS__); \
        fflush(logFile); \
        } \
    }\
    } while (0)

extern FILE*    logFile;
extern unsigned logLevel;

inline std::string NowTime()
{
#ifndef __NO_TIME__
    char buffer[11];
    time_t t;
    time(&t);
    tm r = {0};
    strftime(buffer, sizeof(buffer), "%X", localtime_r(&t, &r));
    struct timeval tv;
    gettimeofday(&tv, 0);
    char result[256] = {0};
    std::sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000);
#else
    char result[256] = {0};
#endif
    std::string str(result, result + 50);
    return str;
}

#endif //__LOG_H__

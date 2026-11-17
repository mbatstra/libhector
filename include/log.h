#ifndef __LOG_H
#define __LOG_H

// #include "config.h"

#ifndef LOG_DEVICE
            #define LOG_DEVICE  Serial
#endif

// Helpers
#ifndef __STRINGIFY
#  define __STRINGIFY(x)    #x
#endif
#define ___STRINGIFY(x) __STRINGIFY(x)
#define __LINESTR__ ___STRINGIFY(__LINE__)

#define DBG(msg, ...)      LOG_DEVICE.printf("%s.%s: " msg "\n", __FILE__, __FUNCTION__, ##__VA_ARGS__)
#define INFO(msg, ...)	   LOG_DEVICE.printf("%s: " msg "\n", __FUNCTION__, ##__VA_ARGS__)
#define WARNING(msg, ...)  LOG_DEVICE.printf("WARN :" __FILE__ ".%s(" __LINESTR__ "): " msg "\n", __FUNCTION__, ##__VA_ARGS__)
#define ERROR(msg, ...)    LOG_DEVICE.printf("ERROR:" __FILE__ ".%s(" __LINESTR__ "): " msg "\n", __FUNCTION__, ##__VA_ARGS__)

#endif // __LOG_H
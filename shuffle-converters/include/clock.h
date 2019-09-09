// adopted from other header files to move clock configuration out of main.c

#ifndef __clock_H
#define __clock_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif
#endif /*__ clock_H */
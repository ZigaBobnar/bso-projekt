/* Gyromouse FreeRTOSConfig overrides.

   All other configuration settings are found in FreeRTOS/Source/include/FreeRTOSConfig.h
*/

#define configUSE_PREEMPTION     1
// #define configUSE_PREEMPTION     0
#define configMINIMAL_STACK_SIZE ( ( unsigned short)256 )
// #define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )


/* Use the defaults for everything else */
#include_next<FreeRTOSConfig.h>

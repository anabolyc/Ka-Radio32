#ifndef _INCLUDE_COMMON_TASKS_H_
#define _INCLUDE_COMMON_TASKS_H_

// Tasks priority
#define PRIO_MAD 		7
#define PRIO_VS1053 	6
#define PRIO_RMT		5
#define PRIO_UART		2
#define PRIO_CLIENT		8
#define PRIO_SERVER		6
#define PRIO_ADDON		7
#define PRIO_LCD		7
#define PRIO_SUBSERV	7
#define PRIO_TIMER		10
#define PRIO_OTA		8

// CPU for task
#define CPU_MAD			1  // internal decoder and vs1053
#define CPU_RMT			0
#define CPU_UART		1
#define CPU_CLIENT		0
#define CPU_SERVER		0
#define CPU_ADDON		0
#define CPU_LCD			0
#define CPU_SUBSERV		0
#define CPU_TIMER		0

#endif
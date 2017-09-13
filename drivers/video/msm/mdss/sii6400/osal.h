/*
  SiI6400 Linux Driver

  Copyright (C) 2012-2013 Silicon Image, Inc.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation version 2.

  This program is distributed "AS-IS" WITHOUT ANY WARRANTY of any
  kind, whether express or implied; INCLUDING without the implied warranty
  of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
  See the GNU General Public License for more details at
  http://www.gnu.org/licenses/gpl-2.0.html.
*/

#ifndef _OSAL_H
#define _OSAL_H

#include "sii_common.h"
#include "osal_linux_kernel.h"

enum sii_os_status {
	SII_OS_STATUS_SUCCESS = 0,
	SII_OS_STATUS_ERR_INVALID_PARAM = -1,
	SII_OS_STATUS_ERR_FAILED = -2,
	SII_OS_STATUS_ERR_NOT_AVAIL = -3,
	SII_OS_STATUS_ERR_SEM_COUNT_EXCEEDED = -4,
	SII_OS_STATUS_QUEUE_FULL = -5,
	SII_OS_STATUS_QUEUE_EMPTY = -6,
	SII_OS_STATUS_TIMEOUT = -7,
	SII_OS_STATUS_ERR_INVALID_OP = -8,
	SII_OS_STATUS_ERR_INVALID_HANDLE = -9,
	SII_OS_STATUS_ERR_IN_USE = -10,
};

#define INT16_T_MAX	(int16_t)((1UL << ((sizeof(int16_t) << 3) - 1)) - 1)
#define INT32_T_MAX	(int32_t)((1UL << ((sizeof(int32_t) << 3) - 1)) - 1)
#define INT64_T_MAX	(int64_t)((1ULL << ((sizeof(int64_t) << 3) - 1)) - 1)

#define SII_OS_INFINITE_WAIT	-1
#define SII_OS_NO_WAIT		0
#define SII_OS_STACK_SIZE_AUTO	-1
#define SII_OS_STATUS_NULL	NULL

struct SiiOsDebugChannel {
	uint32_t channel;
	char *channelName;
};

#define SII_OS_DEBUG_FORMAT_SIMPLE	0x0000u
#define SII_OS_DEBUG_FORMAT_FILEINFO	0x0001u
#define SII_OS_DEBUG_FORMAT_CHANNEL	0x0002u
#define SII_OS_DEBUG_FORMAT_TIMESTAMP	0x0004u

#ifdef SII_OSAL_DEBUG_PRINT
#ifdef SHORT_DEBUG_MESSAGES
#define SII_DEBUG_PRINT(channel, fmt, ...) \
	do { \
		if (DEBUG_LEVEL_NONE != debug_level) \
			SiiOsDebugPrintShort(channel, fmt, ##__VA_ARGS__); \
	} while (0)
#else
#define SII_DEBUG_PRINT(channel, fmt, ...) \
	do { \
		if (DEBUG_LEVEL_NONE != debug_level) \
			SiiOsDebugPrint(__FILE__, __LINE__, channel, \
					fmt, ##__VA_ARGS__); \
	} while (0)
#endif

#define SII_DEBUG_PRINT_SIMPLE(channel, fmt, ...) \
	do { \
		if (DEBUG_LEVEL_NONE != debug_level) \
			SiiOsDebugPrintSimple(channel, fmt, ##__VA_ARGS__); \
	} while (0)

#define SII_OS_DEBUG_ASSERT(assertionCondition) \
	do { \
		uint32_t expressionEvaluation = assertionCondition; \
		if (!expressionEvaluation) { \
			SiiOsDebugAssert(__FILE__, __LINE__, \
				expressionEvaluation, "#assertion condition"); \
		} \
	} while (0)
#else /* SII_OSAL_DEBUG_PRINT */
#define SII_DEBUG_PRINT(channel, ...)
#define SII_DEBUG_PRINT_SIMPLE(channel, ...)
#define SII_OS_DEBUG_ASSERT(assertionCondition)
#endif /* SII_OSAL_DEBUG_PRINT */

enum sii_os_status SiiOsInit(uint32_t maxChannels);
enum sii_os_status SiiOsTerm(void);

enum sii_os_status SiiOsSemaphoreCreate(const char *pName, uint32_t maxCount,
					uint32_t initialValue,
					struct SiiOsSemaphore **pRetSemId);
enum sii_os_status SiiOsSemaphoreDelete(struct SiiOsSemaphore *semId);
enum sii_os_status SiiOsSemaphoreGive(struct SiiOsSemaphore *semId);
enum sii_os_status SiiOsSemaphoreTake(struct SiiOsSemaphore *semId,
					int32_t timeMsec);

enum sii_os_status SiiOsQueueCreate(const char *pName, uint32_t elementSize,
					uint32_t maxElements,
					struct SiiOsQueue **pRetQueueId);
enum sii_os_status SiiOsQueueDelete(struct SiiOsQueue *queueId);
enum sii_os_status SiiOsQueueSend(struct SiiOsQueue *queueId,
					void *pBuffer, uint32_t size);
enum sii_os_status SiiOsQueueReceive(struct SiiOsQueue *queueId, void *pBuffer,
					int32_t timeMsec, uint32_t *pSize);

enum sii_os_status SiiOsTaskCreate(const char *pName,
				void (*pEntryPoint)(void *pArg),
				void *pTaskArg, uint32_t priority,
				uint32_t stackSize, bool taskRunFlag,
				struct work_struct **pRetTaskId);
enum sii_os_status SiiOsTaskSelfDelete(void);
enum sii_os_status SiiOsTaskActivate(struct work_struct *taskId);
enum sii_os_status SiiOsTaskSleepUsec(uint64_t timeUsec);
enum sii_os_status SiiOsTaskSetPriority(struct work_struct *taskId,
					uint32_t newPriority);
enum sii_os_status SiiOsTaskGetPriority(struct work_struct *taskId,
					uint32_t *pPriority);

enum sii_os_status SiiOsTimerCreate(const char *pName,
				void (*pTimerFunction)(void *pArg),
				void *pTimerArg, bool timerStartFlag,
				uint32_t timeMsec, bool periodicFlag,
				struct SiiOsTimer **pRetTimerId);
enum sii_os_status SiiOsTimerDelete(struct SiiOsTimer *timerId);
enum sii_os_status SiiOsTimerSchedule(struct SiiOsTimer *timerId,
					uint32_t timeMsec);
uint32_t SiiOsGetTimeResolution(void);
void SiiOsGetTimeCurrent(struct SiiOsTime *pTime);
int64_t SiiOsGetTimeDifferenceMs(const struct SiiOsTime *pTime1,
				const struct SiiOsTime *pTime2);
int16_t SiiOsGetTimeCurrentMs16(void);
int32_t SiiOsGetTimeCurrentMs32(void);
int64_t SiiOsGetTimeCurrentMs64(void);

void *SiiOsAlloc(const char *pName, uint32_t size, uint32_t flags);
void *SiiOsCalloc(const char *pName, uint32_t size, uint32_t flags);
void *SiiOsRealloc(const char *pName, void *pAddr, uint32_t size,
			uint32_t flags);
void SiiOsFree(void *pAddr);

enum sii_os_status SiiOsBlockPoolCreate(struct SiiOsBlockPool **pRetPool,
					const char *pName, uint32_t blockSize,
					void *poolStart, uint32_t poolSize);
enum sii_os_status SiiOsBlockPoolDelete(struct SiiOsBlockPool *pPool);
enum sii_os_status SiiOsBlockAllocate(struct SiiOsBlockPool *pPool,
					void **pRetBlock);
enum sii_os_status SiiOsBlockRelease(struct SiiOsBlockPool *pPool,
					void *pBlock);

enum sii_os_status SiiOsBytePoolCreate(struct SiiOsBytePool **pRetPool,
					const char *pName, void *poolStart,
					uint32_t poolSize);
enum sii_os_status SiiOsBytePoolDelete(struct SiiOsBytePool *pPool);
enum sii_os_status SiiOsByteAllocate(struct SiiOsBytePool *pPool,
					void **pRetByte, uint32_t size);
enum sii_os_status SiiOsByteRelease(struct SiiOsBytePool *pPool, void *pByte);

enum sii_os_status SiiOsDebugChannelAdd(uint32_t numChannels,
				struct SiiOsDebugChannel *pChannelList);
void SiiOsDebugChannelEnable(uint32_t channel);
void SiiOsDebugChannelDisable(uint32_t channel);
bool SiiOsDebugChannelIsEnabled(uint32_t channel);
void SiiOsDebugPrint(const char *pFileName, uint32_t lineNumber,
			uint32_t channel, const char *pFormat, ...);
void SiiOsDebugPrintShort(uint32_t channel, const char *pFormat, ...);
void SiiOsDebugPrintSimple(uint32_t channel, const char *pFormat, ...);
void SiiOsDebugConfig(uint16_t flags);
void SiiOsDebugAssert(const char *pFileName, uint32_t lineNumber,
			uint32_t expressionEvaluation,
			const char *pConditionText);
void SiiOsDebugInstallAssertHandler(void (*pHandler)(void *pArg), void *pArg);

#endif /* !_OSAL_H */


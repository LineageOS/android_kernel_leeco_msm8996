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

#include "osal.h"
#include <linux/fs.h>
#include <linux/uaccess.h>
#if 1 /* temp include */
#include <linux/delay.h>
#endif

static uint32_t maxDbgChannels;

static void SiiOsTimerFunction(unsigned long data);
static void SiiOsTimerRearm(struct SiiOsTimer *timerId);
#ifdef SII_OSAL_DEBUG_PRINT
static const char *simple_file_name(const char *path);
#endif
static struct file *create_nv_storage(void);


/*
 * Initialize the OSAL and allocate required resources.
 *
 * Parameters:
 *	maxChannels: Maximum number of debug channels to support.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if maxChannels == 0
 */
enum sii_os_status SiiOsInit(uint32_t maxChannels)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (0 == maxChannels) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}
	maxDbgChannels = maxChannels;

done:
	return retval;
}

/*
 * Terminate the OSAL and deallocate all associated resources.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_FAILED if any error occurs
 */
enum sii_os_status SiiOsTerm(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	return retval;
}

/*
 * Create a binary or counting semaphore.
 *
 * Parameters:
 *	pName: Semaphore identifier (16 chars max).
 *	maxCount: Max semaphore count (1 for binary semaphore).
 *	initialValue: Initial count on semaphore (0 for semaphore unavailable).
 *	pRetSemId: Returned semaphore object identifier or NULL on failure.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_NOT_AVAIL if insufficient memory
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsSemaphoreCreate(const char *pName, uint32_t maxCount,
					uint32_t initialValue,
					struct SiiOsSemaphore **pRetSemId)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct SiiOsSemaphore *semaphoreData = NULL;

	dbg("%s", (NULL != pName) ? pName : "");

	if (NULL == pRetSemId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}
	*pRetSemId = NULL;

	if (maxCount < initialValue) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	semaphoreData = SiiOsCalloc(pName, sizeof(struct SiiOsSemaphore), 0);
	if (NULL == semaphoreData) {
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto done;
	}

	if (NULL != pName) {
		strlcpy(semaphoreData->name, pName,
						sizeof(semaphoreData->name));
	}
	spin_lock_init(&semaphoreData->lock);
	semaphoreData->count = initialValue;
	semaphoreData->maxCount = maxCount;
	sema_init(&semaphoreData->semaphore, initialValue);
	semaphoreData->usable = true;
	*pRetSemId = semaphoreData;

done:
	if (SII_OS_STATUS_SUCCESS != retval)
		SiiOsFree(semaphoreData);
	return retval;
}

/*
 * Delete a semaphore.
 *
 * Parameters:
 *	semId: Semaphore object identifier.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 */
enum sii_os_status SiiOsSemaphoreDelete(struct SiiOsSemaphore *semId)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	int retry = 100;

	if (NULL == semId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	dbg("%s", semId->name);

	semId->usable = false;

	/* ToDo: make sure nobody is using the semaphore before freeing this
	 * structure!!!! Temporary sleep to ensure semaphore is not being used.
	 * This is not sufficient! */
	do {
		msleep(50);
		retry--;
		warn("total=100, now retry=%d\n", retry);
	} while ((retry > 0) && (semId->tryuse > 0));

	SiiOsFree(semId);

done:
	return retval;
}

/*
 * Release a semaphore.
 *
 * Parameters:
 *	semId: Semaphore object identifier.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_SEM_COUNT_EXCEEEDED if too many releases occurred
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsSemaphoreGive(struct SiiOsSemaphore *semId)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	unsigned long flags;

	if (NULL == semId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/*dbg("%s", semId->name);*/

	if (!semId->usable) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	spin_lock_irqsave(&semId->lock, flags);
	if (semId->count == semId->maxCount)
		retval = SII_OS_STATUS_ERR_SEM_COUNT_EXCEEDED;
	else
		semId->count++;
	spin_unlock_irqrestore(&semId->lock, flags);

	semId->tryuse++;
	up(&semId->semaphore);
	semId->tryuse--;

done:
	return retval;
}

/*
 * Take a semaphore with or without a timeout.
 *
 * Parameters:
 *	semId: Semaphore object identifier.
 *	timeMsec: Timeout period to wait for semaphore in milliseconds.
 *	  Possible values:
 *	    SII_OS_INFINITE_WAIT to block waiting for the semaphore.
 *	    SII_OS_NO_WAIT to return immediately if semaphore is not available.
 *	    1 -> 2147483647 (0x7fffffff) for a finite wait for the semaphore.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_TIMEOUT on a timeout
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsSemaphoreTake(struct SiiOsSemaphore *semId,
					int32_t timeMsec)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	unsigned long flags;
	int down_retval;

	if (NULL == semId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/*dbg("%s", semId->name);*/

	if (!semId->usable) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	semId->tryuse++;
	down_retval = down_timeout(&semId->semaphore,
					msecs_to_jiffies(timeMsec));
	if (down_retval) {
		if (-ETIME == down_retval)
			retval = SII_OS_STATUS_TIMEOUT;
		else
			retval = SII_OS_STATUS_ERR_FAILED;
	} else {
		spin_lock_irqsave(&semId->lock, flags);
		if (0 < semId->count)
			semId->count--;
		spin_unlock_irqrestore(&semId->lock, flags);
	}
	semId->tryuse--;

done:
	return retval;
}

/*
 * Create a message queue. The element size specifies the maximum size of each
 * message. Smaller messages may be stored in the queue. The actual size of each
 * message is detailed in the receive API.
 *
 * Parameters:
 *	pName: Message queue identifier (16 chars max).
 *	elementSize: Size of each element in the queue.
 *	maxElements: Maximum number of elements in the queue.
 *	pRetQueueId: Returned message queue object identifier or NULL
 *	on failure.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_NOT_AVAIL if insufficient memory
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsQueueCreate(const char *pName, uint32_t elementSize,
					uint32_t maxElements,
					struct SiiOsQueue **pRetQueueId)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct SiiOsQueue *queueData = NULL;
	unsigned int queueSize = 0;
	unsigned int kfifoSize = 0;
	int kfifo_size_order = 0;

	dbg("%s", (NULL != pName) ? pName : "");

	if (NULL == pRetQueueId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}
	*pRetQueueId = NULL;

	if ((0 == elementSize) || (0 == maxElements)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	queueData = SiiOsCalloc(pName, sizeof(struct SiiOsQueue), 0);
	if (NULL == queueData) {
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto done;
	}

	if (NULL != pName)
		strlcpy(queueData->name, pName, sizeof(queueData->name));
	init_waitqueue_head(&queueData->wait);
	sema_init(&queueData->semaphore, 1);
	spin_lock_init(&queueData->lock);
	queueSize = elementSize * maxElements;
	kfifo_size_order = order_base_2(queueSize);
	kfifoSize = (1 << kfifo_size_order);
	if (queueSize > kfifoSize)
		warn("Message queue smaller than requested\n");
	if (kfifo_alloc(&queueData->queue, kfifoSize, GFP_KERNEL)) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	queueData->esize = elementSize;
	queueData->usable = true;
	*pRetQueueId = queueData;

done:
	if (SII_OS_STATUS_SUCCESS != retval)
		SiiOsFree(queueData);
	return retval;
}

/*
 * Delete a message queue.
 *
 * Parameters:
 *	queueId: Message queue object identifier.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsQueueDelete(struct SiiOsQueue *queueId)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	if (NULL == queueId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	dbg("%s", queueId->name);

	queueId->usable = false;

	wake_up_interruptible_sync(&queueId->wait);

	/* ToDo: make sure nobody is using the semaphore before freeing this
	 * structure!!!! Temporary sleep to ensure semaphore is not being used.
	 * This is not sufficient! */
	msleep(50);

	if (down_interruptible(&queueId->semaphore)) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	kfifo_free(&queueId->queue);

	up(&queueId->semaphore);

	SiiOsFree(queueId);

done:
	return retval;
}

/*
 * Send a message (put a message on the queue).
 *
 * Parameters:
 *	queueId: Message queue object identifier.
 *	pBuffer: Pointer to the message to enqueue.
 *	size: Size of the message. Must be less than or equal to the max size
 *		set at queue creation.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_QUEUE_FULL if the queue has no free space
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsQueueSend(struct SiiOsQueue *queueId,
					void *pBuffer, uint32_t size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint32_t count = 0;

	if ((NULL == queueId) || (NULL == pBuffer) ||
	    (queueId->esize < size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/*dbg("%s", queueId->name);*/

	if (!queueId->usable) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (down_interruptible(&queueId->semaphore)) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	if (kfifo_avail(&queueId->queue) < size) {
		up(&queueId->semaphore);
		warn("Message queue is full\n");
		retval = SII_OS_STATUS_QUEUE_FULL;
		goto done;
	}

	count = kfifo_in_spinlocked(&queueId->queue, pBuffer, size,
					&queueId->lock);
	if (size != count) {
		up(&queueId->semaphore);
		dbg("Enqueued message size %d, message to enqueue size %d\n",
		    count, size);
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	up(&queueId->semaphore);

	wake_up_interruptible(&queueId->wait);

done:
	return retval;
}

/*
 * Receive a message (take a message from the queue).
 *
 * Parameters:
 *	queueId: Message queue object identifier.
 *	pBuffer: Pointer to a buffer to hold the dequeued message.
 *	timeMsec: Timeout period to wait for a message in milliseconds.
 *	  Possible values:
 *	    SII_OS_INFINITE_WAIT to block waiting for something on the queue.
 *	    SII_OS_NO_WAIT to return immediately if queue is empty.
 *	    1 -> 2147483647 (0x7fffffff) for a finite wait.
 *	pSize:	 Input - size of pBuffer
 *		Output - size of the dequeued message
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_TIMEOUT on a timeout
 *	SII_OS_STATUS_QUEUE_EMPTY if queue is empty and timeMsec==OS_NO_WAIT
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsQueueReceive(struct SiiOsQueue *queueId, void *pBuffer,
					int32_t timeMsec, uint32_t *pSize)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint32_t count = 0;
	int error = 0;

	if ((NULL == queueId) || (NULL == pBuffer) || (NULL == pSize)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/*dbg("%s", queueId->name);*/

	if (down_interruptible(&queueId->semaphore)) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	if (kfifo_is_empty(&queueId->queue)) {
		up(&queueId->semaphore);

		if (SII_OS_NO_WAIT == timeMsec) {
			retval = SII_OS_STATUS_QUEUE_EMPTY;
			goto done;
		} else if ((0 < timeMsec) ||
			   (SII_OS_INFINITE_WAIT == timeMsec)) {
			error = wait_event_interruptible_timeout(queueId->wait,
					(!(kfifo_is_empty(&queueId->queue))),
					msecs_to_jiffies(timeMsec));
			if (0 == error) {
				retval = SII_OS_STATUS_TIMEOUT;
				goto done;
			} else if (error < 0) {
				retval = SII_OS_STATUS_ERR_FAILED;
				goto done;
			}
		} else {
			err("Invalid timeMsec parameter %d\n", timeMsec);
			retval = SII_OS_STATUS_ERR_INVALID_PARAM;
			goto done;
		}

		if (down_interruptible(&queueId->semaphore)) {
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}
	}

	count = kfifo_out_spinlocked(&queueId->queue, pBuffer,
					*pSize, &queueId->lock);
	up(&queueId->semaphore);
	if (*pSize != count) {
		warn("Size of dequeued message %d, size of message buffer %d\n",
		     count, *pSize);
	}
	*pSize = count;

done:
	return retval;
}

/*
 * Create a task.
 *
 * Parameters:
 *	pName: Task identifier (16 chars max).
 *	pEntryPoint: Task function entry address.
 *	pTaskArg: Parameter passed to task function.
 *	priority: Task priority (for kernel tasks: 15 -> 31,
 *					with 15 lowest priority).
 *	stackSize: Bytes added to the OS required stack size. Possible values:
 *		SII_OS_STACK_SIZE_AUTO to use the OS required stack size
 *	taskRunFlag: If 'true', task is activated immediately.
 *		     If 'false', task is activated only with
 *		     SiiOsTaskActivate().
 *	pRetTaskId: Returned task object identifier or NULL on failure.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_NOT_AVAIL if there is insufficient memory
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsTaskCreate(const char *pName,
				void (*pEntryPoint)(void *pArg),
				void *pTaskArg, uint32_t priority,
				uint32_t stackSize, bool taskRunFlag,
				struct work_struct **pRetTaskId)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_FAILED;

	dbg("%s", (NULL != pName) ? pName : "");

	if ((NULL == pRetTaskId) || (NULL == pEntryPoint)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* Not implemented */

done:
	return retval;
}

/*
 * Self-delete a task.
 *
 * Return value:
 *	Does not return anything if successful
 *	SII_OS_STATUS_ERR_FAILED if task was not created with SiiOsTaskCreate()
 */
enum sii_os_status SiiOsTaskSelfDelete(void)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_FAILED;

	dbg("");

	/* Not implemented */

	return retval;
}

/*
 * Activate a task that was immediately suspended upon creation.
 *
 * Parameters:
 *	taskId: Task object identifier.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsTaskActivate(struct work_struct *taskId)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_FAILED;

	dbg("");

	if (NULL == taskId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* Not implemented */

done:
	return retval;
}

/*
 * Self-put a task to sleep for a finite period of time.
 *
 * Parameters:
 *	timeUsec: Time to sleep, in microseconds.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsTaskSleepUsec(uint64_t timeUsec)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_FAILED;

	dbg("");

	/* Not implemented */

	return retval;
}

/*
 * Set (change) a task's priority.
 *
 * Parameters:
 *	taskId: Task object identifier.
 *	newPriority: Task priority (for kernel tasks: 15 -> 31,
 *					with 15 lowest priority).
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsTaskSetPriority(struct work_struct *taskId,
					uint32_t newPriority)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_FAILED;

	dbg("");

	if (NULL == taskId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* Not implemented */

done:
	return retval;
}

/*
 * Get a task's priority.
 *
 * Parameters:
 *	taskId: Task object identifier.
 *	pPriority: Returned task priority.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsTaskGetPriority(struct work_struct *taskId,
					uint32_t *pPriority)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_FAILED;

	dbg("");

	if ((NULL == taskId) || (NULL == pPriority)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* Not implemented */

done:
	return retval;
}

/*
 * Create a one-shot or a periodic timer.
 *
 * Parameters:
 *	pName: Timer identifier (16 chars max).
 *	pTimerFunction: Function to run when timer fires.
 *	pTimerArg: parameter passed to timer function.
 *	timerStartFlag: If 'true', timer is activated immediately.
 *			If 'false', timer is activated only with
 *						SiiOsTimerSchedule().
 *	timeMsec: Timeout period, in milliseconds.
 *			(Only used if timerStartFlag=='true').
 *	periodicFlag: If 'true', a periodic timer is created.
 *		      If 'false', a one-shot timer is created.
 *	pRetTimerId: Returned timer object identifier or NULL on failure.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_NOT_AVAIL if there is insufficient memory
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsTimerCreate(const char *pName,
			void (*pTimerFunction)(void *pArg),
			void *pTimerArg, bool timerStartFlag,
			uint32_t timeMsec, bool periodicFlag,
			struct SiiOsTimer **pRetTimerId)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct SiiOsTimer *timerData = NULL;

	dbg("%s", (NULL != pName) ? pName : "");

	if (NULL == pRetTimerId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}
	*pRetTimerId = NULL;

	if ((NULL == pTimerFunction)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	timerData = SiiOsCalloc(pName, sizeof(struct SiiOsTimer), 0);
	if (NULL == timerData) {
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto done;
	}

	if (NULL != pName)
		strlcpy(timerData->name, pName, sizeof(timerData->name));
	init_timer(&timerData->timer);
	timerData->timer.function = SiiOsTimerFunction;
	timerData->timer.data = (unsigned long)timerData;
	timerData->function = pTimerFunction;
	timerData->data = pTimerArg;
	timerData->rearm = periodicFlag;
	if (timerStartFlag) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerSchedule(timerData, timeMsec)) {
			err("Timer not started\n");
		}
	}
	timerData->usable = true;

	*pRetTimerId = timerData;

done:
	if (SII_OS_STATUS_SUCCESS != retval)
		SiiOsFree(timerData);
	return retval;
}

/*
 * Delete a timer. If the timer function is active, it will run until it exits.
 * If the timer has not fired, it is dequeued before deletion.
 *
 * Parameters:
 *	timerId: Timer object identifier.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsTimerDelete(struct SiiOsTimer *timerId)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	if (NULL == timerId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	dbg("%s", timerId->name);

	timerId->usable = false;
	timerId->rearm = false;
	del_timer_sync(&timerId->timer);
	SiiOsFree(timerId);

done:
	return retval;
}

/*
 * Activate a timer.
 *
 * Parameters:
 *	timerId: Timer object identifier.
 *	timeMsec: Timeout period, in milliseconds.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsTimerSchedule(struct SiiOsTimer *timerId,
					uint32_t timeMsec)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	if (NULL == timerId) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	dbg("%s", timerId->name);

	if (!timerId->usable) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	timerId->timeout = msecs_to_jiffies(timeMsec);
	SiiOsTimerRearm(timerId);

done:
	return retval;
}

static void SiiOsTimerFunction(unsigned long data)
{
	struct SiiOsTimer *timerId = (struct SiiOsTimer *)data;

	if (NULL != timerId) {
		dbg("%s", timerId->name);

		(timerId->function)(timerId->data);
		if ((timerId->rearm) && (timerId->usable))
			SiiOsTimerRearm(timerId);
	}
}

static void SiiOsTimerRearm(struct SiiOsTimer *timerId)
{
	if (NULL != timerId) {
		dbg("%s", timerId->name);

		if (timerId->usable)
			mod_timer(&timerId->timer, jiffies + timerId->timeout);
	}
}

/*
 * Returns the minimum supported time resolution in milliseconds.
 */
uint32_t SiiOsGetTimeResolution(void)
{
	return 1000 / HZ;
}

/*
 * Get the current system time since system startup.
 *
 * Parameters:
 *	pTime: User-supplied time structure returned with current time.
 */
void SiiOsGetTimeCurrent(struct SiiOsTime *pTime)
{
	if (NULL != pTime)
		getrawmonotonic(&pTime->time);
}

/*
 * Get the difference between two time values in milliseconds.
 *
 * Parameters:
 *	pTime1: First time value to compare.
 *	pTime2: Second time value to compare.
 *
 * Return value:
 *	Time difference in milliseconds. The result will be negative when time1
 *	is smaller than time2.
 */
int64_t SiiOsGetTimeDifferenceMs(const struct SiiOsTime *pTime1,
				const struct SiiOsTime *pTime2)
{
	struct timespec timediff;

	if ((NULL == pTime1) || (NULL == pTime2))
		return 0LL;
	timediff = timespec_sub(pTime1->time, pTime2->time);
	return ((int64_t)timediff.tv_sec * MSEC_PER_SEC) +
				timediff.tv_nsec / NSEC_PER_MSEC;
}

/*
 * Returns current system time since system startup in milliseconds.
 * If the value is larger than will fit in a 16 bit signed integer, then
 * the max value is returned. Should never be negative.
 */
int16_t SiiOsGetTimeCurrentMs16(void)
{
	int64_t now_msec = SiiOsGetTimeCurrentMs64();
	return (now_msec < 0) ? INT16_T_MAX : (now_msec < INT16_T_MAX) ?
				(int16_t)now_msec : INT16_T_MAX;
}

/*
 * Returns current system time since system startup in milliseconds.
 * If the value is larger than will fit in a 32 bit signed integer, then
 * the max value is returned. Should never be negative.
 */
int32_t SiiOsGetTimeCurrentMs32(void)
{
	int64_t now_msec = SiiOsGetTimeCurrentMs64();
	return (now_msec < 0) ? INT32_T_MAX : (now_msec < INT32_T_MAX) ?
				(int32_t)now_msec : INT32_T_MAX;
}

/*
 * Returns current system time since system startup in milliseconds.
 * If the value is larger than will fit in a 64 bit signed integer, then
 * the max value is returned. Should never be negative.
 */
int64_t SiiOsGetTimeCurrentMs64(void)
{
	struct timespec now;
	int64_t now_msec;

	getboottime(&now);
	now_msec = ((int64_t)now.tv_sec * MSEC_PER_SEC) +
				now.tv_nsec / NSEC_PER_MSEC;
	return (now_msec < 0) ? INT64_T_MAX : (now_msec < INT64_T_MAX) ?
				now_msec : INT64_T_MAX;
}

/*
 * Allocate OS memory.
 *
 * Parameters:
 *	pName: name of the block.
 *	size: size in bytes.
 *	flags: OS supported flags (default=0). Currently, flags is ignored.
 *
 * Return value:
 *	Pointer to allocated memory or SII_OS_STATUS_NULL
 */
void *SiiOsAlloc(const char *pName, uint32_t size, uint32_t flags)
{
	void *ptr = kmalloc((size_t)size, GFP_KERNEL);

	/*dbg("Allocing 0x%p", ptr);*/
	return ptr;
}

/*
 * Allocate OS memory and initialize the memory to 0.
 *
 * Parameters:
 *	pName: name of the block.
 *	size: size in bytes.
 *	flags: OS supported flags (default=0). Currently, flags is ignored.
 *
 * Return value:
 *	Pointer to allocated memory or SII_OS_STATUS_NULL
 */
void *SiiOsCalloc(const char *pName, uint32_t size, uint32_t flags)
{
	void *ptr = kzalloc((size_t)size, GFP_KERNEL);

	/*dbg("Callocing 0x%p", ptr);*/
	return ptr;
}

/*
 * Reallocate OS memory. The contents of the block are preserved up to the
 * lesser of the new and old sizes. If new size is smaller than old size,
 * an existing string will be truncated and not be NULL-terminated.
 *
 * Parameters:
 *	pName: name of the block.
 *	pAddr: address of previously allocated block.
 *	size: new size in bytes.
 *	flags: OS supported flags (default=0). Currently, flags is ignored.
 *
 * Return value:
 *	Pointer to allocated memory or SII_OS_STATUS_NULL
 */
void *
SiiOsRealloc(const char *pName, void *pAddr, uint32_t size, uint32_t flags)
{
	void *ptr = krealloc(pAddr, (size_t)size, GFP_KERNEL);

	/*dbg("Reallocing 0x%p", ptr);*/
	return ptr;
}

/*
 * Free OS memory.
 *
 * Parameters:
 *	pAddr: address of memory allocated with SiiOsAlloc or SiiOsCalloc.
 *		If pAddr==NULL, then nothing happens.
 */
void SiiOsFree(void *pAddr)
{
	if (NULL != pAddr) {
		/*dbg("Freeing 0x%p", pAddr);*/
		kfree(pAddr);
	}
}

/*
 * Create a memory block pool using the supplied memory buffer.
 *
 * Parameters:
 *	pRetPool: Returned memory block pool structure or NULL on failure.
 *	pName: Optional name for the memory block pool.
 *	blockSize: Size of blocks in the memory block pool in bytes.
 *	poolStart: User-supplied buffer for the memory block pool. (Not used)
 *	poolSize: Total size of the buffer at poolStart. (Not used)
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if one of the parameters is invalid
 *	SII_OS_STATUS_ERR_INVALID_OP if the memory block pool cannot be created
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsBlockPoolCreate(struct SiiOsBlockPool **pRetPool,
					const char *pName, uint32_t blockSize,
					void *poolStart, uint32_t poolSize)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct kmem_cache *cache = NULL;

	dbg("%s", (NULL != pName) ? pName : "");

	if (NULL == pRetPool) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}
	*pRetPool = NULL;

	if ((0 == blockSize) || (NULL == poolStart) || (0 == poolSize)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	cache = kmem_cache_create(pName, blockSize, 0, SLAB_HWCACHE_ALIGN,
					NULL);
	if (NULL == cache) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	*pRetPool = (struct SiiOsBlockPool *)mempool_create_slab_pool(1, cache);
	if (NULL == *pRetPool) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

done:
	return retval;
}

/*
 * Delete a previously allocated memory block pool. The user-supplied buffer for
 * the memory block pool is not freed.
 *
 * Parameters:
 *	pPool: Memory block pool structure.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_HANDLE if pPool is invalid
 *	SII_OS_STATUS_ERR_INVALID_OP if the memory block pool cannot be deleted
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsBlockPoolDelete(struct SiiOsBlockPool *pPool)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	if (NULL == pPool) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	dbg("");

	mempool_destroy(&pPool->pool);

done:
	return retval;
}

/*
 * Allocate a memory block from the memory block pool and return a pointer
 * to the block.
 *
 * Parameters:
 *	pPool: Memory block pool structure.
 *	pRetBlock: Returned memory block or NULL on failure.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_HANDLE if pPool is invalid
 *	SII_OS_STATUS_ERR_INVALID_OP if a memory block cannot be allocated
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsBlockAllocate(struct SiiOsBlockPool *pPool,
					void **pRetBlock)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	if (NULL == pRetBlock) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}
	*pRetBlock = NULL;

	if (NULL == pPool) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	dbg("");

	*pRetBlock = mempool_alloc(&pPool->pool, GFP_KERNEL);
	if (NULL == *pRetBlock) {
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

done:
	return retval;
}

/*
 * Release a memory block back to the memory block pool.
 *
 * Parameters:
 *	pPool: Memory block pool structure.
 *	pBlock: Memory block to release.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_HANDLE if pPool or pBlock is invalid
 *	SII_OS_STATUS_ERR_INVALID_OP if a memory block cannot be released
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsBlockRelease(struct SiiOsBlockPool *pPool,
					void *pBlock)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	if (NULL == pPool) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	dbg("");

	mempool_free(pBlock, &pPool->pool);

done:
	return retval;
}

/*
 * Create a memory byte pool using the supplied memory buffer.
 *
 * Parameters:
 *	pRetPool: Returned memory byte pool structure or NULL on failure.
 *	pName: Optional name for the memory byte pool.
 *	poolStart: User-supplied buffer for the memory byte pool.
 *	poolSize: Total size of the buffer at poolStart.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if one of the parameters is invalid
 *	SII_OS_STATUS_ERR_INVALID_OP if the memory byte pool cannot be created
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsBytePoolCreate(struct SiiOsBytePool **pRetPool,
					const char *pName, void *poolStart,
					uint32_t poolSize)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_INVALID_OP;

	dbg("%s", (NULL != pName) ? pName : "");

	if (NULL == pRetPool) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}
	*pRetPool = NULL;

	/* Cannot be implemented in linux kernel */

done:
	return retval;
}

/*
 * Delete a previously allocated memory byte pool. The user-supplied buffer for
 * the memory byte pool is not freed.
 *
 * Parameters:
 *	pPool: Memory byte pool structure.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_HANDLE if pPool is invalid
 *	SII_OS_STATUS_ERR_INVALID_OP if the memory byte pool cannot be deleted
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsBytePoolDelete(struct SiiOsBytePool *pPool)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_INVALID_OP;

	dbg("");

	/* Cannot be implemented in linux kernel */

	return retval;
}

/*
 * Allocate a memory buffer from the memory byte pool and return a pointer
 * to the buffer.
 *
 * Parameters:
 *	pPool: Memory byte pool structure.
 *	pRetByte: Returned memory buffer or NULL on failure.
 *	size: Size of the memory buffer to allocate in bytes.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_HANDLE if pPool is invalid
 *	SII_OS_STATUS_ERR_INVALID_OP if a memory buffer cannot be allocated
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsByteAllocate(struct SiiOsBytePool *pPool,
					void **pRetByte, uint32_t size)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_INVALID_OP;

	dbg("");

	/* Cannot be implemented in linux kernel */

	return retval;
}

/*
 * Release a memory buffer back to the memory byte pool.
 *
 * Parameters:
 *	pPool: Memory byte pool structure.
 *	pByte: Memory buffer to release.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_HANDLE if pPool or pByte is invalid
 *	SII_OS_STATUS_ERR_INVALID_OP if the memory buffer cannot be released
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsByteRelease(struct SiiOsBytePool *pPool, void *pByte)
{
	enum sii_os_status retval = SII_OS_STATUS_ERR_INVALID_OP;

	dbg("");

	/* Cannot be implemented in linux kernel */

	return retval;
}

/*
 * Add the listed debug channels to the list of legal debug channels. Channel
 * values that have not been added will be ignored in all other SiiOsDebugXxx
 * functions. Debug output for all channels is disabled by default. For
 * non-debug builds this function does nothing except check parameters.
 *
 * Parameters:
 *	numChannels: Number of debug channels in the supplied channel list.
 *	pChannelList: List of debug channels.
 *
 * Return value:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status SiiOsDebugChannelAdd(uint32_t numChannels,
					struct SiiOsDebugChannel *pChannelList)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == pChannelList) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}
	if (numChannels > maxDbgChannels) {
		err("Too many channels in list\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

#ifdef SII_OSAL_DEBUG_PRINT
	/* TBD */
#endif /* SII_OSAL_DEBUG_PRINT */

done:
	return retval;
}

#ifdef SII_OSAL_DEBUG_PRINT
/*
 * Enable debug messages for the specified channel. If the channel has not been
 * added with SiiOsDebugChannelAdd, nothing happens. For non-debug builds this
 * function does nothing.
 *
 * Parameters:
 *	channel: Channel to be enabled.
 */
void SiiOsDebugChannelEnable(uint32_t channel)
{
	/* TBD */
}

/*
 * Disable debug messages for the specified channel. If the channel has not been
 * added with SiiOsDebugChannelAdd, nothing happens. For non-debug builds this
 * function does nothing.
 *
 * Parameters:
 *	channel: Channel to be disabled.
 */
void SiiOsDebugChannelDisable(uint32_t channel)
{
	/* TBD */
}

/*
 * Query the enabled/disabled status of the specified debug channel.
 *
 * Parameters:
 *	channel: Debug channel to query the status of.
 *
 * Return value:
 *	'true' if debug channel is enabled
 *	'false' if debug channel is disabled
 */
bool SiiOsDebugChannelIsEnabled(uint32_t channel)
{
	/* TBD */
	return false;
}

/* strip the leading path if the given path is absolute */
static const char *simple_file_name(const char *path)
{
	const char *file_name;

	if (NULL == path)
		return path;

	if ('/' != *path)
		return path;

	file_name = strrchr(path, '/');
	if (NULL == file_name)
		return path;

	return file_name + 1;
}

/*
 * Output a debug message if the debug channel is enabled. Usually used with the
 * SII_DEBUG_PRINT macro.
 *
 * Parameters:
 *	pFileName: File name and extension where current code resides.
 *	lineNumber: Line number of current code within the resident file.
 *	channel: Debug output channel.
 *	pFormat: 'printf'-compliant format specification.
 */
void SiiOsDebugPrint(const char *pFileName, uint32_t lineNumber,
			uint32_t channel, const char *pFormat, ...)
{
	va_list args;
	struct va_format vaf;
	char dbg_fmt[] = KERN_DEBUG "%s(%d): %pV";

	va_start(args, pFormat);
	vaf.fmt = pFormat;
	vaf.va = &args;
#if 0
	if (pFormat[0] == '<' && pFormat[2] == '>') {
		memcpy(dbg_fmt, pFormat, 3);
		vaf.fmt = pFormat + 3;
	}
#endif
	printk(dbg_fmt, simple_file_name(pFileName), lineNumber, &vaf);
	va_end(args);
}

/*
 * Output a debug message if the debug channel is enabled. Usually used with the
 * SII_DEBUG_PRINT macro.
 *
 * Parameters:
 *	channel: Debug output channel.
 *	pFormat: 'printf'-compliant format specification.
 */
void SiiOsDebugPrintShort(uint32_t channel, const char *pFormat, ...)
{
	va_list args;
	struct va_format vaf;
	char dbg_fmt[] = KERN_DEBUG "%pV";

	va_start(args, pFormat);
	vaf.fmt = pFormat;
	vaf.va = &args;
#if 0
	if (pFormat[0] == '<' && pFormat[2] == '>') {
		memcpy(dbg_fmt, pFormat, 3);
		vaf.fmt = pFormat + 3;
	}
#endif
	printk(dbg_fmt, &vaf);
	va_end(args);
}

/*
 * Output a simple debug message if the debug channel is enabled. Usually used
 * with the SII_DEBUG_PRINT_SIMPLE macro.
 *
 * Parameters:
 *	channel: Debug output channel.
 *	pFormat: 'printf'-compliant format specification.
 */
void SiiOsDebugPrintSimple(uint32_t channel, const char *pFormat, ...)
{
	va_list args;
	struct va_format vaf;
	char dbg_fmt[] = KERN_DEBUG "%pV";

	va_start(args, pFormat);
	vaf.fmt = pFormat;
	vaf.va = &args;
#if 0
	if (pFormat[0] == '<' && pFormat[2] == '>') {
		memcpy(dbg_fmt, pFormat, 3);
		vaf.fmt = pFormat + 3;
	}
#endif
	printk(dbg_fmt, &vaf);
	va_end(args);
}

/*
 * Set the format of the debug messages.
 *
 * Parameters:
 *	flags: Bit mask of message format flags. The flags are:
 *	  SII_OS_DEBUG_FORMAT_SIMPLE - default simple message (nothing added)
 *	  SII_OS_DEBUG_FORMAT_FILEINFO - filename and line number added
 *	  SII_OS_DEBUG_FORMAT_CHANNEL - channel name added
 *	  SII_OS_DEBUG_FORMAT_TIMESTAMP - time stamp in milliseconds added
 */
void SiiOsDebugConfig(uint16_t flags)
{
	/* TBD */
}

/*
 * Default assertion handler called by the SII_OS_DEBUG_ASSERT macro if no other
 * assertion handler has been supplied with a call to
 * SiiOsDebugInstallAssertHandler().
 *
 * If the assertion condition evaluates to non-zero, then nothing happens.
 * If the assertion condition evaluates to 0, print a debug message and
 * terminate the program.
 *
 * Parameters:
 *	pFileName: File name and extension where current code resides.
 *	lineNumber: Line number of current code within the resident file.
 *	expressionEvaluation: Numerical value of evaluated assertion condition.
 *		!0 - assertion condition OK
 *		0 - assertion condition fail
 *	pConditionText: Assertion condition as text string.
 */
void SiiOsDebugAssert(const char *pFileName, uint32_t lineNumber,
		uint32_t expressionEvaluation, const char *pConditionText)
{
	if (!expressionEvaluation) {
		pr_crit("%s(%d): %s=%d, module exiting",
			simple_file_name(pFileName), lineNumber,
			pConditionText, expressionEvaluation);
		cleanup_and_exit();
	}
}

/*
 * Install an assertion handler to use instead of the default assertion handler.
 *
 * Parameters:
 *	pHandler: Assertion handler routine.
 *	pArg: Data passed to the assertion handler.
 */
void SiiOsDebugInstallAssertHandler(void (*pHandler)(void *pArg), void *pArg)
{
	/* TBD */
}
#else /* SII_OSAL_DEBUG_PRINT */
void SiiOsDebugChannelEnable(uint32_t channel) {}
void SiiOsDebugChannelDisable(uint32_t channel) {}
bool SiiOsDebugChannelIsEnabled(uint32_t channel) { return true; }
void SiiOsDebugPrint(const char *pFileName, uint32_t lineNumber,
			uint32_t channel, const char *pFormat, ...) {}
void SiiOsDebugPrintShort(uint32_t channel, const char *pFormat, ...) {}
void SiiOsDebugPrintSimple(uint32_t channel, const char *pFormat, ...) {}
void SiiOsDebugConfig(uint16_t flags) {}
void SiiOsDebugAssert(const char *pFileName, uint32_t lineNumber,
		uint32_t expressionEvaluation, const char *pConditionText) {}
void SiiOsDebugInstallAssertHandler(void (*pHandler)(void *pArg), void *pArg) {}
#endif /* SII_OSAL_DEBUG_PRINT */

/*
 * Create the Non-Volatile Storage file and fill it with 0xff.
 */
static struct file *create_nv_storage(void)
{
	struct file *filp = NULL;
	uint8_t *buf = NULL;
	int buf_size = NV_STORAGE_SIZE;
	int result = -1;

	dbg("");

	buf = SiiOsAlloc("NVStorage", buf_size, 0);
	if (NULL == buf) {
		err("Failed to allocate buffer of %u bytes\n", buf_size);
		goto done;
	}

	memset(buf, 0xff, buf_size);

	filp = filp_open(configdata_file, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IROTH);
	if (IS_ERR(filp)) {
		err("Could not create NVS file %s\n", configdata_file);
		goto done;
	}

	if ((NULL == filp) ||
	    (NULL == filp->f_op) ||
	    (NULL == filp->f_op->write)) {
		warn("Cannot write to NVS file\n");
	} else {
		mm_segment_t oldfs = get_fs();

		set_fs(KERNEL_DS);

		filp->f_pos = 0;
		result = filp->f_op->write(filp,
					(unsigned char __user __force *)buf,
					buf_size, &filp->f_pos);
		if (result < 0) {
			err("NVS file write failed: 0x%x\n", result);
			goto file_done;
		}

file_done:
		set_fs(oldfs);
	}

done:
	SiiOsFree(buf);
	return filp;
}

/*
 * Open the Non-Volatile Storage file (create if necessary) and write the
 * provided data blob to the file at the provided offset.
 *
 * Parameters:
 *	pData: Data blob to write to the file.
 *	size: Size of the data blob to write in bytes.
 *	offset: Offset in bytes from the start of the file.
 *
 * Return value:
 *	number of bytes written to the file on success (may be less than size)
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
int write_to_nv_storage(const void *pData, uint16_t size, uint32_t offset)
{
	int retval = 0;
	uint16_t max_size = NV_STORAGE_SIZE;
	struct file *filp = NULL;
	int result = -1;

	dbg("");

	if (NULL == pData) {
		err("Invalid parameter\n");
		retval = (int)SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* Check for nothing to write */
	if (!size)
		goto done;

	max_size = (NV_STORAGE_SIZE < offset) ? 0 : NV_STORAGE_SIZE - offset;
	if (!max_size) {
		err("Attempt to write past end of file\n");
		retval = (int)SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (max_size < size) {
		size = max_size;
		warn("Write past end of file (bufsize=%u, maxsize=%u)\n",
		     size, max_size);
	}

	filp = filp_open(configdata_file, O_WRONLY, 0);
	if (IS_ERR(filp) || (NULL == filp)) {
		if (-ENOENT != PTR_ERR(filp)) {
			err("Could not open existing NVS file %s\n",
			    configdata_file);
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		/* If file cannot be opened because it does not exist,
		 * then create it first. */
		filp = create_nv_storage();
		if (IS_ERR(filp) || (NULL == filp)) {
			err("Could not create missing NVS file\n");
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto done;
		}
	}

	if ((NULL == filp->f_op) ||
	    (NULL == filp->f_op->write) ||
	    (NULL == filp->f_op->llseek)) {
		warn("Cannot write/seek to NVS file\n");
	} else {
		mm_segment_t oldfs = get_fs();

		set_fs(KERNEL_DS);

		result = filp->f_op->llseek(filp, offset, SEEK_SET);
		if (result < 0) {
			err("NVS file seek failed: 0x%x\n", result);
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto file_done;
		}

		result = filp->f_op->write(filp,
				(const unsigned char __user __force *)pData,
				size, &filp->f_pos);
		if (result < 0) {
			err("NVS file write failed: 0x%x\n", result);
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto file_done;
		}

file_done:
		set_fs(oldfs);
	}

	if (filp_close(filp, NULL))
		err("Error closing NVS file %s\n", configdata_file);

	if (retval < 0)
		goto done;

	retval = (int)size;

done:
	return retval;
}

/*
 * Open the Non-Volatile Storage file (create if necessary) and read from
 * the file at the provided offset a data blob of the provided size and return
 * the data in the provided data blob. The user must provide the buffer and is
 * responsible for guaranteeing the buffer can hold all the requested data.
 *
 * Parameters:
 *	pData: Data blob to hold the data read from the file.
 *	size: Size of the data blob to read in bytes.
 *	offset: Offset in bytes from the start of the file.
 *
 * Return value:
 *	number of bytes read from the file on success (may be less than size)
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
int read_from_nv_storage(void *pData, uint16_t size, uint32_t offset)
{
	int retval = 0;
	uint16_t max_size = NV_STORAGE_SIZE;
	struct file *filp = NULL;
	int result = -1;

	dbg("");

	if (NULL == pData) {
		err("Invalid parameter\n");
		retval = (int)SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* Check for nothing to read */
	if (!size)
		goto done;

	memset(pData, 0, size);

	max_size = (NV_STORAGE_SIZE < offset) ? 0 : NV_STORAGE_SIZE - offset;
	if (!max_size) {
		err("Attempt to read past end of file\n");
		retval = (int)SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (max_size < size) {
		size = max_size;
		warn("Read past end of file (bufsize=%u, maxsize=%u)\n",
		     size, max_size);
	}

	filp = filp_open(configdata_file, O_RDONLY, 0);
	if (IS_ERR(filp) || (NULL == filp)) {
		if (-ENOENT != PTR_ERR(filp)) {
			err("Could not open existing NVS file %s\n",
			    configdata_file);
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		/* If file cannot be opened because it does not exist,
		 * then create it first. */
		filp = create_nv_storage();
		if (IS_ERR(filp) || (NULL == filp)) {
			err("Could not create missing NVS file\n");
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto done;
		}
	}

	if ((NULL == filp->f_op) ||
	    (NULL == filp->f_op->read) ||
	    (NULL == filp->f_op->llseek)) {
		warn("Cannot read/seek from NVS file\n");
	} else {
		mm_segment_t oldfs = get_fs();

		set_fs(KERNEL_DS);

		result = filp->f_op->llseek(filp, offset, SEEK_SET);
		if (result < 0) {
			err("NVS file seek failed: 0x%x\n", result);
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto file_done;
		}

		result = filp->f_op->read(filp,
					(unsigned char __user __force *)pData,
					size, &filp->f_pos);
		if (result < 0) {
			err("NVS file read failed: 0x%x\n", result);
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto file_done;
		}

file_done:
		set_fs(oldfs);
	}

	if (filp_close(filp, NULL))
		err("Error closing NVS file %s\n", configdata_file);

	if (retval < 0)
		goto done;

	retval = (int)size;

done:
	return retval;
}


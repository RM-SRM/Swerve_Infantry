/*
**************************************************
    File:		fifo.h
    Purpose:    
    
    Notes:
**************************************************
*/


#ifndef __FIFO_FLOAT_H__
#define __FIFO_FLOAT_H__

#include <string.h>
#include "stdint.h"
/**
* The brief description.
* The detail description.
*/
typedef struct _BYTEQueue_t_t_
{
	float* buf;  			// pointer of queue
	int nInPos;          	// input position of queue
	int nOutPos;         	// output position of queue
	int nDataSize;      	 // data size in the queue
	int nQueueSize;          // size of queue
}BYTEQueue_t;


// 队列初始化
void QueueInit_f(BYTEQueue_t* pQ, float* buf, int bufSize);
// 数据入队
int QueueInPut_f(BYTEQueue_t* pQ, float data);
// 多个数据入队
int QueueInPutBuf_f(BYTEQueue_t* pQ, float* buf, int bufSize);
//memcpy 多数据入列
int QueueInPutBufMemcpy_f(BYTEQueue_t* pQ, float* buf, int bufSize);
// 数据出队
int QueueGetDataSize_f(BYTEQueue_t* pQ);
//memcpy 多数据出列
int QueueOutPutBufMemcpy_f(BYTEQueue_t* pQ, float* pOut, int count);
// 将队列中数据输出至buf，输出1个字节
int QueueOutPut_f(BYTEQueue_t* pQ, float* pOut);
// 获取队列剩余空间
int QueueGetFreeSize_f(BYTEQueue_t* pQ);
// 获取队列中数据个数（字节）
int QueueOutPutToBuf_f(BYTEQueue_t* pQ, float* buf, int count);
// 获取队列指定位置的数值
int QueueGetAt_f(BYTEQueue_t* pQ, int nIndex);
// 从队列头删除deleteCont个数据
void QueueDelete_f(BYTEQueue_t* pQ, int deleteCount);
//去当前入队前x帧的数值
float QueueGetPre_f(BYTEQueue_t* pQ, int nIndex);
//队列里的平均值
float QueueGetAverage_f(BYTEQueue_t *pQ);
//返回data与队列平均值的差分
float QueueGetDiff_f(BYTEQueue_t* pQ, float data);
#endif /* __FIFO_H__ */

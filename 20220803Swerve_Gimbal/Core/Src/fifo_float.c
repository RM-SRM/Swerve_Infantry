/*
**************************************************
*   File:		fifo.c
*   Purpose:    
*
*   Notes:
**************************************************
*/

#include "fifo_float.h"

//循环模式完成FIFO

// 队列初始化
void QueueInit_f(BYTEQueue_t* pQ, float* buf, int bufSize)
{
	if(buf == 0 || bufSize <= 0) return;
	pQ->buf = buf;
	pQ->nDataSize = 0;
	pQ->nInPos = 0;
	pQ->nOutPos = 0;
	pQ->nQueueSize = bufSize;
    memset(buf,0, sizeof(bufSize));
}
                         
// 数据入队
int QueueInPut_f(BYTEQueue_t* pQ, float data)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//未创建
	if(pQ->nDataSize >= pQ->nQueueSize) return -2;//队列满
	if(pQ->nInPos >= pQ->nQueueSize) return -3;

	pQ->buf[pQ->nInPos] = data;
	pQ->nInPos ++;
	if(pQ->nInPos >= pQ->nQueueSize) pQ->nInPos = 0;
	pQ->nDataSize ++;

	return 1;
}
// 多个数据入队
int QueueInPutBuf_f(BYTEQueue_t* pQ, float* buf, int bufSize)
{
	int i=0;
	if(pQ == NULL && pQ->buf == 0) return -1;//未创建
	if(pQ->nDataSize >= pQ->nQueueSize) return -2;//队列满
	if(pQ->nInPos >= pQ->nQueueSize) return -3;

	for(i=0;i<bufSize && QueueGetFreeSize_f(pQ) > 0; i++)
	{
        QueueInPut_f(pQ, buf[i]);
	}
	return 1;
}
/**
* .
* 使用memcpy的多数据入列.注意使用的是循环模式
* @param[in]	FIFO指针，需要拷贝的数据指针，数据长度
* @param[out]	outArgName output argument description.
*/
int QueueInPutBufMemcpy_f(BYTEQueue_t* pQ, float* buf, int bufSize)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//未创建
	if(pQ->nDataSize >= pQ->nQueueSize) return -2;//队列满
	if (pQ->nDataSize+ bufSize >pQ->nQueueSize - 1)  return -4;//加入该组数据后溢出
	if(pQ->nInPos >= pQ->nQueueSize) return -3;

	if(pQ->nInPos + bufSize > pQ->nQueueSize )
	{
		uint16_t left_length = pQ->nQueueSize - pQ->nInPos;
		memcpy(&pQ->buf[pQ->nInPos],buf,left_length);
		pQ->nInPos =0;
		memcpy(&pQ->buf[pQ->nInPos],(buf+left_length),bufSize - left_length);
		pQ->nInPos 			+=bufSize - left_length;
	}
	else
	{
		memcpy(&pQ->buf[pQ->nInPos],buf,bufSize);
		pQ->nInPos        +=bufSize;
		if(pQ->nInPos >= pQ->nQueueSize) pQ->nInPos = 0;
	}
	pQ->nDataSize			+=bufSize;
	return 1;
}
//数据出队
int QueueOutPut_f(BYTEQueue_t* pQ, float* pOut)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//未创建
	if(pQ->nDataSize <= 0) return -2;//队列空
	if(pQ->nInPos >= pQ->nQueueSize) return -3;

	*pOut = pQ->buf[pQ->nOutPos];
	pQ->nOutPos ++;
	if(pQ->nOutPos >= pQ->nQueueSize) pQ->nOutPos = 0;
	pQ->nDataSize --;
	if(pQ->nDataSize < 0) pQ->nDataSize = 0;
	return 1;
}
/**
* .
* 使用memcpy的多数据出列.
* @param[in]	FIFO指针，需要拷贝出的数据指针，数据长度.
* @param[out]	outArgName output argument description.
*/
int QueueOutPutBufMemcpy_f(BYTEQueue_t* pQ, float* pOut, int count)
{
	if(pQ == NULL) return -1;
	if(QueueGetDataSize_f(pQ) < count) return -2;
	if(pQ->nOutPos >= pQ->nQueueSize) return -3;
	//当拷贝数据长度+提取位置超出数组长度时，需要先拷贝到数组满出去 之后再从数据开头的位置拷贝
	if(pQ->nOutPos + count > pQ->nQueueSize )
	{
		uint16_t left_length = pQ->nQueueSize - pQ->nOutPos;
		memcpy(pOut,&pQ->buf[pQ->nOutPos],left_length);
		pQ->nOutPos =0;
		memcpy((pOut+left_length),&pQ->buf[pQ->nOutPos],count - left_length);
		pQ->nOutPos +=count - left_length;
	}
	else
	{
		memcpy(pOut,&pQ->buf[pQ->nOutPos],count);
		pQ->nOutPos        +=count;
		if(pQ->nOutPos == pQ->nQueueSize)
			pQ->nOutPos=0;
	}
	pQ->nDataSize			-=count;
	return 1;
}
//将队列中数据输出至buf，输出count个字节
int QueueOutPutToBuf_f(BYTEQueue_t* pQ, float* buf, int count)
{
	int i=0;
	if(pQ == NULL) return -1;
	if(QueueGetDataSize_f(pQ) < count) return -2;

	for(i=0;i<count;i++) QueueOutPut_f(pQ, buf + i);

	return i;
}
// 获取队列剩余空间
int QueueGetFreeSize_f(BYTEQueue_t* pQ)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//未创建
	return pQ->nQueueSize - pQ->nDataSize;
}
// 获取队列中数据个数（字节）
int QueueGetDataSize_f(BYTEQueue_t* pQ)
{
	if(pQ == NULL) return -1;
	return pQ->nDataSize;
}
// 获取队列指定位置的数值
int QueueGetAt_f(BYTEQueue_t* pQ, int nIndex)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//未创建
	if(pQ->nDataSize <= 0) return -2;//队列空
	if(nIndex >= pQ->nQueueSize || nIndex >= pQ->nDataSize || nIndex < 0) return -3;//超出队列内数据个数大小

	if( (nIndex+pQ->nOutPos) >= pQ->nQueueSize)
	{
		nIndex = (nIndex + pQ->nOutPos) - pQ->nQueueSize ;
		return pQ->buf[nIndex];
	}
	return pQ->buf[nIndex+pQ->nOutPos];
}

// 从队列头删除deleteCont个数据
void QueueDelete_f(BYTEQueue_t* pQ, int deleteCount)
{
	int i=0;
	float tmp;
	for(i=0;i<deleteCount;i++)
	{
        QueueOutPut_f(pQ, &tmp);
	}
}
float QueueGetPre_f(BYTEQueue_t* pQ, int nIndex)
{
    if(pQ == NULL && pQ->buf == 0) return -1;//未创建
    if(pQ->nDataSize <= 0) return -2;//队列空
    if(nIndex >= pQ->nQueueSize || nIndex >= pQ->nDataSize || nIndex < 0) return -3;

    int i = (pQ->nInPos - nIndex + pQ->nQueueSize) / pQ->nQueueSize;
    return pQ->buf[i];
}
float QueueGetAverage_f(BYTEQueue_t *pQ) {
    if (pQ == NULL && pQ->buf == 0) return -1;//未创建
    if (pQ->nDataSize <= 0) return -2;//队列空


    float length, total = 0;
    if (pQ->nDataSize >= pQ->nQueueSize) {
        length = pQ->nQueueSize;
    } else {
        length = pQ->nDataSize;
    }
    for (int i = 0; i < length; ++i) {
        total += pQ->buf[i];
    }
    total -= pQ->buf[pQ->nInPos];
    return total / length;
}
float QueueGetDiff_f(BYTEQueue_t* pQ, float data)
{
    if(pQ == NULL && pQ->buf == 0) return -1;//未创建
    if(pQ->nDataSize >= pQ->nQueueSize) return -2;//队列满
    if(pQ->nInPos >= pQ->nQueueSize) return -3;

    float average = QueueGetAverage_f(pQ);

    pQ->buf[pQ->nInPos] = data;
    pQ->nInPos ++;
    if(pQ->nInPos >= pQ->nQueueSize) pQ->nInPos = 0;
    pQ->nDataSize ++;

    return data - average;
}
/* End of this file */

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


// ���г�ʼ��
void QueueInit_f(BYTEQueue_t* pQ, float* buf, int bufSize);
// �������
int QueueInPut_f(BYTEQueue_t* pQ, float data);
// ����������
int QueueInPutBuf_f(BYTEQueue_t* pQ, float* buf, int bufSize);
//memcpy ����������
int QueueInPutBufMemcpy_f(BYTEQueue_t* pQ, float* buf, int bufSize);
// ���ݳ���
int QueueGetDataSize_f(BYTEQueue_t* pQ);
//memcpy �����ݳ���
int QueueOutPutBufMemcpy_f(BYTEQueue_t* pQ, float* pOut, int count);
// �����������������buf�����1���ֽ�
int QueueOutPut_f(BYTEQueue_t* pQ, float* pOut);
// ��ȡ����ʣ��ռ�
int QueueGetFreeSize_f(BYTEQueue_t* pQ);
// ��ȡ���������ݸ������ֽڣ�
int QueueOutPutToBuf_f(BYTEQueue_t* pQ, float* buf, int count);
// ��ȡ����ָ��λ�õ���ֵ
int QueueGetAt_f(BYTEQueue_t* pQ, int nIndex);
// �Ӷ���ͷɾ��deleteCont������
void QueueDelete_f(BYTEQueue_t* pQ, int deleteCount);
//ȥ��ǰ���ǰx֡����ֵ
float QueueGetPre_f(BYTEQueue_t* pQ, int nIndex);
//�������ƽ��ֵ
float QueueGetAverage_f(BYTEQueue_t *pQ);
//����data�����ƽ��ֵ�Ĳ��
float QueueGetDiff_f(BYTEQueue_t* pQ, float data);
#endif /* __FIFO_H__ */

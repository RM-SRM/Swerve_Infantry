/*
**************************************************
*   File:		fifo.c
*   Purpose:    
*
*   Notes:
**************************************************
*/

#include "fifo_float.h"

//ѭ��ģʽ���FIFO

// ���г�ʼ��
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
                         
// �������
int QueueInPut_f(BYTEQueue_t* pQ, float data)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//δ����
	if(pQ->nDataSize >= pQ->nQueueSize) return -2;//������
	if(pQ->nInPos >= pQ->nQueueSize) return -3;

	pQ->buf[pQ->nInPos] = data;
	pQ->nInPos ++;
	if(pQ->nInPos >= pQ->nQueueSize) pQ->nInPos = 0;
	pQ->nDataSize ++;

	return 1;
}
// ����������
int QueueInPutBuf_f(BYTEQueue_t* pQ, float* buf, int bufSize)
{
	int i=0;
	if(pQ == NULL && pQ->buf == 0) return -1;//δ����
	if(pQ->nDataSize >= pQ->nQueueSize) return -2;//������
	if(pQ->nInPos >= pQ->nQueueSize) return -3;

	for(i=0;i<bufSize && QueueGetFreeSize_f(pQ) > 0; i++)
	{
        QueueInPut_f(pQ, buf[i]);
	}
	return 1;
}
/**
* .
* ʹ��memcpy�Ķ���������.ע��ʹ�õ���ѭ��ģʽ
* @param[in]	FIFOָ�룬��Ҫ����������ָ�룬���ݳ���
* @param[out]	outArgName output argument description.
*/
int QueueInPutBufMemcpy_f(BYTEQueue_t* pQ, float* buf, int bufSize)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//δ����
	if(pQ->nDataSize >= pQ->nQueueSize) return -2;//������
	if (pQ->nDataSize+ bufSize >pQ->nQueueSize - 1)  return -4;//����������ݺ����
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
//���ݳ���
int QueueOutPut_f(BYTEQueue_t* pQ, float* pOut)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//δ����
	if(pQ->nDataSize <= 0) return -2;//���п�
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
* ʹ��memcpy�Ķ����ݳ���.
* @param[in]	FIFOָ�룬��Ҫ������������ָ�룬���ݳ���.
* @param[out]	outArgName output argument description.
*/
int QueueOutPutBufMemcpy_f(BYTEQueue_t* pQ, float* pOut, int count)
{
	if(pQ == NULL) return -1;
	if(QueueGetDataSize_f(pQ) < count) return -2;
	if(pQ->nOutPos >= pQ->nQueueSize) return -3;
	//���������ݳ���+��ȡλ�ó������鳤��ʱ����Ҫ�ȿ�������������ȥ ֮���ٴ����ݿ�ͷ��λ�ÿ���
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
//�����������������buf�����count���ֽ�
int QueueOutPutToBuf_f(BYTEQueue_t* pQ, float* buf, int count)
{
	int i=0;
	if(pQ == NULL) return -1;
	if(QueueGetDataSize_f(pQ) < count) return -2;

	for(i=0;i<count;i++) QueueOutPut_f(pQ, buf + i);

	return i;
}
// ��ȡ����ʣ��ռ�
int QueueGetFreeSize_f(BYTEQueue_t* pQ)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//δ����
	return pQ->nQueueSize - pQ->nDataSize;
}
// ��ȡ���������ݸ������ֽڣ�
int QueueGetDataSize_f(BYTEQueue_t* pQ)
{
	if(pQ == NULL) return -1;
	return pQ->nDataSize;
}
// ��ȡ����ָ��λ�õ���ֵ
int QueueGetAt_f(BYTEQueue_t* pQ, int nIndex)
{
	if(pQ == NULL && pQ->buf == 0) return -1;//δ����
	if(pQ->nDataSize <= 0) return -2;//���п�
	if(nIndex >= pQ->nQueueSize || nIndex >= pQ->nDataSize || nIndex < 0) return -3;//�������������ݸ�����С

	if( (nIndex+pQ->nOutPos) >= pQ->nQueueSize)
	{
		nIndex = (nIndex + pQ->nOutPos) - pQ->nQueueSize ;
		return pQ->buf[nIndex];
	}
	return pQ->buf[nIndex+pQ->nOutPos];
}

// �Ӷ���ͷɾ��deleteCont������
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
    if(pQ == NULL && pQ->buf == 0) return -1;//δ����
    if(pQ->nDataSize <= 0) return -2;//���п�
    if(nIndex >= pQ->nQueueSize || nIndex >= pQ->nDataSize || nIndex < 0) return -3;

    int i = (pQ->nInPos - nIndex + pQ->nQueueSize) / pQ->nQueueSize;
    return pQ->buf[i];
}
float QueueGetAverage_f(BYTEQueue_t *pQ) {
    if (pQ == NULL && pQ->buf == 0) return -1;//δ����
    if (pQ->nDataSize <= 0) return -2;//���п�


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
    if(pQ == NULL && pQ->buf == 0) return -1;//δ����
    if(pQ->nDataSize >= pQ->nQueueSize) return -2;//������
    if(pQ->nInPos >= pQ->nQueueSize) return -3;

    float average = QueueGetAverage_f(pQ);

    pQ->buf[pQ->nInPos] = data;
    pQ->nInPos ++;
    if(pQ->nInPos >= pQ->nQueueSize) pQ->nInPos = 0;
    pQ->nDataSize ++;

    return data - average;
}
/* End of this file */

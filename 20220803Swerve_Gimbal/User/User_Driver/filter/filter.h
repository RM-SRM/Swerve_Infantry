#ifndef __FILTER_H
#define __FILTER_H	 

typedef struct{
	double raw_value;
	double xbuf[5];
	double ybuf[5];
	double filtered_value;
}Filter_t;

extern Filter_t MPUz50Hz,MPUy50Hz,MPUx50Hz;
extern Filter_t PIDOUTPUT50Hz;
extern void Chebyshev50HzLPF(Filter_t *F);
extern void Chebyshev100HzLPF(Filter_t *F);
extern void ControlDesiner(Filter_t *F);
extern void ControlDesiner2(Filter_t *F);
extern void Corrector_Yaw_Speed(Filter_t *F);
extern void Corrector_Yaw_Position(Filter_t *F);

#endif

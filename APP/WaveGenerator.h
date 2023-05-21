/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVEGENERATOR_H
#define __WAVEGENERATOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private defines -----------------------------------------------------------*/
#define DAC_FS			(float)3.3
#define DAC_BITS		12
#define DAC_RESOLUTION	(DAC_FS/((float)pow(2,DAC_BITS)-1.0))
#define DAC_CTE_CONV	(float)1240.909091 /* 2^bits - 1 / FS */
#define OFFSET_UP		0.1
#define NUM_PTS			20.0
#define	SAMPLE_FREQ		50000.0 /* Frequency to DAC sampling */

#define TIM_CLK 120000000.0
#define MAX_SAMPLE_FREQ 50000.0
#define MAX_AMP 3.0
#define MAX_ARR 4294967295
#define SAMPLES_TO_SEND 200
#define INIT_PTOS 40


/* Private typedef -----------------------------------------------------------*/
typedef enum {
	NO_ERROR = (int) 0, /* No error occurred */
	WG_GENERAL_ERROR = (int) 50 /* General or undefined error */
} errorWaveGenerator;



struct WaveGenerator_T {

	void (*Initialice)(void);
	void (*ProcessData)(char *bufIn, uint16_t lenIn, char *bufOut,
			uint16_t *lenOut, void *cb_arg);
	void (*UpdateTestStep)(void);


}extern WG;

#ifdef __cplusplus
}
#endif

#endif /* __WAVEGENERATOR_H */

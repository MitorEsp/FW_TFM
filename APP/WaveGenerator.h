
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVEGENERATOR_H
#define __WAVEGENERATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
	NO_ERROR = (int) 0, /* No error occurred */
	WG_GENERAL_ERROR = (int) 50 /* General or undefined error */
} errorWaveGenerator;

//typedef enum{
//	WG_CMD_IDN		= "IDN?",
//	WG_CMD_ERR		= "ERR?"
//
//}cmdWaveGenarator;

struct WaveGenerator_T {

	void (*ProcessData)(char *bufIn, uint16_t lenIn, char *bufOut,
			uint16_t *lenOut, void *cb_arg);
	void (*UpdateTestStep)(void);
	uint32_t (*getFreq)(void);
	float (*getAmp)(void);

}extern WG;

#ifdef __cplusplus
}
#endif

#endif /* __WAVEGENERATOR_H */

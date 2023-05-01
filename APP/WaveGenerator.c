/* Private includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "WaveGenerator.h"
#include "main.h"

/* Private functions */
static errorWaveGenerator WG_IDN(char *bufOut, uint16_t *lenOut);
static errorWaveGenerator WG_TST(char *bufOut, uint16_t *lenOut, void *cb_arg);
static errorWaveGenerator WG_ABOR(char *bufOut, uint16_t *lenOut);
static errorWaveGenerator WG_INIT(char *bufOut, uint16_t *lenOut);
static void resetOwnVars(void);

/* Private variables */
uint32_t actualFreq; /* Actual frequency test */
float actualAmp; /* Actual amplitude test */
bool flagFrecRetries; /* Indicates if the measurement is out of valid margins */
uint8_t cntFrecRetries; /* Retries counter if the measurements is out of valid margins*/
bool flagEndTest; /* Indicates the end of the test */

static errorWaveGenerator WG_IDN(char *bufOut, uint16_t *lenOut) {

	/* *IDN?

	 The Identification (IDN) query outputs an identifying string.
	 The response will show the following information:

	 <company name>, <model number>, <serial number>, <firmware revision> */

	/* Get unique ID*/
	uint32_t (*uID_1) = (uint32_t*) 0x1FFF7A10;
	uint32_t (*uID_2) = (uint32_t*) 0x1FFF7A14;
	uint32_t (*uID_3) = (uint32_t*) 0x1FFF7A18;

	*lenOut =
			sprintf(bufOut,
					"Company\t= NONE\nPN\t= STM32-F207ZG\nSN\t= %08X%08X%08X\nFW\t= %d.%d",
					(unsigned int) *uID_3, (unsigned int) *uID_2,
					(unsigned int) *uID_1, (int) FW_VERSION,
					(int) FW_SUBVERSION);

	return NO_ERROR;
}

static errorWaveGenerator WG_TST(char *bufOut, uint16_t *lenOut, void *cb_arg) {

	uint16_t data_in = 0, data_out = 2000;
	ptrHWparams ptrHWp;
	memcpy(&ptrHWp, cb_arg, sizeof(ptrHWp));

	/* Set arbitrary voltage to test the loop-back*/
	HAL_DAC_SetValue(ptrHWp.ptrHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, data_out);
	HAL_DAC_Start(ptrHWp.ptrHdac, DAC_CHANNEL_1);

	/* Measure the loop-back */
	HAL_ADC_Start(ptrHWp.ptrHadc1);
	HAL_ADC_PollForConversion(ptrHWp.ptrHadc1, 1);
	data_in = HAL_ADC_GetValue(ptrHWp.ptrHadc1);
	HAL_ADC_Stop(ptrHWp.ptrHadc1);

	/* Stops the stimulus */
	HAL_DAC_SetValue(ptrHWp.ptrHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(ptrHWp.ptrHdac, DAC_CHANNEL_1);
	HAL_DAC_Stop(ptrHWp.ptrHdac, DAC_CHANNEL_1);

	float error = ((float) (data_out - data_in) / (float) data_out) * 100.0;

	*lenOut = sprintf(bufOut, "DAC-ADC loopback has %.3f%% of error", error);

	return NO_ERROR;
}

static errorWaveGenerator WG_ABOR(char *bufOut, uint16_t *lenOut) {

	/*TODO
	 * Kill timers
	 * sut down ADC-DAC*/

	setTestStart(false);

	resetOwnVars();

	*lenOut = sprintf(bufOut, "Test terminated manually");

	return NO_ERROR;
}

static errorWaveGenerator WG_INIT(char *bufOut, uint16_t *lenOut) {

	/*TODO
	 * Init timers
	 * Init down ADC-DAC*/

	resetOwnVars();

	setTestStart(true);

	*lenOut = sprintf(bufOut, "Test initialized");

	return NO_ERROR;
}

void WG_Process_Data(char *bufIn, uint16_t lenIn, char *bufOut,
		uint16_t *lenOut, void *cb_arg) {

	/*
	 * Igual lo que tienes que hacer es traerte todo aca de manera mas estatica.
	 * Es decir, en el conect te traes todos los punteros:
	 * -hdac
	 * -hadc
	 * -htim1
	 * -htimX
	 *
	 * Los almaceno en mi chisme, y ya les tendria?
	 * no lose,
	 *
	 * la cosa es como paso el handler del adc-dac al timer-callback
	 *
	 * */

	/* Provides the basic device information */
	if (strcmp(bufIn + 4, "*IDN?") == 0) {
		WG_IDN(bufOut, lenOut);
	}
	/* Check the loop back ADC-DAC */
	else if (strcmp(bufIn + 4, "*TST?") == 0) {
		WG_TST(bufOut, lenOut, cb_arg);
	}
	/* Stops the test */
	else if (strcmp(bufIn + 4, "ABOR") == 0) {
		WG_ABOR(bufOut, lenOut);
	}
	/* Initiate the test */
	else if (strcmp(bufIn + 4, "INIT") == 0) {
		WG_INIT(bufOut, lenOut);
	}
	/* Default case */
	else {
		*lenOut = sprintf(bufOut, "No valid command has sent");
	}

}

//static uint32_t actualFreq; /* Actual frequency test */
//static float actualAmp; /* Actual amplitude test */
//static bool flagFrecRetries; /* Indicates if the measurement is out of valid margins */
//static uint8_t cntFrecRetries; /* Retries counter if the measurements is out of valid margins*/
//static bool flagEndTest; /* Indicates the end of the test */

static void resetOwnVars(void) {
	actualFreq = 30;
	actualAmp = 3.0;
	flagFrecRetries = false;
	cntFrecRetries = 0;
	flagEndTest = false;

}

void WG_Update_Test_Step(void) {

	/* if it's needed to repeat the frequency for the feedback */
	if (flagFrecRetries) {
		cntFrecRetries++;
	}

	/* Normal step advance or 10 seconds of retries in the same frequency */
	if ((!flagFrecRetries) || (cntFrecRetries > 9)) {

		/* if the previous frequency is just 10KHz the test was end */
		if (actualFreq == 100000) {

			/* Notify the end of the test*/
			flagEndTest = true;

			/* Stops timers callbacks */
			setTestStart(false);
		} else {
			/* Reset retries variables */
			cntFrecRetries = 0;
			flagFrecRetries = false;

			/* Calculate new parameters for new frequency test*/
			actualFreq = (actualFreq * 1.05);

			/* if a external filter limits the amplitude its no need to recalculate here */
#if EXTERNAL_FILTER
			/* Constant Area */
			if (actualFreq < 50) {
				actualAmp = 3.0;
			}
			/* First order system behavior */
			else {
				actualAmp = 3.0 / sqrt(1 + pow(actualFreq / 50.0, 2));
			}
#endif

			/* Fit the last frequency to 10KHz*/
			if (actualFreq > 100000)
				actualFreq = 100000;
		}

	}
}

uint32_t WG_Get_Frequency(void) {
	return actualFreq;
}

float WG_Get_Amplitude(void) {
	return actualAmp;
}

struct WaveGenerator_T WG = {
/* .variablePublica = ; */
.ProcessData = WG_Process_Data, /* */
.UpdateTestStep = WG_Update_Test_Step, /* */
.getFreq = WG_Get_Frequency, /* */
.getAmp = WG_Get_Amplitude, /* */

};

/* Private includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "WaveGenerator.h"
#include "main.h"
#include "lwip.h"

/* Private macros */
#define SAMPLES_PERIOD	20 /* Samples measured per period */
#define	SAMPLE_FREQ		50000 /* Frequency to DAC sampling */

/* Private variables ---------------------------------------------------------*/
uint32_t actualFreq; /* Actual frequency test */
float actualAmp; /* Actual amplitude test */
bool flagFrecRetries; /* Indicates if the measurement is out of valid margins */
uint8_t cntFrecRetries; /* Retries counter if the measurements is out of valid margins*/
bool flagEndTest; /* Indicates the end of the test */
uint16_t ADCprescaler; /* Pre-scaler to measure 20 samples máx. per period */
uint16_t ADCcount; /* Count to divide the ADC sampling frequency */

union {
	uint8_t ii;
	uint64_t safeMem;
} mIdx;/* Index where is stored the data measured */
uint32_t sIdx; /* Sample index of DAC */
float buffSamples[255]; /* Variable where store the samples */

/* Private function prototypes -----------------------------------------------*/
static errorWaveGenerator WG_IDN(char *bufOut, uint16_t *lenOut);
static errorWaveGenerator WG_TST(char *bufOut, uint16_t *lenOut, void *cb_arg);
static errorWaveGenerator WG_ABOR(char *bufOut, uint16_t *lenOut, void *cb_arg);
static errorWaveGenerator WG_INIT(char *bufOut, uint16_t *lenOut/*,void *cb_arg*/);
static errorWaveGenerator WG_UPD(char *bufOut, uint16_t *lenOut);
static errorWaveGenerator WG_KEEP(char *bufOut, uint16_t *lenOut);
static void Reset_Own_Vars(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Process the instruction to provide the identification of uC.
  * @param	none
  * @retval	bufOut pointer that will contain the data to outside the uC.
  * @retval	lenOut length of bufOut.
  */
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
}//WG_IDN

/**
  * @brief  Process the instruction to auto-test the loopback DAC-ADC.
  * @retval	bufOut pointer that will contain the data to outside the uC.
  * @retval	lenOut length of bufOut.
  * @param	cb_arg pointer HW handlers (ADC & DAC)
  */
static errorWaveGenerator WG_TST(char *bufOut, uint16_t *lenOut, void *cb_arg) {

	uint16_t dataIn = 0, dataOut = 2000;
	ptrHWparams ptrHWp;
	memcpy(&ptrHWp, cb_arg, sizeof(ptrHWp));

	/* Set arbitrary voltage to test the loop-back*/
	HAL_DAC_SetValue(ptrHWp.ptrHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dataOut);
	HAL_DAC_Start(ptrHWp.ptrHdac, DAC_CHANNEL_1);

	/* Measure the loop-back */
	HAL_ADC_Start(ptrHWp.ptrHadc1);
	HAL_ADC_PollForConversion(ptrHWp.ptrHadc1, 1);
	dataIn = HAL_ADC_GetValue(ptrHWp.ptrHadc1);
	HAL_ADC_Stop(ptrHWp.ptrHadc1);

	/* Stops the stimulus */
	HAL_DAC_SetValue(ptrHWp.ptrHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(ptrHWp.ptrHdac, DAC_CHANNEL_1);
	HAL_DAC_Stop(ptrHWp.ptrHdac, DAC_CHANNEL_1);

	float error =
			dataOut > dataIn ?
					((float) (dataOut - dataIn) / (float) dataOut) * 100.0 :
					((float) (dataIn - dataOut) / (float) dataOut) * 100.0;

	*lenOut = sprintf(bufOut, "DAC-ADC loopback has %.3f%% of error", error);

	return NO_ERROR;
}//WG_TST

/**
  * @brief  Process the instruction to stop the test in uC.
  * @retval	bufOut pointer that will contain the data to outside the uC.
  * @retval	lenOut length of bufOut.
  * @param	cb_arg pointer HW handlers (ADC & DAC)
  */
static errorWaveGenerator WG_ABOR(char *bufOut, uint16_t *lenOut, void *cb_arg) {

	setTestStart(false);

	ptrHWparams ptrHWp;
	memcpy(&ptrHWp, cb_arg, sizeof(ptrHWp));

	/* Stops the stimulus and the measurements */
	HAL_DAC_SetValue(ptrHWp.ptrHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(ptrHWp.ptrHdac, DAC_CHANNEL_1);
	HAL_DAC_Stop(ptrHWp.ptrHdac, DAC_CHANNEL_1);
//	HAL_ADC_Stop(ptrHWp.ptrHadc1);

	*lenOut =
			flagEndTest ?
					sprintf(bufOut, "Test end") :
					sprintf(bufOut, "Test terminated manually");

	Reset_Own_Vars();

	return NO_ERROR;
}//WG_ABOR

/**
  * @brief  Process the instruction to start the test in uC.
  * @param	none
  * @retval	bufOut pointer that will contain the data to outside the uC.
  * @retval	lenOut length of bufOut.
  */
static errorWaveGenerator WG_INIT(char *bufOut, uint16_t *lenOut/*,void *cb_arg*/) {

	Reset_Own_Vars();

	//*lenOut = sprintf(bufOut, "Test initialized");
	*lenOut = 0;

	setTestStart(true);

	return NO_ERROR;
}//WG_INIT

/**
  * @brief  Process the instruction update data from uC to PC
  * @param	none
  * @retval	bufOut pointer that will contain the data to outside the uC.
  * @retval	lenOut length of bufOut.
  */
static errorWaveGenerator WG_UPD(char *bufOut, uint16_t *lenOut) {

	*lenOut =
			flagEndTest ?
					sprintf(bufOut, "END %lu ", actualFreq) :
					sprintf(bufOut, "RUN %lu ", actualFreq);

	uint8_t auxLen = 0;

//	for (uint16_t findex = 0; findex < sizeof(buffSamples); findex++) {
//		/* THIS MUST BE APPEND */
//		auxLen = sprintf(bufOut, "%.3f ",
//				buffSamples[findex] > 0.0 ? buffSamples[findex] : 0.0);
//		*lenOut += auxLen;
//	}

	return NO_ERROR;
}//WG_UPD

/**
  * @brief  Process the instruction to carry on with the test
  * @param	none
  * @retval	bufOut pointer that will contain the data to outside the uC.
  * @retval	lenOut length of bufOut.
  */
static errorWaveGenerator WG_KEEP(char *bufOut, uint16_t *lenOut) {

	setTestStart(true);

	//*lenOut = sprintf(bufOut, "Test in progress");
	*lenOut = 0;

	return NO_ERROR;
}//WG_KEEP

/**
  * @brief  Process the incoming datagram.
  * @param  bufIn pointer that contains the payload of a incoming datagram.
  * @param	lenIn length of pointer bufIn.
  * @param	cb_arg pointer to arguments provided from UDP callback.
  * @retval	bufOut pointer that will contain the data to outside the uC.
  * @retval	lenOut length of bufOut.
  */
void WG_Process_Data(char *bufIn, uint16_t lenIn, char *bufOut,
		uint16_t *lenOut, void *cb_arg) {

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
		WG_ABOR(bufOut, lenOut, cb_arg);
	}
	/* Initiate the test */
	else if (strcmp(bufIn + 4, "INIT") == 0) {
		WG_INIT(bufOut, lenOut/*, cb_arg*/);
	}
	/* Update info for the PC */
	else if (strcmp(bufIn + 4, "UPD") == 0) {
		WG_UPD(bufOut, lenOut);
	}
	/* Keep the test  */
	else if (strcmp(bufIn + 4, "KEEP") == 0) {
		WG_KEEP(bufOut, lenOut);
	}
	/* Default case */
	else {
		*lenOut = sprintf(bufOut, "No valid command has sent");
	}

}//WG_Process_Data

/**
  * @brief  Reset all in tests used variables.
  * @param  None
  * @retval	None
  */
static void Reset_Own_Vars(void) {
	actualFreq = 30;
	actualAmp = 3.0;
	flagFrecRetries = false;
	cntFrecRetries = 0;
	flagEndTest = false;
	ADCprescaler = 1000000 / (actualFreq * SAMPLES_PERIOD);
	ADCcount = 0;
	mIdx.safeMem = 0;
	sIdx = 0;
	memset(buffSamples, 0, sizeof(buffSamples));
}//Reset_Own_Vars

/**
  * @brief  Update the parameters between frequency test steps.
  * @param  None
  * @retval	None
  */
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

			/* Stops timers call-backs */
			setTestStart(false);
		} else {
			/* Reset retries variables */
			cntFrecRetries = 0;
			flagFrecRetries = false;

			/* Calculate new parameters for new frequency test*/
			actualFreq = (actualFreq * 1.05);

			/* Calculate the new sample frequency pre-scaler */
			ADCprescaler = SAMPLE_FREQ / (actualFreq * SAMPLES_PERIOD);

			/* Reset data stored and index */
			memset(buffSamples, 0, sizeof(buffSamples));
			mIdx.safeMem = 0;
			sIdx = 0; /* The máx index achieved in 10 seconds is 10^7, the máx number of uint32 is ~4*10^9 */

			/* if a external filter limits the amplitude its no need to recalculate here */
#if NOT_EXT_FILTER
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
}//WG_Update_Test_Step

/**
  * @brief  Actual frequency step getter.
  * @param  None
  * @retval	actualFreq Actual frequency step test.
  */
uint32_t WG_Get_Frequency(void) {
	return actualFreq;
}

/**
  * @brief  Amplitude step getter.
  * @param  None
  * @retval	actualAmp Actual Amplitude step test.
  */
float WG_Get_Amplitude(void) {
	return actualAmp;
}

/**
  * @brief  Pre-scaler to ADC sampling getter.
  * @param  None
  * @retval	ADCprescaler Actual ADC pre-scaler step test.
  */
uint16_t WG_Get_ADC_Prescaler(void) {
	return ADCprescaler;
}

/**
  * @brief  Stores the input samples in a array to provides to PC.
  * @param  sample
  * @retval	None
  */
void WG_Store_Sample(float sample) {
	buffSamples[mIdx.ii++] = sample;
}

/**
  * @brief  Index of sample to generate in output (DAC) getter.
  * 		Increments one for the next call.
  * @param  None
  * @retval	sIdx Actual index of output sample.
  */
uint32_t WG_Get_Index_of_Sample(void) {
	return sIdx++;
}

/**
  * @brief  Actual count to pre-scaler ADC sampling getter.
  * @param  None
  * @retval	ADCcount Actual count to pre-scaler ADC sampling.
  */
uint32_t WG_Get_ADC_Sample_Count(void) {
	return ADCcount;
}

/**
  * @brief  Actual count to pre-scaler ADC sampling setter.
  * @param  ADCcount Actual count to pre-scaler ADC sampling.
  * @retval	None
  */
void WG_Set_ADC_Sample_Count(uint32_t setADCcount) {
	ADCcount = setADCcount;
}

/* Pointer provider struct to public functions. */
struct WaveGenerator_T WG = {
.ProcessData = WG_Process_Data, /* */
.UpdateTestStep = WG_Update_Test_Step, /* */
.getFreq = WG_Get_Frequency, /* */
.getAmp = WG_Get_Amplitude, /* */
.getADCPres = WG_Get_ADC_Prescaler, /**/
.storeSample = WG_Store_Sample, /* */
.getIndexofSample = WG_Get_Index_of_Sample, /* */
.getADCSampleCount = WG_Get_ADC_Sample_Count, /* */
.setADCSampleCount = WG_Set_ADC_Sample_Count, /* */

};

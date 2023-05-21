/* Private includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "WaveGenerator.h"
#include "main.h"
#include "lwip.h"

/* Private macros */

/* Private variables ---------------------------------------------------------*/

bool flagFrecRetries; /* Indicates if the measurement is out of valid margins */
uint8_t cntFrecRetries; /* Retries counter if the measurements is out of valid margins*/
bool flagEndTest; /* Indicates the end of the test */

union {
	uint8_t ii;
	uint64_t safeMem;
} mIdx;/* Index where is stored the data measured */
float sIdx; /* Sample index of DAC */
float buffSamples[256]; /* Variable where store the samples */

struct waveGeneratorInfo {
	uint32_t ARR; /* Auto Reload Register for TIM2 */
	uint32_t PSC; /* Pre-scaler for TIM2 */
	float actualFreq; /* TIM2 frequency in function of ARR and PSC */
	uint8_t numPtos; /* Number of points per cycle in initialization, and while testing the number of dataOuts stored*/
	float sampleFreq; /* Sampling DAC-ADC frequency */
	float actualAmp;
	uint16_t dataOut[INIT_PTOS];
};

uint8_t indexDataOut; /* Index to waveGeneratorInfo.dataOut[] */

uint8_t tstStep;

struct waveGeneratorInfo wgD[180]; /* This vector has all step data */

char udpBufOut[20 + (SAMPLES_TO_SEND * 6)]; /* ¡Caution with this length, it depends in the number of samples to send */
uint16_t udpLenOut;

/* Private function prototypes -----------------------------------------------*/
static errorWaveGenerator WG_IDN(char *bufOut, uint16_t *lenOut);
static errorWaveGenerator WG_TST(char *bufOut, uint16_t *lenOut, void *cb_arg);
static errorWaveGenerator WG_ABOR(char *bufOut, uint16_t *lenOut, void *cb_arg);
static errorWaveGenerator WG_INIT(char *bufOut,
		uint16_t *lenOut/*,void *cb_arg*/);
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
} //WG_IDN

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
} //WG_TST

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
} //WG_ABOR

/**
 * @brief  Process the instruction to start the test in uC.
 * @param	none
 * @retval	bufOut pointer that will contain the data to outside the uC.
 * @retval	lenOut length of bufOut.
 */
static errorWaveGenerator WG_INIT(char *bufOut,
		uint16_t *lenOut/*,void *cb_arg*/) {

	Reset_Own_Vars();

	//*lenOut = sprintf(bufOut, "Test initialized");
	*lenOut = 0;

	setTestStart(true);

	return NO_ERROR;
} //WG_INIT

/**
 * @brief  Process the instruction update data from uC to PC
 * @param	none
 * @retval	bufOut pointer that will contain the data to outside the uC.
 * @retval	lenOut length of bufOut.
 */
static errorWaveGenerator WG_UPD(char *bufOut, uint16_t *lenOut) {

	*lenOut =
			flagEndTest ?
					sprintf(bufOut, "END %.3f ", wgD[tstStep].actualFreq) :
					sprintf(bufOut, "RUN %.3f ", wgD[tstStep].actualFreq);

	uint8_t auxLen = 0;

	/* I do not why I can send 200 data and not all buffer (256) */
	for (uint16_t findex = 0; findex < 200; findex++) {
		auxLen = sprintf(bufOut + *lenOut, "%.3f ",
				buffSamples[findex] > 0.0 ? buffSamples[findex] : 0.0);
		*lenOut += auxLen;
	}
//	*lenOut = udpLenOut;
//	memcpy(bufOut,udpBufOut,udpLenOut);
	WG.UpdateTestStep();

	return NO_ERROR;
} //WG_UPD

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
} //WG_KEEP

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

} //WG_Process_Data

void WG_Initialice(void) {

	struct waveGeneratorInfo wgInfo;

	float error, tempFreq;
	int numIterations = 0;
	bool keepCalculate = true;

	/* Initial data */
	wgInfo.actualFreq = 30;
	wgInfo.numPtos = INIT_PTOS;
	wgInfo.actualAmp = MAX_AMP;

	while (keepCalculate) {

		/* If freq is force to 100KHz is the last round of calculus */
		if (wgInfo.actualFreq == 100000)
			keepCalculate = false;

		/* Set the máx sample freq */
		while (MAX_SAMPLE_FREQ < wgInfo.numPtos * wgInfo.actualFreq) {
			/* ensure 6 points min */
			if (wgInfo.numPtos == 6)
				break;
			wgInfo.numPtos--;
		}

		/* Calculate timer parameters ARR and PSC */
		for (wgInfo.ARR = 1; wgInfo.ARR < MAX_ARR; wgInfo.ARR++) {
			wgInfo.PSC = -1
					+ (TIM_CLK
							/ ((wgInfo.numPtos * wgInfo.actualFreq)
									* (wgInfo.ARR + 1)));

			error = wgInfo.PSC - (int) wgInfo.PSC;

			if (error < 0.0005) {
				break;
			}
		}

		/* Re-Calculate the frequency in function of ARR and PSC */
		tempFreq = TIM_CLK
				/ ((wgInfo.numPtos) * (wgInfo.ARR + 1) * (wgInfo.PSC + 1));

		/* The re-calculated freq must be lower than the original freq in order to not overcome 5% step rule*/
		if (tempFreq > wgInfo.actualFreq) {
			wgInfo.PSC++;
			wgInfo.actualFreq = TIM_CLK
					/ ((wgInfo.numPtos) * (wgInfo.ARR + 1) * (wgInfo.PSC + 1));
		}

		/* Re-calculate sampleFreq */
		wgInfo.sampleFreq = wgInfo.actualFreq * wgInfo.numPtos;

		/* if a external filter limits the amplitude its no need to recalculate here */
#if NOT_EXT_FILTER
		/* Re-calcualte amplitude */
		/* Constant Area */
		if (wgInfo.actualFreq < 50.0) {
			wgInfo.actualAmp = 3.0;
		}
		/* First order system behavior */
		else {
			wgInfo.actualAmp = 3.0 / sqrt(1 + pow(wgInfo.actualFreq / 50.0, 2));
		}
#endif

		/* Pre-calculate points to DAC */
		/* If more than 1 cycle fits in dataOut array, use it */
		for (int repeatCycle = 0; repeatCycle < INIT_PTOS / wgInfo.numPtos;
				repeatCycle++) {

			for (int iPto = wgInfo.numPtos * repeatCycle;
					iPto < wgInfo.numPtos * (repeatCycle + 1); iPto++) {
				wgInfo.dataOut[iPto] =
						(DAC_CTE_CONV
								* (OFFSET_UP
										+ ((wgInfo.actualAmp / 2.0)
												* (cos(
														2.0 * M_PI
																* ((float) wgInfo.actualFreq)
																* (iPto)
																/ wgInfo.sampleFreq)
														+ 1.0))));
			}

		}

		/* Number of points pre-stored for each freq */
		wgInfo.numPtos =
				(int) (INIT_PTOS / wgInfo.numPtos) > 1 ?
						(int) (INIT_PTOS / wgInfo.numPtos) * wgInfo.numPtos :
						wgInfo.numPtos;

		/* Store info */
		wgD[numIterations] = wgInfo;

		/* 5% of freq increment */
		wgInfo.actualFreq = wgInfo.actualFreq * 1.05;

		numIterations++;

		/* If the freq exceds 100Khz force 100KHz */
		if (wgInfo.actualFreq > 100000)
			wgInfo.actualFreq = 100000;

	}

}

/**
 * @brief  Reset all in tests used variables.
 * @param  None
 * @retval	None
 */
static void Reset_Own_Vars(void) {

	indexDataOut = 0;
	tstStep = 55;
	flagFrecRetries = false;
	cntFrecRetries = 0;
	flagEndTest = false;
	mIdx.safeMem = 0;
	sIdx = 0;
	memset(buffSamples, 0, sizeof(buffSamples));
} //Reset_Own_Vars

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
		if (wgD[tstStep].actualFreq == 100000) {

			/* Notify the end of the test*/
			flagEndTest = true;

			/* Stops timers call-backs */
			setTestStart(false);
		} else {
			/* Reset retries variables */
			cntFrecRetries = 0;
			flagFrecRetries = false;

			/* Increments the index where restore data step info */
			tstStep++;

			Reconfigure_TIM2();

			/* Calculate the new sample frequency pre-scaler */

			/* Reset data stored and index */
			memset(buffSamples, 0, sizeof(buffSamples));
			mIdx.safeMem = 0;
			sIdx = 0; /* The máx index achieved in 10 seconds is 10^7, the máx number of uint32 is ~4*10^9 */
			indexDataOut = 0;

//			if (tstStep == 59) {
//				BREAKPOINT;
//			}

		}

	}
} //WG_Update_Test_Step

/* Pointer provider struct to public functions. */
struct WaveGenerator_T WG = { /* */
.Initialice = WG_Initialice, /* */
.ProcessData = WG_Process_Data, /* */
.UpdateTestStep = WG_Update_Test_Step, /* */

};

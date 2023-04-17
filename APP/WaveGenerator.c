#include <string.h>
#include <stdio.h>
#include "WaveGenerator.h"
#include "main.h"

static errorWaveGenerator WG_Initialize(char *bufOut, uint16_t *lenOut,
		void *cb_arg);

static errorWaveGenerator WG_Initialize(char *bufOut, uint16_t *lenOut,
		void *cb_arg) {

//	Lo suyo seria estimular por el DAC medir por el ADC y comprobar que esta dentro de un margen
	// Igual el connect con devolver el ID y comprobar la conexion valdría
	// otro boton para chequea el loopback
	uint16_t data_in = 0, data_out = 2000;
	ptrHWparams ptrHWp;
	memcpy(&ptrHWp, cb_arg, sizeof(ptrHWp));

	HAL_DAC_SetValue(ptrHWp.ptrHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, data_out);
	HAL_DAC_Start(ptrHWp.ptrHdac, DAC_CHANNEL_1);

	HAL_ADC_Start(&ptrHWp.ptrHadc1);

	HAL_ADC_PollForConversion(&ptrHWp.ptrHadc1, 1);

	data_in = HAL_ADC_GetValue(&ptrHWp.ptrHadc1);

	HAL_ADC_Stop(&ptrHWp.ptrHadc1);

	HAL_DAC_SetValue(ptrHWp.ptrHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(ptrHWp.ptrHdac, DAC_CHANNEL_1);

	//If there are more than 5% of error notify it
	if ((data_in > data_out + 10) || (data_in < data_out - 10)) {
		*lenOut = sprintf(bufOut, "ID=%X%X%X", *uID_3, *uID_2, *uID_1);

	} else {
		/* Get unique ID*/
		uint32_t (*uID_1) = (uint32_t*) 0x1FFF7A10;
		uint32_t (*uID_2) = (uint32_t*) 0x1FFF7A14;
		uint32_t (*uID_3) = (uint32_t*) 0x1FFF7A18;

		*lenOut = sprintf(bufOut, "ID=%X%X%X", *uID_3, *uID_2, *uID_1);

		return NO_ERROR;
	}
}

void WG_Process_Data(char *bufIn, uint16_t lenIn, char *bufOut,
		uint16_t *lenOut, void *cb_arg) {

	/* Check connect and HW function*/
	if (strcmp(bufIn + 4, "*IDN?") == 0) {
		WG_Initialize(bufOut, lenOut, cb_arg);
	}
	/* Default case */
	else {
		*lenOut = sprintf(bufOut, "No valid command has sent");
	}

}

struct WaveGenerator_T WG = {
/* .variablePublica = ; */
.ProcessData = WG_Process_Data,

};

#include "sensor.h"
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart1;
extern Sensor_t sensor;
/**
  * @brief  Lay du lieu tu cam bien
  * @param 	
  * @retval 
  */
void Sensor_Convertdata(Sensor_t* handle){
		handle->vAcquy = ((float) handle->ADC_Process[0])*3.32/4095;
		handle->vCO = (((float) handle->ADC_Process[1])*(3.32/4095));
		handle->vNO2 = ((float) handle->ADC_Process[2])*3.32/4095;
		handle->vSO2 = ((float) handle->ADC_Process[3])*3.32/4095;
	#ifdef DEBUG
		printf("vCO: %f vNO2: %f vSO2: %f\r\n",sensor.vCO, sensor.vNO2, sensor.vSO2);
	#endif
		for(int i = 0; i < 4; i++){
			handle->ADC_Process[i] = 0;
		}
		handle->ppmCO = 3154.5*(handle->vCO - 1.66);
		handle->ppmNO2 = -71.24*(handle->vNO2 - 1.66);
		handle->ppmSO2 = 294.81*(handle->vSO2 - 1.66);
		handle->pAcquy = (handle->vAcquy/3.3)*100;

	 handle->fAdc = false;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1){
				for( int i = 0; i<4; i++){
			sensor.ADC_Process[i] += sensor.ADC_Raw[i];
		}
		if(sensor.cADC < 7) {sensor.cADC++;}
		else{
			sensor.cADC = 0;
			sensor.fAdc = true;
			for(int j = 0; j < 4; j++){
			sensor.ADC_Process[j] = sensor.ADC_Process[j]>>3;
			}
			#ifdef DEBUG
				printf("ADC[1]: %d ADC[4]: %d ADC[5]: %d ADC[6]: %d\r\n",sensor.ADC_Process[0], sensor.ADC_Process[1], sensor.ADC_Process[2],sensor.ADC_Process[3]);
			#endif
			HAL_ADC_Stop_DMA(hadc);
		}
	}
}
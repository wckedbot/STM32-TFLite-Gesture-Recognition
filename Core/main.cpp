#include "gesture_model.h"
#include "RccConfig.h"
#include "Delay.h"
#include "I2C.h"
#include "main.h"
#include "tensorflow/lite/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include <string.h>
using namespace std;
namespace
{
    tflite::ErrorReporter* error_reporter = nullptr;
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* model_input = nullptr;
    TfLiteTensor* model_output = nullptr;

    // Create an area of memory to use for input, output, and intermediate arrays.
    // Finding the minimum value for your model may require some trial and error.
    constexpr uint32_t kTensorArenaSize = 100 * 1024;
    uint8_t tensor_arena[kTensorArenaSize];
}

UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);

#define NUM_GESTURES 2
#define NUM_SAMPLES 40
#define Threshold 20
char* GESTURES[] = {
		"p",
		"f"
};
int samples = 0;


void handle_output(tflite::ErrorReporter* error_reporter, float x_value, float y_value);
void MPU_Write (uint8_t Address, uint8_t Reg, uint8_t Data)
{
	I2C_Start ();
	I2C_Address (Address);
	I2C_Write (Reg);
	I2C_Write (Data);
	I2C_Stop ();
}

void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
	I2C_Start ();
	I2C_Address (Address);
	I2C_Write (Reg);
	I2C_Start ();  // repeated start
	I2C_Read (Address+0x01, buffer, size);
	I2C_Stop ();
}


#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;


float Ax , Ay, Az, baseAx, baseAy, baseAz, Ax_F, Ay_F, Az_F = 0;

uint8_t check;

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	MPU_Read (MPU6050_ADDR,WHO_AM_I_REG, &check, 1);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		MPU_Write (MPU6050_ADDR, PWR_MGMT_1_REG, Data);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		MPU_Write(MPU6050_ADDR, SMPLRT_DIV_REG, Data);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
		Data = 0x00;
		MPU_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, Data);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s
		Data = 0x00;
		MPU_Write(MPU6050_ADDR, GYRO_CONFIG_REG, Data);
	}

}

void MPU6050_Calibrate (void)
{
	uint8_t Rx_data[6];


	MPU_Read (MPU6050_ADDR, ACCEL_XOUT_H_REG, Rx_data, 6);

	Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
	Accel_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
	Accel_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);



	Ax += (Accel_X_RAW/16384.0);
	Ay += (Accel_Y_RAW/16384.0);
	Az += (Accel_Z_RAW/16384.0);
}

void MPU6050_Read (void)
{
	uint8_t Rx_data[6];


	MPU_Read (MPU6050_ADDR, ACCEL_XOUT_H_REG, Rx_data, 6);

	Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
	Accel_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
	Accel_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);



	Ax = (Accel_X_RAW/16384.0);
	Ay = (Accel_Y_RAW/16384.0);
	Az = (Accel_Z_RAW/16384.0);
}


//void read_data()
//{
//	for(int i = 0; i < 40; i++)
//	{
//		MPU6050_Read();
//		printf("%f",Ax);
//		printf("%s", ",");
//		printf("%f", Ay);
//		printf("%s",",");
//		printf("%f", Az);
//		printf("\n");
//
//	}
//	samples++;
//	printf("-----");
//	printf("\n");
//	printf("%d", samples);
//	printf("%s","----");
//	printf("\n");
//}

int main ()
{
	//SystemClock_Config();
	TIM6Config ();
	I2C_Config ();
	MPU6050_Init ();
	HAL_Init();
	MX_USART2_UART_Init();
	for (int i = 0; i < 10; i++)
	{
		MPU6050_Calibrate();

	}
	baseAx = Ax/10.0;
	baseAy = Ay/10.0;
	baseAz = Az/10.0;


	static tflite::MicroErrorReporter micro_error_reporter;
	  	error_reporter = &micro_error_reporter;

	    model = tflite::GetModel(gesture_model);

	  	if(model->version() != TFLITE_SCHEMA_VERSION)
	  	{
	  		TF_LITE_REPORT_ERROR(error_reporter,
	  	                         "Model provided is schema version %d not equal "
	  	                         "to supported version %d.",
	  	                         model->version(), TFLITE_SCHEMA_VERSION);
	  	    return 0;
	  	}

	  	// This pulls in all the operation implementations we need.
	  	static tflite::ops::micro::AllOpsResolver resolver;

	  	// Build an interpreter to run the model with.
	  	static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
	  	interpreter = &static_interpreter;

	  	// Allocate memory from the tensor_arena for the model's tensors.
	  	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	  	if (allocate_status != kTfLiteOk)
	  	{
	  	    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
	  	    return 0;
	  	}

	  	// Obtain pointers to the model's input and output tensors.
	  	model_input = interpreter->input(0);
	  	model_output = interpreter->output(0);

	while(1){
		samples = 0;
		while(samples < NUM_SAMPLES)
		{
			MPU6050_Read();
			Ax_F = Ax - baseAx;
			Ay_F = Ay - baseAy;
			Az_F = Az - baseAz;
			model_input->data.f[samples * 3 + 0] = (Ax_F + 2) / 4.0;
			model_input->data.f[samples * 3 + 1] = (Ay_F + 2)/ 4.0;
			model_input->data.f[samples * 3 + 2] = (Az_F + 2) / 4.0;
			samples++;
			Delay_ms(10);
		}

		if (samples == NUM_SAMPLES) {
			TfLiteStatus invoke_status = interpreter->Invoke();
			 if (invoke_status != kTfLiteOk)
			 {
				TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed on x_val:");
				return 0;
			 }
			 float out1 = model_output->data.f[0];
			 float out2 = model_output->data.f[1];

			 if (out1 > out2 & out1 > 0.5){
				 handle_output(error_reporter, out1, out2);
				 printf("%s\n", "punch");
			 }
			 else if(out2 > out1 & out2 > 0.9){
				 printf("%s\n", "flex");
				 handle_output(error_reporter, out1, out2);
			 }
			 else
			 {
				 printf("%s\n", "No Gesture");
				 handle_output(error_reporter, out1, out2);
			 }
	}
	}
}

void handle_output(tflite::ErrorReporter* error_reporter, float x_value, float y_value)
{
	// Log the current X and Y values
	TF_LITE_REPORT_ERROR(error_reporter, "x_value: %f, y_value: %f\n", x_value, y_value);
}


static void MX_USART2_UART_Init(void)
{


  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#include <string.h>
#include <stdio.h>
#include "app_x-cube-ai.h"
#include "ai_datatypes_defines.h"

static ai_handle network = AI_HANDLE_NULL;
static ai_buffer ai_input[AI_NETWORK_IN_NUM] = { AI_NETWORK_IN_1 };
static ai_buffer ai_output[AI_NETWORK_OUT_NUM] = { AI_NETWORK_OUT_1 };
AI_ALIGNED(4)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 0 */
    /* add these to the code */
		
    /* end of add these */

    ai_error err;
    err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
		char errs[50];
    if(err.type != AI_ERROR_NONE) {
    	printf("E: AI error - type=%d code=%d\r\n", err.type, err.code);
			sprintf(errs ,"E: AI error - type=%d code=%d\r\n", err.type, err.code);
			BSP_LCD_DisplayStringAt(30,30,(uint8_t*) errs, LEFT_MODE);
    }


    ai_network_report report;
    ai_bool res;
    res = ai_network_get_info(network,&report);
    char input_report[16];
    char output_report[16];


    /* initialize network */
    const ai_network_params params = {
    AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
    AI_NETWORK_DATA_ACTIVATIONS(activations) };
    if (!ai_network_init(network, &params)) {
    err = ai_network_get_error(network);
    /* manage the error */
    if(err.type != AI_ERROR_NONE) {
    	printf("E: AI error - type=%d code=%d\r\n", err.type, err.code);
    }
    }
		//BSP_LCD_DisplayStringAt(0,0,(uint8_t*) "Init succeeded", LEFT_MODE);
    /* USER CODE END 0 */
}


void MX_X_CUBE_AI_Process(const ai_float *in_data, ai_float *out_data,const ai_u16 batch_size)
{
		
    //BSP_LCD_DisplayStringAt(30,30,(uint8_t*) "1234", LEFT_MODE);
    /* USER CODE BEGIN 1 */
    ai_input[0].size = 1;
    ai_input[0].data = AI_HANDLE_PTR(in_data);
    ai_output[0].size = 1;
    ai_output[0].data = AI_HANDLE_PTR(out_data);
		//BSP_LCD_DisplayStringAt(30,50,(uint8_t*) "4567", LEFT_MODE);
    ai_error err;
    ai_int nbatch;
    nbatch = ai_network_run(network, &ai_input[0], &ai_output[0]);
	
	//BSP_LCD_DisplayStringAt(30,70,(uint8_t*) nbatch, LEFT_MODE);
//    err = ai_network_get_error(network);
//    /* manage the error */
//		char errs[50];
//    printf("E: AI error - type=%d code=%d\r\n", err.type, err.code);
//		sprintf(errs ,"E: AI error - type=%d code=%d\r\n", err.type, err.code);
//		BSP_LCD_DisplayStringAt(30,90,(uint8_t*) errs, LEFT_MODE);
		//BSP_LCD_DisplayStringAt(30,110,(uint8_t*) "-7-7-7", LEFT_MODE);
    /* USER CODE END 1 */
}

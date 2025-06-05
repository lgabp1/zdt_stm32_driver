# ZDT stm32 driver - driver_files

Here is provided the source code for the `zdt_stm32_driver` library.

## CAN configuration

[CAN_configuration_tips.md](./CAN_configuration_tips.md) can provide some additionnal tips to set up CAN for a STM32CubeIDE project.

## Library usage

First, please add all the files in [Code/](./Code/) in your project.

### CAN and ZDT handler config
Before anything else, please generate code and confirm which `FDCAN_HandleTypeDef` is used. For example if using FDCAN1 port:
```c
FDCAN_HandleTypeDef hfdcan1;
```

Import `driver_ZDTEmmV50.h` header:
```c
/* USER CODE BEGIN Includes */
#include "driver_ZDTEmmV50.h"
/* USER CODE END Includes */
```

If needed, add a filter for receiving CAN:
```c
int main(void)
{
    /* ... */

	/* USER CODE BEGIN 2 */
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x0000;  // Set to 0 to accept all IDs
	sFilterConfig.FilterID2 = 0x0000;  // Set to 0 to accept all IDs
	sFilterConfig.RxBufferIndex = 0;
	if (CAN_add_filter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
	Error_Handler(); /* Filter configuration Error */
	}
```

Configure ZDT handler instance and initialize it
```c
	ZDT_Handler zdt;
	zdt.hfdcan = &hfdcan1;
	zdt.mot_id = 0x01U;
	zdt.checksum = ZDT_CS0x6B;
	zdt.receive_timeout = 1000;

  /* Init ZDT handler instance and CAN */
	if(ZDT_CAN_setup(&zdt)!= ZDT_OK)
	{
	  Error_Handler();
	}
```

### Usage

Following the setup, one may call any `ZDT_cmd_...` functions or `ZDT_receive_reached_message`. Please note that every such functions return a `ZDT_ReturnCode` which can help debugging issues.

Example code
```c
	retz = ZDT_cmd_motor_enable_control(&zdt, true, false); // Command 0xF3
	HAL_Delay(100);
	while (1)
	{
		retz = ZDT_cmd_position_mode_control(&zdt, ZDT_CCW, 10000, 500, 0, true, false); // Command 0xFD
		HAL_Delay(500);
		retz = ZDT_cmd_immediate_stop(&zdt, false); // Command 0xFE
		HAL_Delay(300);
		retz = ZDT_cmd_position_mode_control(&zdt, ZDT_CW, 10000, 500, 0, true, false); // Command 0xFD
		HAL_Delay(500);
		retz = ZDT_cmd_immediate_stop(&zdt, false); // Command 0xFE
		HAL_Delay(1000);
	}
```
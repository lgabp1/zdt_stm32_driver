/*
 * driver_can.c
 *
 *  Created on: May 28, 2025
 *      Author: LeoG
 */

#include "driver_can.h"

/* Extern variables */
uint8_t CANRxDataBuffer[CAN_RX_DATA_BUFFER_SIZE]; // Size can be reduced to max CAN data frame size, 64 bytes should support all FD CAN traffic
FDCAN_RxHeaderTypeDef CANRxHeaderBuffer;
FDCAN_TxHeaderTypeDef CANTxHeaderTemplate; // Template header, will be used as reference when sending new messages
// volatile bool CANIsThereNewMessage; // Whether a new message was received, triggering the interrupt.
CAN_MessageQueue_t* CANRxQueues; // Queues for corresponding ids.
uint32_t CANRxQueuesCount = 0; // Number of queues (number of ids to queue messages from

/* Function prototypes*/
HAL_StatusTypeDef _CAN_get_data_DLC_value(uint8_t n, uint32_t *dlc_bytes);
HAL_StatusTypeDef _CAN_get_data_length_value(uint32_t dlc_bytes, uint8_t *n);

/*
 * Callback function to receive and store CAN messages in buffers. Can be overridden if need be.
 */
__weak void CAN_receive_callback_fill_buffers(FDCAN_HandleTypeDef *hfdcan)
{
	/* Retrieve Rx messages from RX FIFO0 */
	if (HAL_FDCAN_GetRxMessage(hfdcan, CAN_RX_FIFO, &CANRxHeaderBuffer, CANRxDataBuffer) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
	uint32_t can_id = CANRxHeaderBuffer.Identifier;
	for (int ind = 0; ind<CANRxQueuesCount; ind++) {
		if (can_id == CANRxQueues[ind].can_id) { // If not whitelisted, will be thrown away
			CAN_Message_t new_msg;
			new_msg.header = CANRxHeaderBuffer;
			memcpy(new_msg.data, CANRxDataBuffer, CAN_RX_DATA_BUFFER_SIZE);
			CAN_queue_add(&(CANRxQueues[ind]), new_msg); // Append to queue
			break;
		}
	}
}

/*
 * Sets a new header template for CAN messages.
 */
void CAN_set_header_template(const FDCAN_TxHeaderTypeDef TxHeader)
{
	/* Just copies it */
	CANTxHeaderTemplate = TxHeader;
}

/*
 * Gets current header template for CAN messages.
 */
FDCAN_TxHeaderTypeDef CAN_get_header_template() {
	FDCAN_TxHeaderTypeDef RetHeader = CANTxHeaderTemplate; // copies it
	return RetHeader;
}

/*
 * Sends a CAN message and blocks until the message is sent.
 */
HAL_StatusTypeDef CAN_send(FDCAN_HandleTypeDef *hfdcan, int timeout, uint32_t can_id, bool is_extended_id, uint8_t *TxData, uint8_t size)
{
	HAL_StatusTypeDef ret;
	uint32_t tickstart = HAL_GetTick();

	/* Configure default template */
	CANTxHeaderTemplate.Identifier = can_id;
	if (is_extended_id) {
		CANTxHeaderTemplate.IdType = FDCAN_EXTENDED_ID;
	} else {
		CANTxHeaderTemplate.IdType = FDCAN_STANDARD_ID;
	}
	ret = _CAN_from_SIZE_to_DLC(size, &(CANTxHeaderTemplate.DataLength)); // Get corresponding DataLength
	if(ret != HAL_OK) {return ret;}

	ret = HAL_FDCAN_AddMessageToTxBuffer(hfdcan, &CANTxHeaderTemplate, TxData, CAN_TX_BUFFER_USED); // Add message to Tx buffer
	if(ret != HAL_OK) {return ret;}

	ret = HAL_FDCAN_EnableTxBufferRequest(hfdcan, CAN_TX_BUFFER_USED); // Send Tx buffer message
	if(ret != HAL_OK) {return ret;}

	/* Polling for transmission complete on buffer */
	while(HAL_FDCAN_IsTxBufferMessagePending(hfdcan, CAN_TX_BUFFER_USED) == 1) {
		if ( timeout >= 0 && (HAL_GetTick() - tickstart) > timeout) {
			return HAL_TIMEOUT; // Raise timeout
		}
	}
	return HAL_OK;
}

/*
 * Receives a CAN message from given can_id with an optional timeout.
 */
HAL_StatusTypeDef CAN_receive_from(FDCAN_HandleTypeDef *hfdcan, int timeout, uint32_t can_id, uint8_t *RxData, FDCAN_RxHeaderTypeDef *RxHeader)
{
	HAL_StatusTypeDef ret;
	uint32_t tickstart = HAL_GetTick();
	/* Retrieve corresponding queue */
	CAN_MessageQueue_t *queuep;
	for (int ind = 0; ind<CANRxQueuesCount; ind++) {
		if (can_id == CANRxQueues[ind].can_id) { // If not whitelisted, will be thrown away
			queuep = &(CANRxQueues[ind]);
			break;
		}
	}

	/* Wait for new message */
	while (1) {
		if ( (*queuep).length > 0 ) { // If message pending
			break; // New message flag is raised !
		}
		else if ( timeout >= 0 && (HAL_GetTick() - tickstart) > timeout) {
			return HAL_TIMEOUT; // Raise timeout
		}
	}

	/* Copy buffers to given pointers */
	CAN_Message_t msg;
	ret = CAN_queue_pop(queuep, &msg); // Retrieve message and comsume it
	if (ret) {return HAL_ERROR;}
	*RxHeader = msg.header;
	uint8_t n;
	ret = _CAN_from_DLC_to_SIZE(msg.header.DataLength, &n);
	if (ret || n > CAN_RX_DATA_BUFFER_SIZE) {return HAL_ERROR;}
	for (uint8_t i = 0; i < n; i++) { RxData[i] = msg.data[i]; } // copy
	return HAL_OK;
}

/*
 * Overrides the Callback behavior of HAL library.
 * */
#if 1
#if CAN_RX_FIFO_USED == 0
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifoXITs)
#endif
#if CAN_RX_FIFO_USED == 1
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifoXITs)
#endif
{
	if((RxFifoXITs & CAN_RX_NEW_MESSAGE_INTERRUPT) != RESET) // On new message
	{
		CAN_receive_callback_fill_buffers(hfdcan); // Fill Data and Header buffer
		CAN_receive_callback_user(hfdcan); // Call user callback in all cases

		/* Activate callback again */
		if (HAL_FDCAN_ActivateNotification(hfdcan, CAN_RX_NEW_MESSAGE_INTERRUPT, 0) != HAL_OK)
		{
			Error_Handler(); // Notification Error
		}
	}
}
#endif

/*
 * Wrapper for configuring a CAN filter with HAL_FDCAN_ConfigFilter.
 */
HAL_StatusTypeDef CAN_add_filter(FDCAN_HandleTypeDef *hfdcan, const FDCAN_FilterTypeDef *sFilterConfig)
{
  return HAL_FDCAN_ConfigFilter(hfdcan, sFilterConfig);
}

/*
 * Starts the CAN port and enables the callback notification (wraps HAL_FDCAN_Start and HAL_FDCAN_ActivateNotification). + Adds ids to the whitelist
 */
HAL_StatusTypeDef CAN_start(FDCAN_HandleTypeDef *hfdcan, uint32_t *RxCANIds, uint32_t CANIdsCount)
{
	HAL_StatusTypeDef ret;
	if (CANRxQueuesCount + CANIdsCount < 1) {return 1;} // At least 1 id should be whitelisted !

	if (!(hfdcan->State == HAL_FDCAN_STATE_BUSY)) { // First init
		ret = HAL_FDCAN_Start(hfdcan);
		if (ret != HAL_OK) {
			return ret;
		}

		ret = HAL_FDCAN_ActivateNotification(hfdcan, CAN_RX_NEW_MESSAGE_INTERRUPT, 0); // Activate interrupt
		if (ret != HAL_OK) {
			return ret;
		}
	}

	/* Configure default template */
	CANTxHeaderTemplate.Identifier = 0x0;
	CANTxHeaderTemplate.IdType = FDCAN_EXTENDED_ID;
	CANTxHeaderTemplate.TxFrameType = FDCAN_DATA_FRAME;
	CANTxHeaderTemplate.DataLength = FDCAN_DLC_BYTES_8;
	CANTxHeaderTemplate.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	CANTxHeaderTemplate.BitRateSwitch = FDCAN_BRS_OFF;
	CANTxHeaderTemplate.FDFormat = FDCAN_CLASSIC_CAN;
	CANTxHeaderTemplate.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CANTxHeaderTemplate.MessageMarker = 0;

	/* Configure the CAN id tracking queues */
	CAN_MessageQueue_t* temp = (CAN_MessageQueue_t*)realloc(CANRxQueues, (CANRxQueuesCount + CANIdsCount) * sizeof(CAN_MessageQueue_t));
    if (temp == NULL) {
    	return HAL_ERROR;
    }
    CANRxQueues = temp;
	for (int ind = 0; ind<CANIdsCount; ind++) { // Init new queues
		CAN_queue_init(&(CANRxQueues[CANRxQueuesCount + ind]), RxCANIds[ind]);
	}
	CANRxQueuesCount += CANIdsCount;

	return HAL_OK;
}

/*
 * Retrieves the current protocol status of the CAN port (wrapper HAL_FDCAN_GetProtocolStatus).
 */
HAL_StatusTypeDef CAN_get_status(const FDCAN_HandleTypeDef *hfdcan, FDCAN_ProtocolStatusTypeDef *ProtocolStatus)
{
	return HAL_FDCAN_GetProtocolStatus(hfdcan, ProtocolStatus);
}

/*
 * Given message size (uint8_t), returns corresponding dlc_bytes (FDCAN_data_length_code)
 */
HAL_StatusTypeDef _CAN_from_SIZE_to_DLC(uint8_t  n, uint32_t *dlc_bytes) {
	if      (n == 0)  { *dlc_bytes = FDCAN_DLC_BYTES_0;  }
	else if (n == 1)  { *dlc_bytes = FDCAN_DLC_BYTES_1;  }
	else if (n == 2)  { *dlc_bytes = FDCAN_DLC_BYTES_2;  }
	else if (n == 3)  { *dlc_bytes = FDCAN_DLC_BYTES_3;  }
	else if (n == 4)  { *dlc_bytes = FDCAN_DLC_BYTES_4;  }
	else if (n == 5)  { *dlc_bytes = FDCAN_DLC_BYTES_5;  }
	else if (n == 6)  { *dlc_bytes = FDCAN_DLC_BYTES_6;  }
	else if (n == 7)  { *dlc_bytes = FDCAN_DLC_BYTES_7;  }
	else if (n == 8)  { *dlc_bytes = FDCAN_DLC_BYTES_8;  }
	else if (n <= 12) { *dlc_bytes = FDCAN_DLC_BYTES_12; }
	else if (n <= 16) { *dlc_bytes = FDCAN_DLC_BYTES_16; }
	else if (n <= 20) { *dlc_bytes = FDCAN_DLC_BYTES_20; }
	else if (n <= 24) { *dlc_bytes = FDCAN_DLC_BYTES_24; }
	else if (n <= 32) { *dlc_bytes = FDCAN_DLC_BYTES_32; }
	else if (n <= 48) { *dlc_bytes = FDCAN_DLC_BYTES_48; }
	else if (n <= 64) { *dlc_bytes = FDCAN_DLC_BYTES_64; }
	else {return HAL_ERROR;} // Size not supported
	return HAL_OK;
}

/*
 * Given message dlc_bytes (FDCAN_data_length_code), returns corresponding size (uint8_t)
 */
HAL_StatusTypeDef _CAN_from_DLC_to_SIZE(uint32_t dlc_bytes, uint8_t *n) {
	if      (dlc_bytes == FDCAN_DLC_BYTES_0 )  { *n = 0;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_1 )  { *n = 1;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_2 )  { *n = 2;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_3 )  { *n = 3;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_4 )  { *n = 4;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_5 )  { *n = 5;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_6 )  { *n = 6;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_7 )  { *n = 7;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_8 )  { *n = 8;  }
	else if (dlc_bytes == FDCAN_DLC_BYTES_12)  { *n = 12; }
	else if (dlc_bytes == FDCAN_DLC_BYTES_16)  { *n = 16; }
	else if (dlc_bytes == FDCAN_DLC_BYTES_20)  { *n = 20; }
	else if (dlc_bytes == FDCAN_DLC_BYTES_24)  { *n = 24; }
	else if (dlc_bytes == FDCAN_DLC_BYTES_32)  { *n = 32; }
	else if (dlc_bytes == FDCAN_DLC_BYTES_48)  { *n = 48; }
	else if (dlc_bytes == FDCAN_DLC_BYTES_64)  { *n = 64; }
	else {return HAL_ERROR;} // Size not supported
	return HAL_OK;
}


/* === Message FIFO Queue definition ===  */

/*
 * Initialize the CAN_MessageQueue_t queue object
 */
void CAN_queue_init(CAN_MessageQueue_t* q, uint32_t can_id) {
    q->front = q->rear = NULL;
    q->length = 0U;
    q->can_id = can_id;
}

/*
 * Add a Message to the queue
 */
int CAN_queue_add(CAN_MessageQueue_t* q, CAN_Message_t msg) {
    CAN_MessageNode_t* newNode = (CAN_MessageNode_t*)malloc(sizeof(CAN_MessageNode_t));
    if (newNode == NULL) {
        return 1;
    }

    newNode->content = msg;
    newNode->next = NULL;

    if (q->length == 0) {
        q->front = q->rear = newNode;
    } else {
        q->rear->next = newNode;
        q->rear = newNode;
    }
    q->length++; // Increment size when enqueuing

    if (q->length > CAN_RX_QUEUES_MAX_SIZE) { // Pop oldest message if queue filled
    	CAN_Message_t temp_msg;
    	return CAN_queue_pop(q, &temp_msg);
    }
    return 0;
}

/*
 * Remove a Message from the queue
 */
int CAN_queue_pop(CAN_MessageQueue_t* q, CAN_Message_t *msg) {
    if (q->length == 0) {
        return 1;
    }

    CAN_MessageNode_t* popped_node = q->front;
    CAN_Message_t popped_msg = popped_node->content;
    q->front = q->front->next;

    if (q->front == NULL) {
        q->rear = NULL;
    }

    free(popped_node);
    q->length--; // Decrement size when dequeuing
    *msg = popped_msg;
    return 0;
}

/*
 * Clear queue
 */
int CAN_queue_clear(CAN_MessageQueue_t* q) {
    int ret; CAN_Message_t temp_msg;
    while (q->length > 0) {
        ret = CAN_queue_pop(q, &temp_msg);
        if (ret) {
        	return ret;
        }
    }
    return 0;
}

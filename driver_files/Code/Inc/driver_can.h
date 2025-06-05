/*
 * driver_can.h
 *
 *  Created on: May 28, 2025
 *      Author: LeoG
 */

/*
 * TODO ?
 * void make_header
 * void make_filter
 * timeout send ?
 * Add support for non interrupt
 */

#ifndef INC_DRIVER_CAN_H_
#define INC_DRIVER_CAN_H_

#include <main.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* === User definitions ===*/
#define CAN_RX_DATA_BUFFER_SIZE 8 // Size in bytes.  Size can be reduced to max CAN data frame size, 64 bytes should support all FD CAN traffic
extern uint8_t CANRxDataBuffer[];
#define CAN_RX_FIFO_USED 0 // 0:FIFO 0 is used, 1: FIFO 1 used
#define CAN_RX_BUFFER_USED FDCAN_RX_BUFFER0 // e.g. FDCAN_TX_BUFFER0
#define CAN_TX_BUFFER_USED FDCAN_TX_BUFFER0 // e.g. FDCAN_TX_BUFFER0
#define CAN_RX_QUEUES_MAX_SIZE 10 // Max queue size for received messages
/**
 * @brief Callback function called after CAN_receive_callback_fill_buffers. Override this to define custom callback logic.
 *.
 * @param hfdcan Pointer to an FDCAN_HandleTypeDef structure.
 */
__weak void CAN_receive_callback_user(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Callback function to receive and store CAN messages in buffers. Can be overridden if need be.
 *
 * If the message retrieval is successful, it sets a flag indicating that a new message has been received.
 * In case of a reception error, it calls the error handler defined in main.c.
 *
 * @param hfdcan Pointer to an FDCAN_HandleTypeDef structure.
 */
__weak void CAN_receive_callback_fill_buffers(FDCAN_HandleTypeDef *hfdcan);

/* === Abstraction functions ===*/

/**
 * @brief Sets a new header template for CAN messages.
 *
 * This function copies the provided FDCAN Tx header configuration into
 * the global CANTxHeaderTemplate, which is used as the template for subsequent CAN messages.
 *
 * @param TxHeader The FDCAN_TxHeaderTypeDef structure containing new header configuration
 */
void CAN_set_header_template(const FDCAN_TxHeaderTypeDef TxHeader);

/**
 * @brief Gets current header template for CAN messages.
 *
 * This function returns a copy of CANTxHeaderTemplate, which is used as the template for subsequent CAN messages.
 *
 * @return The FDCAN_TxHeaderTypeDef structure containing header configuration
 */
FDCAN_TxHeaderTypeDef CAN_get_header_template();

/**
 * @brief Sends a CAN message and blocks until the message is sent.
 *
 * This function configures and sends a CAN message using the specified FDCAN handle.
 * It sets up the CAN message header with the given CAN ID and determines if the ID is extended or standard.
 * The function then adds the message to the transmit buffer and enables the transmit buffer request.
 * It blocks until the message transmission is complete.
 *
 * @param hfdcan Pointer to an FDCAN_HandleTypeDef structure.
 * @param can_id The CAN identifier to be used for the message.
 * @param is_extended_id Flag indicating whether the CAN ID is extended (true) or standard (false).
 * @param TxData Pointer to the data buffer containing the message data to be sent.
 * @param size The size of the data buffer in bytes.
 *
 * @return HAL_StatusTypeDef HAL_StatusTypeDef HAL_OK on success, or HAL_ERROR on processing error.
 */
HAL_StatusTypeDef CAN_send(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, bool is_extended_id, uint8_t *TxData, uint8_t size); // Send CAN message

/**
 * @brief Receives a CAN message with an optional timeout.
 *
 * Wait for a new CAN message to be received. If a timeout is specified (non-negative),
 * returns with a timeout error if no message is received within the specified time.
 * Once a message is received, copies the message data and header into the provided buffers.
 * Assumes the provided buffers to have sufficient sizes and been already allocated.
 *
 * @param hfdcan Pointer to an FDCAN_HandleTypeDef structure.
 * @param timeout Maximum wait time in milliseconds; negative values wait indefinitely.
 * @param RxData Buffer to store received message data.
 * @param RxHeader Structure to store the received message header.
 *
 * @return HAL_StatusTypeDef HAL_StatusTypeDef HAL_OK on success, HAL_TIMEOUT on timeout, or HAL_ERROR on processing error.
 */
HAL_StatusTypeDef CAN_receive_from(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, int timeout, uint8_t *RxData, FDCAN_RxHeaderTypeDef *RxHeader); // Receive CAN message

/* === Call wrappers === */

/**
 * @brief Wrapper for configuring a CAN filter with HAL_FDCAN_ConfigFilter.
 *
 * Configures a CAN filter using the provided FDCAN handle and filter configuration.
 *
 * @param hfdcan Pointer to an FDCAN_HandleTypeDef structure.
 * @param sFilterConfig Pointer to an FDCAN_FilterTypeDef structure containing the filter configuration.
 *
 * @return HAL_StatusTypeDef Returns the status of the filter configuration operation.
 */
HAL_StatusTypeDef CAN_add_filter(FDCAN_HandleTypeDef *hfdcan, const FDCAN_FilterTypeDef *sFilterConfig);

/**
 * @brief Starts the CAN port and enables the callback notification (wraps HAL_FDCAN_Start and HAL_FDCAN_ActivateNotification).
 *
 * Initializes the CAN port using the provided handle and activates notifications for new messages.
 * Also configures a default transmission header template.
 *
 * @param hfdcan Pointer to an FDCAN_HandleTypeDef structure.
 * @param RxCANIds Array of ids to track
 * @param CANIdsCount Number of ids to track
 *
 * @return HAL_StatusTypeDef Returns the status of the start operation.
 */
HAL_StatusTypeDef CAN_start(FDCAN_HandleTypeDef *hfdcan, uint32_t *RxCANIds, uint32_t CANIdsCount);

/**
 * @brief Retrieves the current protocol status of the CAN port (wrapper for HAL_FDCAN_GetProtocolStatus).
 *
 * Wrapper for HAL_FDCAN_GetProtocolStatus to get the protocol status of the specified CAN handle.
 *
 * @param hfdcan Pointer to an FDCAN_HandleTypeDef structure.
 * @param ProtocolStatus Pointer to an FDCAN_ProtocolStatusTypeDef structure where the protocol status will be stored.
 *
 * @return HAL_StatusTypeDef Returns the status of the protocol status retrieval operation.
 */
HAL_StatusTypeDef CAN_get_status(const FDCAN_HandleTypeDef *hfdcan, FDCAN_ProtocolStatusTypeDef *ProtocolStatus); // Wrapper for HAL_FDCAN_GetProtocolStatus

/*
 * @brief Given message size (uint8_t), returns corresponding dlc_bytes (FDCAN_data_length_code)
 */
HAL_StatusTypeDef _CAN_from_SIZE_to_DLC(uint8_t n, uint32_t *dlc_bytes);

/*
 * @brief Given message dlc_bytes (FDCAN_data_length_code), returns corresponding size (uint8_t)
 */
HAL_StatusTypeDef _CAN_from_DLC_to_SIZE(uint32_t dlc_bytes, uint8_t *n);

/* === Automated defines === */
#if CAN_RX_FIFO_USED == 0
#define CAN_RX_NEW_MESSAGE_INTERRUPT FDCAN_IT_RX_FIFO0_NEW_MESSAGE
#define CAN_RX_FIFO FDCAN_RX_FIFO0
#endif
#if CAN_RX_FIFO_USED == 1
#define CAN_RX_NEW_MESSAGE_INTERRUPT FDCAN_IT_RX_FIFO1_NEW_MESSAGE
#define CAN_RX_FIFO FDCAN_RX_FIFO1
#endif

/* === Extern variables and prototypes === */
extern FDCAN_RxHeaderTypeDef CANRxHeaderBuffer;
extern FDCAN_TxHeaderTypeDef CANTxHeaderTemplate; // Template header, will be used as reference when sending new messages
volatile extern bool CANIsThereNewMessage; // Whether a new message was received, triggering the interrupt.
extern void Error_Handler(void);


/* === Message FIFO Queue definition ===  */
typedef struct { // Classical CAN Message content representation
	FDCAN_RxHeaderTypeDef header;
    char data[CAN_RX_DATA_BUFFER_SIZE];
} CAN_Message_t;

typedef struct CAN_MessageNode_t { // CAN Message FIFO Queue node
    CAN_Message_t content;
    struct CAN_MessageNode_t* next;
} CAN_MessageNode_t;

typedef struct { // CAN Message FFIFO Queue
    CAN_MessageNode_t* front;
    CAN_MessageNode_t* rear;
    uint32_t length;
    uint32_t can_id; // Corresponding id
} CAN_MessageQueue_t;

extern CAN_MessageQueue_t* CANRxQueues; // Queues for corresponding ids.
extern uint32_t CANRxQueuesCount; // Number of queues (number of ids to queue messages from

/**
 * @brief Initialize the CAN_MessageQueue_t queue object
 *
 * @param q Pointer to an CAN_MessageQueue_t structure to initialize.
 */
void CAN_queue_init(CAN_MessageQueue_t* q, uint32_t can_id);

/**
 * @brief Add a Message to the queue
 *
 * @param q Pointer to an CAN_MessageQueue_t structure.
 * @param msg Message to add.
 *
 * @return 0 on success, 1 on processing error.
 */
int CAN_queue_add(CAN_MessageQueue_t* q, CAN_Message_t msg);

/**
 * @brief Remove a Message from the queue
 *
 * @param q Pointer to an CAN_MessageQueue_t structure.
 * @param msg Pointer to a Message to save poped message to.
 *
 * @return 0 on success, 1 on processing error.
 */
int CAN_queue_pop(CAN_MessageQueue_t* q, CAN_Message_t *msg);


/**
 * @brief Clear queue
 *
 * @param q Pointer to an CAN_MessageQueue_t structure.
 *
 * @return 0 on success, 1 on processing error.
 */
int CAN_queue_clear(CAN_MessageQueue_t* q);


#endif /* INC_DRIVER_CAN_H_ */

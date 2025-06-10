/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device CDC ACM applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "app_threadx.h"

//=============================================================================
// Local macros, variables and constants
//=============================================================================

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t *data;
    uint16_t length;
} UsbTxPacket;

typedef enum {
    USB_CDC_EVENT_Connected = 1 << 0,
} USB_CDC_EVENT_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_BUFFER_SIZE 512

#define USB_TX_BYTE_POOL_SIZE (1024 * 1) // 1 KB pool size
#define USB_TX_QUEUE_SIZE 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;

static uint8_t usb_rx_buffer[USB_BUFFER_SIZE];

// USB TX queues and byte pools
TX_BYTE_POOL usb_tx_byte_pool;
uint8_t usb_tx_byte_pool_memory[USB_TX_BYTE_POOL_SIZE];

TX_QUEUE usb_tx_queue;
UsbTxPacket* usb_tx_queue_buffer[USB_TX_QUEUE_SIZE];

TX_EVENT_FLAGS_GROUP usb_cdc_events;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

//=============================================================================
// Local function prototypes
//=============================================================================


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//=============================================================================
// Implementation
//=============================================================================

UINT USB_CDC_init() {
    UINT ret;

    // Create byte pool for outgoing USB messages
    ret = tx_byte_pool_create(&usb_tx_byte_pool, "USB TX byte Pool",
        usb_tx_byte_pool_memory, USB_TX_BYTE_POOL_SIZE);

    if(ret != TX_SUCCESS) {
        printf("Failed to create usb tx byte pool, error=%d\n", ret);
        return ret;
    }

    // Create queue for outgoing USB messages
    ret = tx_queue_create(&usb_tx_queue, "USB TX Queue",
        sizeof(UsbTxPacket*) / sizeof(ULONG), usb_tx_queue_buffer,
        sizeof(usb_tx_queue_buffer));

    if(ret != TX_SUCCESS) {
        printf("Failed to create usb tx queue, error=%d\n", ret);
        return ret;
    }

    // create semaphore for connection handling
    ret = tx_event_flags_create(&usb_cdc_events, "USB CDC events");
    if(ret != TX_SUCCESS) {
        printf("Failed to create usb cdc events, error=%d\n", ret);
        return ret;
    }

    return ret;
}

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */

  // Set cdc instance
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;

  // Set TX event to start threads
  tx_event_flags_set(&usb_cdc_events, USB_CDC_EVENT_Connected, TX_OR);

  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */

  HAL_GPIO_WritePin(TP_A2_GPIO_Port, TP_A2_Pin, GPIO_PIN_SET);

  // Clear CDC instance
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  cdc_acm = UX_NULL;

  // Clear flag by reading it
  ULONG eventFlags;
  tx_event_flags_get(&usb_cdc_events, USB_CDC_EVENT_Connected, TX_OR, &eventFlags, TX_NO_WAIT);

  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

VOID usbx_cdc_read_thread_entry(ULONG thread_input)
{
    /* Local Variables */
    ULONG actual_length;
    ULONG eventFlags;
    UX_SLAVE_DEVICE *device;
    device = &_ux_system_slave->ux_system_slave_device;
    UINT status = UX_SUCCESS;

    /* Infinite Loop */
    while(1)
    {
        // Wait for connection
        tx_event_flags_get(&usb_cdc_events, USB_CDC_EVENT_Connected, TX_OR, &eventFlags, TX_WAIT_FOREVER);

        while ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
        {
            // read bytes from USB
            status = ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)usb_rx_buffer, USB_BUFFER_SIZE / 2, &actual_length);

            if(status == UX_SUCCESS && actual_length > 0) {

                // TODO Put data on queue for echo
            	// Usually internal processing here - now just put data on queue for echo

            	UsbTxPacket *packet;
            	uint8_t *packetData;

            	// Allocate space for the packet structure
            	if (tx_byte_allocate(&usb_tx_byte_pool, (VOID **)&packet, sizeof(UsbTxPacket), TX_NO_WAIT) != TX_SUCCESS) {
            		printf("Failed to allocate packet structure\n");
            		continue;
            	}

            	// Allocate space for the data buffer
            	if (tx_byte_allocate(&usb_tx_byte_pool, (VOID **)&packetData, actual_length, TX_NO_WAIT) != TX_SUCCESS) {
            		printf("Failed to allocate packetData\n");

            		// release previous allocation
            		tx_byte_release(packet);
            		continue;
            	}

            	// copy incoming message to new packet
            	memcpy(packetData, usb_rx_buffer, actual_length);
            	packet->data = packetData;
            	packet->length = actual_length;

            	// Add to queue (if full, handle overflow case)
            	if (tx_queue_send(&usb_tx_queue, &packet, TX_NO_WAIT) != TX_SUCCESS) {
            		printf("TX queue full, dropping packet\n");

            		// release previous allocation
            		tx_byte_release(packetData);
            		tx_byte_release(packet);
            		continue;
            	}
            }
            else if (status == UX_ERROR || device->ux_slave_device_state != UX_DEVICE_CONFIGURED)
            {
                // Disconnected or timeout, break and wait for reconnect
                break;
            }
        }

        // Optional short sleep to reduce CPU usage in case of polling disconnect state
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);
    }
}

VOID usbx_cdc_write_thread_entry(ULONG thread_input)
{
    /* Local Variables */
    UX_SLAVE_DEVICE *device;
    device = &_ux_system_slave->ux_system_slave_device;

    ULONG eventFlags;
    UsbTxPacket *packet;
    ULONG bytesWritten;
    UINT status;

    /* Infinite Loop */
    while (1)
    {
        // Wait for connection
        tx_event_flags_get(&usb_cdc_events, USB_CDC_EVENT_Connected, TX_OR, &eventFlags, TX_WAIT_FOREVER);

        // block until a message is ready to be sent
        if (tx_queue_receive(&usb_tx_queue, &packet, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            // packet was successfully fetched from queue
            // Check if device is still properly configured
            if ((device->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm != UX_NULL))
            {
                // Send over USB
                status = ux_device_class_cdc_acm_write(cdc_acm, packet->data, packet->length, &bytesWritten);
                if (status != UX_SUCCESS)
                {
                    printf("Write failed with status: %d\n", status);
                }
            }

            // Clean up transfer - release memory after transmission
            tx_byte_release(packet->data);
            tx_byte_release(packet);
        }
    }
}

/* USER CODE END 1 */

/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USB HID Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <string.h>
#include "USB.h"
#include "USB_HID.h"


/*********************************************************************
*       Information that are used during enumeration
**********************************************************************/
static const USB_DEVICE_INFO usb_deviceInfo = {
    0x058B,                         /* VendorId */
    0x027B,                         /* ProductId */ 
    "Infineon Technologies",        /* VendorName */ 
    "HID mouse sample",             /* ProductName */
    "12345678"                      /* SerialNumber */ 
};


/*********************************************************************
* usb_HIDReport
*  This report is generated according to HID spec and
*  HID Usage Tables specifications.
**********************************************************************/
const U8 usb_HIDReport[] = {
    USB_HID_GLOBAL_USAGE_PAGE + 1,      USB_HID_USAGE_PAGE_GENERIC_DESKTOP,
    USB_HID_LOCAL_USAGE + 1,            USB_HID_USAGE_MOUSE,
    USB_HID_MAIN_COLLECTION + 1,        USB_HID_COLLECTION_APPLICATION,
    USB_HID_LOCAL_USAGE + 1,            USB_HID_USAGE_POINTER,
    USB_HID_MAIN_COLLECTION + 1,        USB_HID_COLLECTION_PHYSICAL,
    USB_HID_GLOBAL_USAGE_PAGE + 1,      USB_HID_USAGE_PAGE_BUTTON,
    USB_HID_LOCAL_USAGE_MINIMUM + 1,    1,
    USB_HID_LOCAL_USAGE_MAXIMUM + 1,    3,
    USB_HID_GLOBAL_LOGICAL_MINIMUM + 1, 0,
    USB_HID_GLOBAL_LOGICAL_MAXIMUM + 1, 1,
    USB_HID_GLOBAL_REPORT_COUNT + 1,    3,
    USB_HID_GLOBAL_REPORT_SIZE + 1,     1,
    USB_HID_MAIN_INPUT + 1,             USB_HID_VARIABLE,  /* 3 button bits */ 
    USB_HID_GLOBAL_REPORT_COUNT + 1,    1,
    USB_HID_GLOBAL_REPORT_SIZE + 1,     5,
    USB_HID_MAIN_INPUT + 1,             USB_HID_CONSTANT,  /* 5 bit padding */ 
    USB_HID_GLOBAL_USAGE_PAGE + 1,      USB_HID_USAGE_PAGE_GENERIC_DESKTOP,
    USB_HID_LOCAL_USAGE + 1,            USB_HID_USAGE_X,
    USB_HID_LOCAL_USAGE + 1,            USB_HID_USAGE_Y,
    USB_HID_GLOBAL_LOGICAL_MINIMUM + 1, (uint8_t) -127,
    USB_HID_GLOBAL_LOGICAL_MAXIMUM + 1, 127,
    USB_HID_GLOBAL_REPORT_SIZE + 1,     8,
    USB_HID_GLOBAL_REPORT_COUNT + 1,    2,
    USB_HID_MAIN_INPUT + 1,             (USB_HID_VARIABLE | USB_HID_RELATIVE),
    USB_HID_MAIN_ENDCOLLECTION,
    USB_HID_MAIN_ENDCOLLECTION
};


/*******************************************************************************
* Macros
********************************************************************************/
#define MOUSE_STEP_SIZE_PLUS ((uint8_t) (20))
#define MOUSE_STEP_SIZE_MINUS ((uint8_t) (-20))
#define CURSOR_STEP_POS     (1U)
#define CURSOR_DELAY        (128U)
#define MOUSE_DATA_LEN      (3U)


/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void usb_add_hid(void);


/*******************************************************************************
* Global Variables
********************************************************************************/
static USB_HID_HANDLE usb_hidContext;


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
*
*  1. It initializes the USB Device block
*  and enumerates as a HID mouse device.
*
*  2. It moves the cursor from left to right
*  and vice-versa indefinitely.
*
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t counter = 0;
    uint8_t mouse_data[MOUSE_DATA_LEN];

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "emUSB Device: HID mouse application "
           "****************** \r\n\n");

    /* Initializes the USB stack */
    USBD_Init();
    
    /* Endpoint Initialization for HID class */
    usb_add_hid();
    
    /* Set device info used in enumeration */
    USBD_SetDeviceInfo(&usb_deviceInfo);
    
    /* Start the USB stack */
    USBD_Start();

    /* Turning the LED on to indicate device is active */
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

    for (;;)
    {
        
        /* Wait for configuration */ 
        while ((USBD_GetState() & USB_STAT_CONFIGURED ) != USB_STAT_CONFIGURED) 
        {
            cyhal_system_delay_ms(50);
        }
        
        counter++;

        memset(mouse_data, 0, MOUSE_DATA_LEN);
      
        if(0 == (counter % CURSOR_DELAY))
        {
            /* Shift left*/
            mouse_data[CURSOR_STEP_POS] = MOUSE_STEP_SIZE_PLUS;   
        }

        if(0 == (counter % (2*CURSOR_DELAY)))
        {
            /* Shift right*/
            mouse_data[CURSOR_STEP_POS] = MOUSE_STEP_SIZE_MINUS;  
        }

        /* Write data to host*/
        USBD_HID_Write(usb_hidContext, &mouse_data[0], MOUSE_DATA_LEN, 0);      
      
        cyhal_system_delay_ms(10);
                    
    }
}


/*********************************************************************
* Function Name: USBD_HID_Mouse_Init 
**********************************************************************       
* Summary
*  Add HID mouse class to USB stack
*
* Parameters:
*  void
*
* Return:
*  void
***********************************************************************/
static void usb_add_hid(void) {
    USB_HID_INIT_DATA InitData;
    USB_ADD_EP_INFO   EPIntIn;

    memset(&InitData, 0, sizeof(InitData));
    memset(&EPIntIn, 0, sizeof(EPIntIn));
    EPIntIn.Flags           = 0;                             /* Flags not used */ 
    EPIntIn.InDir           = USB_DIR_IN;                    /* IN direction (Device to Host) */ 
    EPIntIn.Interval        = 64;                            /* Interval of 8 ms (125 us * 64) */ 
    EPIntIn.MaxPacketSize   = USB_FS_INT_MAX_PACKET_SIZE;    /* Maximum packet size (64 for Interrupt) */ 
    EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         /* Endpoint type - Interrupt */ 

    InitData.EPIn = USBD_AddEPEx(&EPIntIn, NULL, 0);
    InitData.pReport = usb_HIDReport;
    InitData.NumBytesReport = sizeof(usb_HIDReport);
    usb_hidContext = USBD_HID_Add(&InitData);
}

/*********************************************************************/

/* [] END OF FILE */

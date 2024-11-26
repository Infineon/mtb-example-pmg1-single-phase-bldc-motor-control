/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the HPE motor control system Example
*                    for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*******************************************************************************/


/*******************************************************************************
* Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "stdio.h"
#include "motor_control.h"
#include "parameters.h"
#include "string.h"

/*******************************************************************************
* Global variable declaration
 ******************************************************************************/
float vbus_volt;

#if DEBUG_PRINT
char_t str[250];
volatile bool ENTER_LOOP = true; /* Variable used for tracking the print status */
extern uint32_t ref_rpm, rpm;
extern float integral_error;
extern int16_t rpm_error;
extern float pwm_compare;
extern int16_t comm_adv_percent;
extern int16_t current_val;
char_t fault[50] = "\0"; /* Initialize with NULL character to indicate no fault */
#endif /* DEBUG_PRINT */


/*******************************************************************************
* Function Name: main
*******************************************************************************
* Summary:
* The main function performs the following actions:
*    1. Initializes the BSP
*    2. Initializes the the peripherals TCPWM, 8-bit SAR ADC, UART
*    3. Runs the motor control algorithm
*
* Parameters:
*  void
*
* Return:
*  int
*
******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if(result != CY_RSLT_SUCCESS)
    {
        /* Insert error handler here */
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Function to initialize all the peripherals required */
    peripheral_init();

    /* Enable global interrupts */
    __enable_irq();

    /* Enable USBPD charge pump to turn on the DENFETs to allow 3.3V input capability on P2.2 and P2.3 (GPIO_20VT pins) */
    PDSS->pump_ctrl &= ~(PDSS_PUMP_CTRL_PD_PUMP | PDSS_PUMP_CTRL_BYPASS_LV);

    Cy_SysLib_Delay(100u); /* Delay to stabilize the DC input voltage before turning on the H-bridge */

#if DEBUG_PRINT
    /* Start the Timer */
    sw_timer_start(TIMER2, UART_PRINT_INVL, uart_timer_cb); /* UART serial print software-timer */
#endif /* DEBUG_PRINT */

    /* Start the hardware timer */
    Cy_TCPWM_TriggerStart(CYBSP_TIMER_HW, CYBSP_TIMER_MASK);

#if DEBUG_PRINT
    /* Sequence to clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");

    /* Print the code example name */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "PMG1 MCU: Single-phase BLDC motor control");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** \r\n\n");
#endif /* DEBUG_PRINT */

    /* Measure the VBUS voltage using 8-bit ADC */
    vbus_volt = vbus_measure();

    /* Check the VBUS voltage condition required for successful start-up and to prevent motor damage */
    if((vbus_volt > VBUS_LIMIT_MIN) && (vbus_volt < VBUS_LIMIT_MAX))
    {
        /* Start the motor by open-loop commutation for pre-determined time */
        execute_startup();
    }
    else
    {
        /* Assert LOW at Fault line command to host */
        Cy_GPIO_Clr(FAULT_PORT, FAULT_PIN); /* Expected pin config: Open-Drain, Drive LOW with external pull-up resistor */

#if DEBUG_PRINT
        strcpy(fault, "VBUS out of range"); /* Fault type is printed on UART monitor */
#endif /* DEBUG_PRINT */
    }

    for(;;)
    {
#if DEBUG_PRINT
#if PRINT_SYSTEM_DATA
        /* Print system data */
        sprintf(str,"ref_rpm: %d ; rpm: %d ; duty: %d ; VBUS: %d.%d V ; %s\r\n", (int)ref_rpm, (int)rpm, (int)(pwm_compare/(PWM_PERIOD_CNT/100)), (int)vbus_volt, (int)((vbus_volt - (int)vbus_volt)*100), fault);
#elif PRINT_PI_CONTROL_DATA
        /* Print control data */
        sprintf(str,"ref_rpm: %d ; rpm: %d ; rpm_error: %d ; integral_error: %d ; duty: %d\r\n", (int)ref_rpm, (int)rpm, (int)rpm_error, (int)integral_error, (int)(pwm_compare/(PWM_PERIOD_CNT/100)));
#elif PRINT_COMM_ADVANCE_DATA
#if ADAPTIVE_COMM_CTRL_ENABLE
        sprintf(str,"ref_rpm: %d; rpm: %d; commutation-phase-advance: %d; current: %d mA\r\n", (int)ref_rpm, (int)rpm, (int)comm_adv_percent, (int)current_val);
#else
        sprintf(str,"ref_rpm: %d; rpm: %d; commutation-phase-advance: %d\r\n", (int)ref_rpm, (int)rpm, (int)comm_adv_percent);
#endif /* ADAPTIVE_COMM_CTRL_ENABLE */
#endif /* COMM_ADVANCE_DATA */

        /* Prints control/system data through UART at regular time intervals */
        UART_transmit_data();
#endif /* DEBUG_PRINT */

        /* Motor control tasks to be run at regular time intervals
         * This function should be executed with least possible delay in the for-loop */
        control_update();

#if DEBUG_PRINT
        if(ENTER_LOOP)
        {
            /* Print debug message on for loop entry */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Entered for loop \r\n");
            ENTER_LOOP = false;
        }
#endif /* DEBUG_PRINT */
    }
}
/* [] END OF FILE */

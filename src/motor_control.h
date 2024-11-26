/******************************************************************************
* File Name: motor_control.h
*
* Description: This is the header file for all motor control related tasks and functions
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"

/*******************************************************************************
* Variable declaration
*******************************************************************************/
/* Structure to store the run-time parameters of software timer */
typedef struct
{
    bool active;
    uint16_t startTime;
    uint16_t overflow;
    uint32_t duration;    /* time in microseconds */
    void (*callback)(void);
}stc_sw_timer;

/* software timer enumerator */
typedef enum
{
    TIMER0,    /* Commutation timer */
    TIMER1,    /* Control timer */
    TIMER2,    /* UART timer */
    TIMER3     /* PWM deadband timer */
}en_timer_num;

/*******************************************************************************
* Function prototypes
*******************************************************************************/
void peripheral_init(void);
cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void);
void hw_timer_isr(void);
void hall_timer_isr(void);
void hall_sensor_isr(void);
void ref_speed_measure(void);
float vbus_measure(void);
int16_t current_measure(void);
void piControl(void);
void ctrl_timer_cb(void);
void adaptive_ctrl_timer_cb(void);
void control_update(void);
void adaptive_ctrl(void);
void sw_timer_start(uint8_t, uint32_t, void (*callback)(void));
void sw_timer_event_handler(void);
void comm_adv_control(void);
void pwm_deadband_control(void);
void execute_startup(void);
void trigger_fault(void);
void rotor_lock_check_cb(void);
int16_t fir_filter(int16_t input, uint8_t N, int16_t* buff);
void UART_Isr(void);
void UART_transmit_data(void);
void uart_timer_cb(void);
void check_status(char *message, cy_rslt_t status);
/* [] END OF FILE */

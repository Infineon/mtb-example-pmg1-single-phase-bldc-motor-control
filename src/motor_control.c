/******************************************************************************
* File Name: motor_control.c
*
* Description: This is the source file that contains all motor control related tasks and functions
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "motor_control.h"
#include "parameters.h"
#include "stdio.h"
#include "string.h"
#include "inttypes.h"

/*******************************************************************************
* Global variables
*******************************************************************************/
cy_stc_usbpd_context_t USBPD;    /* USBPD context - required for ADC operation */

#if DEBUG_PRINT
cy_stc_scb_uart_context_t CYBSP_UART_context;
extern char_t str[250];
extern char_t fault[50];
#endif /* DEBUG_PRINT */

/* Variables required for commutation advancement */
int16_t current_val = 0;
int16_t adv_offset = 0;
int16_t comm_adv_percent = COMM_ADV_MIN; /* Default initial value of % commutation advancement */

/* Variable to store the VBUS voltage measured using ADC */
extern float vbus_volt;

/* Variables for hall-timer */
volatile uint16_t hall_timer_tc_cnt = 0u;

/* Variables for motor control tasks */
volatile uint8_t closedLoop = 0u;
volatile float pwm_compare = 0;
uint16_t capture_tot = 65535u;
volatile uint16_t hall_fail_cnt = 0;
volatile uint32_t rpm_raw = 0u;
volatile uint32_t rpm = 0u;
volatile uint32_t ref_rpm = REF_RPM_MIN;
volatile int16_t rpm_error = 0u;
volatile float integral_error = 0u;

/* Status flags */
volatile bool drive_dir_high;
volatile bool gl_ctrl_update_flag = 0;
volatile bool gl_uart_print_flag = 0;
volatile uint8_t gl_comm_adv_flag_cnt = 0;

/* Variables for sw-timer */
stc_sw_timer timer[SW_TIMERS_NUM]; /* Array of software timers */

/* Hall input GPIO Interrupt Configuration */
const cy_stc_sysint_t hall_intr_config =
{
    .intrSrc = HALL_IN_IRQ,       /* Source of interrupt signal */
    .intrPriority = 3UL,           /* Interrupt priority is 3*/
};

/* Initialize the interrupt vector table with the timer interrupt handler address and assign priority. */
const cy_stc_sysint_t timer_intr_config =
{
    /*.intrSrc =*/ CYBSP_TIMER_IRQ,    /* Interrupt source is Timer interrupt */
    /*.intrPriority =*/ 2UL            /* Interrupt priority is 2 */
};

/* UART interrupt configuration structure */
cy_stc_sysint_t uart_intr_config =
{
    .intrSrc      = CYBSP_UART_IRQ,
    .intrPriority = 3UL,
};

/*******************************************************************************
* Function Name: peripheral_init
********************************************************************************
* Summary:
*  - Initializes all the peripherals required for motor control operation
*
* Parameters:
* context_ptr
*
* Return:
*  None
*
*******************************************************************************/
void peripheral_init(void)
{
    cy_en_usbpd_status_t usbpdStatus;
    cy_en_tcpwm_status_t tcpwmStatus;

    /* To set data field in USBPD context structure to NULL.
     * Required for uninterrupted USBPD driver initialization */
    memset((void *)&USBPD, 0, sizeof (cy_stc_usbpd_context_t));

    /* Initialize the USBPD driver */
    usbpdStatus = Cy_USBPD_Init(&USBPD, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
                     (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
    if(usbpdStatus != CY_USBPD_STAT_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_USBPD_Init failed with error code", usbpdStatus);
#endif /* DEBUG_PRINT */
       /* Error handling */
        CY_ASSERT(CY_ASSERT_FAILED);
    }

#if DEBUG_PRINT
    /* Hook interrupt service routine and enable interrupt */
    (void) Cy_SysInt_Init(&uart_intr_config, &UART_Isr);
    NVIC_EnableIRQ(uart_intr_config.intrSrc);

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);
#endif /* DEBUG_PRINT */

    /* Initialize Hall sensor GPIO interrupt */
    if (CY_SYSINT_SUCCESS != Cy_SysInt_Init(&hall_intr_config, &hall_sensor_isr))
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Clear any pending interrupt and enable the GPIO interrupt */
    NVIC_ClearPendingIRQ(hall_intr_config.intrSrc);
    NVIC_EnableIRQ(hall_intr_config.intrSrc);


    /* Initialize the counter interrupt */
    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&timer_intr_config, &hw_timer_isr))
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Initialize the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not.
     */
    tcpwmStatus = Cy_TCPWM_Counter_Init(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, &CYBSP_TIMER_config);

    if(tcpwmStatus != CY_TCPWM_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_TCPWM_Counter_Init failed with error code", tcpwmStatus);
#endif /* DEBUG_PRINT */
       /* Error handling */
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /*Enables the counter in the TCPWM block for the Counter operation*/
    Cy_TCPWM_Counter_Enable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);

    /* Enable Interrupt */
    NVIC_EnableIRQ(timer_intr_config.intrSrc);

    /* Set the counter period */
    Cy_TCPWM_Counter_SetPeriod(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, TIMER_PERIOD_CNT);


    /* Initialize PWM using the configuration structure generated using device configurator */
    tcpwmStatus= Cy_TCPWM_PWM_Init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config);

    if(tcpwmStatus != CY_TCPWM_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_TCPWM_Counter_Init failed with error code", tcpwmStatus);
#endif /* DEBUG_PRINT */
       /* Error handling */
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Update the PWM period value by writing the period count value */
    Cy_TCPWM_PWM_SetPeriod0(CYBSP_PWM_HW, CYBSP_PWM_NUM, PWM_PERIOD_CNT);

    /* Enable the TCPWM as PWM */
    Cy_TCPWM_PWM_Enable(CYBSP_PWM_HW, CYBSP_PWM_NUM);

    /* Retrieve the HSIOM connection to GPIO port */
    if(HSIOM_SEL_GPIO == Cy_GPIO_GetHSIOM(REF_RPM_PORT, REF_RPM_NUM))
    {
        /* Connect GPIO to AMUXA */
        Cy_GPIO_SetHSIOM(REF_RPM_PORT, REF_RPM_NUM, HSIOM_SEL_AMUXA);
    }

    /* Enables the PD block and the registers required for ADC operation */
    usbpdStatus = Cy_USBPD_Adc_Init(&USBPD, CY_USBPD_ADC_ID_0);
    if(usbpdStatus != CY_USBPD_STAT_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_USBPD_Adc_Init failed with error code", usbpdStatus);
#endif /* DEBUG_PRINT */
       /* Error handling */
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Select the reference voltage of ADC */
    /* PMG1-S0 device has both VDDD and 2.0 V from RefGen block as the reference voltages */
    usbpdStatus = Cy_USBPD_Adc_SelectVref(&USBPD, CY_USBPD_ADC_ID_0, ADC_VREF);
    if(usbpdStatus != CY_USBPD_STAT_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_USBPD_Adc_SelectVref failed with error code", usbpdStatus);
#endif /* DEBUG_PRINT */
       /* Error handling */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/* Part of USBPD driver initialization */
cy_stc_pd_dpm_config_t* get_dpm_connect_stat()
{
    return NULL;    /* This value is not required here, hence NULL is returned */
}


/*******************************************************************************
* Function Name: execute_startup
********************************************************************************
* Summary:
* Function to trigger start the motor
* Starts the open-loop commutations for pre-determined time
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void execute_startup(void)
{
    /* Handles DIR pin control for for start-up */
    /*  Rising hall edge detection */
    if(Cy_GPIO_Read(HALL_IN_PORT, HALL_IN_PIN) == 1u)
    {
#if !DIR_INVERT
        /* Drive the DIR output pin HIGH for commutation */
        Cy_GPIO_Set(DIR_PORT, DIR_PIN);
#else
        /* Drive the DIR output pin LOW for commutation */
        Cy_GPIO_Clr(DIR_PORT, DIR_PIN);
#endif /* !DIR_INVERT */
    }

    /*  Falling hall edge detection */
    else
    {
#if !DIR_INVERT
        /* Drive the DIR output pin LOW for commutation */
        Cy_GPIO_Clr(DIR_PORT, DIR_PIN);
#else
        /* Drive the DIR output pin HIGH for commutation */
        Cy_GPIO_Set(DIR_PORT, DIR_PIN);
#endif /* !DIR_INVERT */
    }

    /* Update the initial PWM duty cycle by writing the compare value */
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, PWM_STARTUP_COMPARE_CNT);

    /* Start the control PWM */
    Cy_TCPWM_TriggerStart(CYBSP_PWM_HW, CYBSP_PWM_MASK);

    /* Start the software timer to check rotor-lock during startup */
    sw_timer_start(TIMER1, STARTUP_ROTOR_LOCK_INVL, rotor_lock_check_cb);
}


/*******************************************************************************
* Function Name: ctrl_timer_cb
********************************************************************************
* Summary:
* Function to handle the periodic events related to speed control
* Checks the condition of 'motor alive'
* Shuts off the control in case of a hall sensor failure
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ctrl_timer_cb(void)
{
    gl_ctrl_update_flag = 1;

    /* Check if the hall signal is not detected for a certain number of hall cycles as specified by 'HALL_CNT_MOTOR_FAIL' */
    if(hall_fail_cnt < (uint16_t)((HALL_CNT_MOTOR_FAIL * capture_tot)/(float)CTRL_UPDATE_INVL))
    {
        /* Continue closed-loop control routine */
        sw_timer_start(TIMER1, CTRL_UPDATE_INVL, ctrl_timer_cb);
        hall_fail_cnt++; /* Increment the failure count variable; This variable is regularly set to 0 on detecting hall interrupts */
    }
    else /* Assert motor failure/rotor lock */
    {
        /* Fault is detected and appropriate actions are taken */
        trigger_fault();
    }
}

/*******************************************************************************
* Function Name: control_update
********************************************************************************
* Summary:
* Function to handle the periodic events related to speed control
* Check the condition of 'motor alive'
* Shut off the control in case of a hall sensor failure
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void control_update(void)
{
    static uint8_t ovp_debounce_cnt = 0u;
    static uint8_t uvp_debounce_cnt = 0u;

    /* Check timer flag to update the control parameters periodically */
    if(gl_ctrl_update_flag)
    {
        gl_ctrl_update_flag = 0;

#if COMM_ADVANCE_ENABLE
#if ADAPTIVE_COMM_CTRL_ENABLE
        /* Run adaptive control routine */
        adaptive_ctrl();
#endif /* ADAPTIVE_COMM_CTRL_ENABLE */

#if LINEAR_COMM_ADVANCE_ENABLE
        /* Commutation phase advancement calculation based on RPM
         * The below expression create a set of linear relation based on the experimental test results */
        if(rpm < 2500u) /* Minimum RPM below which fixed commutation advancement is to be used; Motor design dependent parameter */
            comm_adv_percent = COMM_ADV_MIN + adv_offset;
        else
            comm_adv_percent = (int8_t)(COMM_ADV_MIN + COMM_VS_RPM_SLOPE * (rpm - 2500) + adv_offset);
#else
        comm_adv_percent = FIXED_COMM_ADVANCE_VALUE;
#endif /* LINEAR_COMM_ADVANCE_ENABLE */

        /* Condition to check the lower limit to prevent unexpected behavior */
        if(comm_adv_percent < COMM_ADV_MIN)
            comm_adv_percent = COMM_ADV_MIN;
#endif /* COMM_ADVANCE_ENABLE */

#if FIXED_PWM_DUTY_ENABLE
        pwm_compare = FIXED_PWM_COMPARE_CNT;
        Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, (uint16_t)pwm_compare);
#else
        /* Measure the reference speed using 8-bit ADC */
        ref_speed_measure();

        /* Function to implement closed-loop speed control */
        /* Control value is updated at regular time intervals specified by configurable 'CTRL_UPDATE_INVL' value */
        piControl();
#endif /* FIXED_PWM_DUTY_ENABLE */

        /* Measure the VBUS voltage using 8-bit ADC */
        vbus_volt = vbus_measure();

        /* Monitor if the VBUS voltage is within safe limit to prevent motor damage */
        if(vbus_volt > VBUS_OVP_LIMIT)
            ovp_debounce_cnt++;
        else if(ovp_debounce_cnt > 0)
            ovp_debounce_cnt--;

        /* Monitor if the VBUS voltage is within safe limit */
        if(vbus_volt < VBUS_UVP_LIMIT)
            uvp_debounce_cnt++;
        else if(uvp_debounce_cnt > 0)
            uvp_debounce_cnt--;

        /* OVP/UVP debounce filter to eliminate VBUS noise */
        if((ovp_debounce_cnt == 3u) || (uvp_debounce_cnt == 3u))
        {
            /* Fault is detected and appropriate actions are taken */
            trigger_fault();
        }
    }
}


/*******************************************************************************
* Function Name: adaptive_ctrl
********************************************************************************
* Summary:
* Function to handle adaptive tuning of the commutation phase advancement
* By default this function is disabled
* Enabled this function using the macro: ADAPTIVE_COMM_CTRL_ENABLE requires adjusting few parameters based on the system & motor design specifications
* WARNING: Enabling this function without properly setting the parameters may cause unexpected results
* Adaptive commutation control may be used for evaluation purpose
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void adaptive_ctrl(void)
{
    static int16_t current_change = 0;
    static int16_t current_val_old = 500; /* Initial arbitrary value should be greater than the value of 'current_val' */
    static uint16_t adaptive_time_cnt = 0;
    static bool pos_adv = 1; /* Default value = 1 to start with increasing comm_adv_percent */
    static int32_t rpm_old = 0;

    /* Count variable to track the timing required for execution of adaptive control routine */
    if(adaptive_time_cnt < 65535u) adaptive_time_cnt++; /* 65535 is the maximum range of 16-bit unsigned integer variable */

    /* Measure the average input current drawn by the motor */
    current_val = current_measure();

    /* Adaptive current measurement routine */
    if(adaptive_time_cnt == (uint16_t)(CURRENT_COMPARE_INTVL/CTRL_UPDATE_INVL))
    {
        /* Measure the change in current following the commutation change */
        current_change = current_val - current_val_old;

        /* Store the new current value */
        current_val_old = current_val;

        /* If current increases, then reverse the phase change */
        if(current_change > MIN_CURRENT_RISE_STEP)
        {
            /* Check if advance is currently set to positive */
            if(pos_adv)
                pos_adv = 0;
            else
                pos_adv = 1;
        }
    }
    /* Adaptive phase-tuner routine */
    if(adaptive_time_cnt == (uint16_t)(ADAPTIVE_TUNING_INTVL/CTRL_UPDATE_INVL))
    {
        /* Check if the motor is accelerating/decelerating. If yes, stop performing adaptive tuning
         * This is because, during this state, the reason for current change is unknown */
        if((((int32_t)rpm - rpm_old) < (ACCELERATION_LIMIT * ADAPTIVE_TUNING_INTVL/1000000)) || (((int32_t)rpm - rpm_old) > -(ACCELERATION_LIMIT * ADAPTIVE_TUNING_INTVL/1000000)))
        {
            /* Minimum current change (+/-) required to continue adaptive phase tuning
             * If current change is less than the defined threshold, then run with the existing value of 'comm_adv_percent'
             * Increasing this current window size may lead to commutation phase stabilizing at wrong values at times because of current measure inaccuracies */
            if((current_change > CURRENT_CHANGE_WINDOW)||(current_change < -CURRENT_CHANGE_WINDOW))
            {
                /* This conditional statement decides whether to increase/decrease the phase_offset */
                if(pos_adv)
                {
                    adv_offset += COMM_CHANGE_STEP;
                    if(adv_offset > ADV_OFFSET_LIMIT) /* Maximum offset limit */
                        adv_offset = ADV_OFFSET_LIMIT;
                }
                else
                {
                    adv_offset -= COMM_CHANGE_STEP;
                    if(adv_offset < -ADV_OFFSET_LIMIT) /* Minimum offset limit */
                        adv_offset = -ADV_OFFSET_LIMIT;
                }
            }
        }
        /* Store the current rpm value */
        rpm_old = (int32_t)rpm;

        /* Setting this variable to zero is required to re-run the adaptive routine regularly */
        adaptive_time_cnt = 0;
    }
}


/*******************************************************************************
* Function Name: hw_timer_isr
********************************************************************************
* Summary:
* Handler function is executed on CYBSP_TIMER overflow
* Used to run the timer based tasks to handle software timers events
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void hw_timer_isr(void)
{
    /* Timer overflow count variable used for tracking time in rpm calculation */
    hall_timer_tc_cnt++;

    /* Overflow counter for software timer event handler */
    for(uint8_t i = 0; i < SW_TIMERS_NUM; i++)
    {
        if(timer[i].active)
        {
            /* Increment the overflow flag to track the no. of timer overflows */
            timer[i].overflow++;
        }
    }
    /* Software timer event handler */
    sw_timer_event_handler();

    /* Clear the Timer interrupt on Terminal Count */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC);
}

/*******************************************************************************
* Function Name: sw_timer_start
********************************************************************************
* Summary:
* Starts a particular software timer for a pre-determined time duration (in microseconds)
* Registers callback function associated with the corresponding software timer
*
* Parameters:
* - timerIndex
* - duration
* - callback
*
* Return:
*  None
*
*******************************************************************************/
void sw_timer_start(uint8_t timerIndex, uint32_t duration, void (*callback)(void))
{
    if(timerIndex >= SW_TIMERS_NUM)
        CY_ASSERT(CY_ASSERT_FAILED);    /* Timer ID beyond limit */

    if(!timer[timerIndex].active)    /* Don't disturb an already active timer */
    {
        timer[timerIndex].active = true;
        timer[timerIndex].startTime = Cy_TCPWM_Counter_GetCounter(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);
        timer[timerIndex].duration = duration;
        timer[timerIndex].overflow = 0u;
        timer[timerIndex].callback = callback;
    }
}


/*******************************************************************************
* Function Name: sw_timer_event_handler
********************************************************************************
* Summary:
* Runs various events for software timers
* Triggers the corresponding callback function on a particular timer expiry
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void sw_timer_event_handler(void)
{
    /* Read the hardware timer count value */
    uint16_t currentTime = Cy_TCPWM_Counter_GetCounter(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);

    for(uint8_t i = 0; i < SW_TIMERS_NUM; i++)
    {
        /* Process only the active timers */
        if(timer[i].active)
        {
            /* Calculate the time elapsed corresponding to respective timer IDs */
            uint32_t elapsed_time = (uint32_t)((int32_t)(currentTime + timer[i].overflow * (TIMER_PERIOD_CNT + TIMER_PERIOD_OFFSET_ERROR)) - (int32_t)timer[i].startTime);

            if(elapsed_time >= timer[i].duration)
            {
                /* Execute the respective callback on timer expiry */
                timer[i].active = false;
                timer[i].overflow = 0;
                timer[i].callback();
            }
        }
    }
}


/*******************************************************************************
* Function Name: comm_adv_control
********************************************************************************
* Summary:
* Function to handle the commutation-advancement timer expiry events
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void comm_adv_control(void)
{
    /* Condition to start commutation advancement */
    if(gl_comm_adv_flag_cnt >= (COMM_ADVANCE_HALL_CNT - 1))
    {
        /* Commutation advancement control */
        if(drive_dir_high)
        {
            /* Drive the DIR output pin HIGH for commutation */
            Cy_GPIO_Set(DIR_PORT, DIR_PIN);
        }
        else
        {
            /* Drive the DIR output pin LOW for commutation */
            Cy_GPIO_Clr(DIR_PORT, DIR_PIN);
        }
    }
}


/*******************************************************************************
* Function Name: pwm_deadband_control
********************************************************************************
* Summary:
* Function to handle PWM dead-band control event
* Kills PWM signal at the function is called
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void pwm_deadband_control(void)
{
    /* Kill the PWM signal to allow free-wheeling of current to reduce current spike */
    Cy_TCPWM_TriggerStopOrKill(CYBSP_PWM_HW, CYBSP_PWM_MASK);
}


/*******************************************************************************
* Function Name: hall_sensor_isr
********************************************************************************
* Summary:
* Hall-sensor interrupt handler; triggers on both rising & falling edge interrupts
* Handler function is used to measuring the motor RPM based on Hall sensor feedback
* as well as for closed-loop commutation timing
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void hall_sensor_isr(void)
{
    volatile static int16_t capture_val_old = 0u;
    static int16_t fir_rpm_buff[FIR_RPM_SAMPLE_CNT] = {0}; /* Buffer to hold FIR filter samples for RPM */

    /* Start closed-loop control on detecting the hall-sensor interrupts */
    if(closedLoop == 0u) closedLoop = 1u; /* Flag is used for startup rotor lock detection */

    /* Stop the commutation-advance timer forcefully if not deactivated already
     * This is needed in some corner cases where hall interrupts are detected before commutation advancing timer elapses */
    timer[TIMER0].active = false;

    /* Variable to keep the track of hall interrupts during the startup acceleration */
    if(gl_comm_adv_flag_cnt < COMM_ADVANCE_HALL_CNT) gl_comm_adv_flag_cnt++;

    /* Condition to prevent closed-loop operation when a fault is detected => closedLoop = 255u */
    if(closedLoop == 1u)
    {
#if PWM_DEADBAND_ENABLE
        /* Restart the control PWM */
        Cy_TCPWM_TriggerStart(CYBSP_PWM_HW, CYBSP_PWM_MASK);
#endif /* PWM_DEADBAND_ENABLE */

        /* Handles DIR pin control for closed-loop commutation */
        /*  Rising hall edge detection */
        if(Cy_GPIO_Read(HALL_IN_PORT, HALL_IN_PIN) == 1u)
        {
#if !DIR_INVERT
            drive_dir_high = 0;

            if((gl_comm_adv_flag_cnt < COMM_ADVANCE_HALL_CNT) || !COMM_ADVANCE_ENABLE)
            {
                /* Drive the DIR output pin HIGH for commutation */
                Cy_GPIO_Set(DIR_PORT, DIR_PIN);
            }
#else
            drive_dir_high = 1;

            if((gl_comm_adv_flag_cnt < COMM_ADVANCE_HALL_CNT) || !COMM_ADVANCE_ENABLE)
            {
                /* Drive the DIR output pin LOW for commutation */
                Cy_GPIO_Clr(DIR_PORT, DIR_PIN);
            }
#endif /* !DIR_INVERT */
            /* Copy the hall signal to the host connector pin */
            Cy_GPIO_Set(FAULT_PORT, FAULT_PIN);
        }
        /*  Falling hall edge detection */
        else if(Cy_GPIO_Read(HALL_IN_PORT, HALL_IN_PIN) == 0u)
        {
#if !DIR_INVERT
            drive_dir_high = 1;

            if((gl_comm_adv_flag_cnt < COMM_ADVANCE_HALL_CNT) || !COMM_ADVANCE_ENABLE)
            {
                /* Drive the DIR output pin LOW for commutation */
                Cy_GPIO_Clr(DIR_PORT, DIR_PIN);
            }
#else
            drive_dir_high = 0;

            if((gl_comm_adv_flag_cnt < COMM_ADVANCE_HALL_CNT) || !COMM_ADVANCE_ENABLE)
            {
                /* Drive the DIR output pin HIGH for commutation */
                Cy_GPIO_Set(DIR_PORT, DIR_PIN);
            }
#endif /* !DIR_INVERT */
            /* Copy the hall signal to the host connector pin */
            Cy_GPIO_Clr(FAULT_PORT, FAULT_PIN);
        }

        /* Initiate closed-loop control routine once on detecting hall edge */
        if(gl_comm_adv_flag_cnt == 1u)
        {
            timer[TIMER1].active = false; /* TIMER1 is used during startup. So it must be stopped first to reuse it */
            ctrl_timer_cb(); /* Control timer callback function for closed-loop control */
        }

        /* Read the Timer count value, corresponding to the time interval between Hall triggers */
        uint16_t capture_val_new = Cy_TCPWM_Counter_GetCounter(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);

        capture_tot = (uint16_t)(((int32_t)(hall_timer_tc_cnt * (TIMER_PERIOD_CNT + TIMER_PERIOD_OFFSET_ERROR)) - (int32_t)capture_val_old) + (int32_t)capture_val_new);
        hall_timer_tc_cnt = 0; /* Clear the variable immediately */

#if COMM_ADVANCE_ENABLE
        /* Calculate the required commutation timing based on the % advancement demanded */
        uint32_t advance_time = (uint32_t)((float)(100 - comm_adv_percent)/100 * capture_tot - COMM_ADVANCE_OFFSET_CALIBRATE);

        /* Trigger start the software timer for commutation-advancement */
        sw_timer_start(TIMER0, advance_time, comm_adv_control);
#endif /* COMM_ADVANCE_ENABLE */

#if PWM_DEADBAND_ENABLE
        /* Calculate the PWM dead time based on the Hall signal interval */
        uint32_t pwm_dead_time = (uint32_t)((float)(100 - PWM_DEADBAND_PERCENT)/100 * capture_tot - COMM_ADVANCE_OFFSET_CALIBRATE);

        /* Trigger start the software timer for commutation-advancement */
        sw_timer_start(TIMER3, pwm_dead_time, pwm_deadband_control);
#endif /* PWM_DEADBAND_ENABLE */

        /* Store the previously captured Hall time interval value */
        capture_val_old = capture_val_new;

        /* Calculate the current motor RPM based on Hall sensor feedback */
        rpm_raw = (uint32_t)((SEC_IN_MIN/4) * ((float)TIMER_CLK_FREQ / (float)capture_tot) * MICROSECONDS);
        /* NOTE: At least HALL interrupt cycles are required to calculate RPM value correctly */

        rpm = fir_filter((int16_t)rpm_raw, FIR_RPM_SAMPLE_CNT, fir_rpm_buff);   /* 54us for 10 samples */

        /* Asserting the motor is alive */
        hall_fail_cnt = 0;
    }
    /* Clear the hall sensor GPIO interrupt */
    Cy_GPIO_ClearInterrupt(HALL_IN_PORT, HALL_IN_NUM);
}


/*******************************************************************************
* Function Name: ref_speed_measure
********************************************************************************
*
* Summary:
*  This function reads the external analog reference speed signal through 8-bit SAR ADC
*  Calculates the reference RPM corresponding to the analog input signal
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ref_speed_measure(void)
{
    static uint16_t count = 0;
    static uint32_t rpm_ramp = 2000u; /* Initial value of ref_rpm to start acceleration */

#if INTERNAL_RPM_SEL
    /* Soft-start implementation using acceleration control */
    if((count < 900u) && (rpm_ramp < FIXED_REF_RPM))
    {
        count++; /* Each increment occurs at a time interval defined in CTRL_UPDATE_INVL */
        rpm_ramp += 20u; /* Ramp the rpm in steps of 20 RPM/CTRL_UPDATE_INVL */
        ref_rpm = rpm_ramp;
    }
    else
        ref_rpm = FIXED_REF_RPM;
#else
    uint8_t adcResult1;
    static uint8_t soft_start_flag = 1;

    /* Disconnect 'VBUS_SENSE' GPIO from AMUXA */
    Cy_GPIO_SetHSIOM(VBUS_SENSE_PORT, VBUS_SENSE_NUM, HSIOM_SEL_GPIO);

    /* Connect 'REF_RPM' GPIO to AMUXA */
    Cy_GPIO_SetHSIOM(REF_RPM_PORT, REF_RPM_NUM, HSIOM_SEL_AMUXA);

    /* Calibrate ADC volts per division value before taking any ADC readings; VDDD supply level could vary across time */
    Cy_USBPD_Adc_Calibrate(&USBPD, CY_USBPD_ADC_ID_0);

    /* Enables the ADC block to function as an ADC and returns the sample value in ADC units */
    /* AMUXA line is used as the ADC input source */
    adcResult1 = Cy_USBPD_Adc_Sample(&USBPD, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_INPUT_AMUX_A);

    /* Calculate the reference RPM */
    ref_rpm = REF_RPM_MIN + (REF_RPM_MAX - REF_RPM_MIN) * adcResult1 / ADC_RESULT_MAX;

    /* Soft-start implementation using acceleration control */
    if((count < 900u) && (rpm_ramp < ref_rpm) && (soft_start_flag == 1))
    {
        count++; /* Each increment occurs at a time interval defined in CTRL_UPDATE_INVL */
        rpm_ramp += 20u; /* Ramp the rpm in steps of 20 RPM/CTRL_UPDATE_INVL */
        ref_rpm = rpm_ramp;
    }
    else
        soft_start_flag = 0;
#endif /* INTERNAL_RPM_SEL */
}


/*******************************************************************************
* Function Name: vbus_measure
********************************************************************************
*
* Summary:
*  This function reads the VBUS voltage scaled down using the voltage divider
*  Scaling factor: 'VBUS_SCALE_FACTOR' defined in parameters.h file
*
* Parameters:
*  None
*
* Return:
*  volt
*
*******************************************************************************/
float vbus_measure(void)
{
    float volt;
    uint8_t adcResult2;

    /* Disconnect 'REF_RPM' GPIO from AMUXA */
    Cy_GPIO_SetHSIOM(REF_RPM_PORT, REF_RPM_NUM, HSIOM_SEL_GPIO);

    /* Connect 'VBUS_SENSE' GPIO to AMUXA */
    Cy_GPIO_SetHSIOM(VBUS_SENSE_PORT, VBUS_SENSE_NUM, HSIOM_SEL_AMUXA);

    /* Calibrate ADC volts per division value before taking any ADC readings; VDDD supply level could vary across time */
    Cy_USBPD_Adc_Calibrate(&USBPD, CY_USBPD_ADC_ID_0);

    /* Enables the ADC block to function as an ADC and returns the sample value in ADC units */
    /* AMUXA line is used as the ADC input source */
    adcResult2 = Cy_USBPD_Adc_Sample(&USBPD, CY_USBPD_ADC_ID_0, CY_USBPD_ADC_INPUT_AMUX_A);

    /* Calculate the VBUS voltage */
    volt = VBUS_SCALE_FACTOR * ADC_VREF_VOLT * adcResult2 / (float)ADC_RESULT_MAX;

    return volt;
}


/*******************************************************************************
* Function Name: piControl
********************************************************************************
*
* Summary:
*  This function implements the tasks related to closed-loop motor control
*  This function runs a PI-control loop for speed control
*  Sets the PWM duty cycle required to achieve a desired speed
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void piControl(void)
{
    if(closedLoop == 1u)
    {
    /* START: PI-CONTROL LOOP */
        /* RPM error */
        rpm_error = ref_rpm - rpm;

        /* RPM integral error */
        integral_error += KI * rpm_error;

        /* Integral lower bound */
        if(integral_error <  -INTG_LIMIT)
            integral_error = -INTG_LIMIT;

        /* Integral upper bound */
        if(integral_error >  INTG_LIMIT)
            integral_error = INTG_LIMIT;

        /* PI-controller to calculate the new value of PWM duty cycle */
        pwm_compare = (KP * rpm_error + integral_error);

        /* PWM duty cycle upper constraint */
        if(pwm_compare > PWM_COMPARE_MAX)
            pwm_compare = PWM_COMPARE_MAX;

        /* PWM duty cycle lower constraint */
        if(pwm_compare < PWM_COMPARE_MIN)
            pwm_compare = PWM_COMPARE_MIN;
    /* END: PI-CONTROL LOOP */
    }

    /* Update the new PWM duty cycle by writing the new compare value */
    Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, (uint16_t)pwm_compare);
}


/*******************************************************************************
* Function Name: trigger_fault
********************************************************************************
*
* Summary:
*  This function is called whether any of the fault conditions are detected by
*  by the firmware
*
*  The function immediately kills the drive PWM signal and resets all control flags
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void trigger_fault(void)
{
    /* Stop the control timer to end the control and clear the corresponding timer flag */
    timer[TIMER1].active = false;
    gl_ctrl_update_flag = 0;

    /* Kill the PWM signal to stop the motor */
    Cy_TCPWM_TriggerStopOrKill(CYBSP_PWM_HW, CYBSP_PWM_MASK); /* Kill the PWM only after stopping the control timer (TIMER1) */

    closedLoop = 255u; /* Assigned with an arbitrary value other than 0u and 1u to indicate fault */
    pwm_compare = 0u;
    rpm = 0u;

    /* Assert LOW at Fault line command to host */
    Cy_GPIO_Clr(FAULT_PORT, FAULT_PIN); /* Expected pin config: Open-Drain, Drive LOW with external pull-up resistor */

#if DEBUG_PRINT
    strcpy(fault, "Rotor lock/Hall sensor fail"); /* Fault type is printed on UART monitor */
#endif /* DEBUG_PRINT */
}


/*******************************************************************************
* Function Name: rotor_lock_check_cb
********************************************************************************
*
* Summary:
*  Checks whether the rotor is locked during startup
*
*  Triggers a fault if rotor-lock is detected
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void rotor_lock_check_cb(void)
{
    if(!closedLoop) /* closedLoop = 1 implies that at least one Hall edge is detected */
        trigger_fault();
}


/*******************************************************************************
* Function Name: current_measure
********************************************************************************
* Summary:
* Function to measure current using LSCSA and 8-bit ADC
* The current samples are low-pass filtered digitally to smoothen the varying motor current waveform
* The result yields a low-pass filtered average current value
*
* Parameters:
*  None
*
* Return:
*  avg_current
*
*******************************************************************************/
int16_t current_measure(void)
{
    int16_t newCurrent, avg_current;
    static int16_t fir_buff1[FIR1_SAMPLE_CNT] = {0}; /* Buffer to hold FIR-1 filter samples */
    static int16_t fir_buff2[FIR2_SAMPLE_CNT] = {0}; /* Buffer to hold FIR-2 filter samples */

    /* Measure the current sample and low-pass filter it digitally to yield the average value
     * The  two code lines takes around (153us + 1.4ms) execution time, respectively
     *  Argument FIR1_SAMPLE_CNT specifies the no. of current samples used for filtering */
    newCurrent = (int16_t)(((int16_t)Cy_USBPD_Vbus_MeasureCur(&USBPD) + LSCSA_OFFSET_CALIBRATION_FACTOR) * LSCSA_SLOPE_CALIBRATION_FACTOR);   /* 153us */
    avg_current = fir_filter(newCurrent, FIR1_SAMPLE_CNT, fir_buff1);   /* 1.4ms for 255 samples */

    /* Returns the second MA averaged filter output with input taken from the output of the first MA filter output
     * The second filter may be eliminated if low-pass filtering is used in the hardware, which saves 1.4ms time */
    avg_current = fir_filter(avg_current, FIR2_SAMPLE_CNT, fir_buff2);   /* 1.4ms for 255 samples */

    /* The filtered output is scaled by 10 to compensate the LSCSA output */
    avg_current = 10 * avg_current;

    /* Offset and slope correction is performed to yield the final current value */
    avg_current = (int16_t)((avg_current + CURRENT_OFFSET_CORRECTION_FACTOR) * CURRENT_SLOPE_CORRECTION_FACTOR);

    return avg_current;
}


/*******************************************************************************
* Function Name: fir_filter
********************************************************************************
* Summary:
* Digital FIR filter to average the current sample values
* Moving Average (MA) filtering method is used.
* Takes a new current sample as input and provides the filtered output based on previous current sample values
*
* This function is a blocking function and takes around 2ms for execution with 255 samples used for filtering
*
* Parameters:
*  input, N
*
* Return:
*  output
*
*******************************************************************************/
int16_t fir_filter(int16_t input, uint8_t N, int16_t *buff)
{
    /* Temporary buffer to store the output while processing */
    float output = 0;

    /* Shift the filter array elements */
    for(uint8_t i = 0; i<(N-1); i++)
    {
        buff[i] = buff[i+1];
    }
    /* Store the new current sample value */
    buff[N-1] = input;

    for(uint8_t i = 0; i<N; i++)
    {
        output += buff[i];
    }
    /* Calculate the average value of the samples stored in the buffer */
    output = output / N;

    return (int16_t)output;
}


/*******************************************************************************
* Function Name: UART_transmit_data
********************************************************************************
*
* Summary:
*  This function transmits UART data at pre-determined time interval
*  The interval is specified by 'UART_PRINT_INVL' macro in parameters.h file
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
#if DEBUG_PRINT
void UART_transmit_data(void)
{
    if(gl_uart_print_flag)
    {
        /* Start transmit operation (do not check status) */
        Cy_SCB_UART_Transmit(CYBSP_UART_HW, str, sizeof(str), &CYBSP_UART_context);

        gl_uart_print_flag = 0;
    }
}

/* UART timer callback */
void uart_timer_cb(void)
{
    gl_uart_print_flag = 1;
    sw_timer_start(TIMER2, UART_PRINT_INVL, uart_timer_cb);
}

/* UART interrupt processing for UART High-Level API */
void UART_Isr(void)
{
    Cy_SCB_UART_Interrupt(CYBSP_UART_HW, &CYBSP_UART_context);
}
#endif /* DEBUG_PRINT */


/*******************************************************************************
* Function Name: check_status
********************************************************************************
* Summary:
*  Prints the error message.
*
* Parameters:
*  error_msg - message to print if any error encountered.
*  status - status obtained after evaluation.
*
* Return:
*  void
*
*******************************************************************************/
#if DEBUG_PRINT
void check_status(char *message, cy_rslt_t status)
{
    char error_msg[50];

    sprintf(error_msg, "Error Code: 0x%08" PRIX32 "\n", status);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\nFAIL: ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, message);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, error_msg);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
}
#endif /* DEBUG_PRINT */
/* [] END OF FILE */

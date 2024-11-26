/******************************************************************************
* File Name: parameters.h
*
* Description: This is the header file which includes all the parameter definitions
* required for the motor control operations
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*******************************************************************************/

/*******************************************************************************
* Macros
*******************************************************************************/

/* CY ASSERT failure */
#define CY_ASSERT_FAILED                      (0u)

/* System configuration */
#define VDDD                                  (float)(3.3)    /* 3.3V */
#define VBUS_RANGE                            (float)(16.6)    /* (V); Maximum measurable range of input DC voltage to the system; This voltage corresponds to 3.3V output at voltage divider */
#define VBUS_SCALE_FACTOR                     (VBUS_RANGE/VDDD)    /* Scale down VBUS to chip VDDD level using external voltage divider */
#define VBUS_LIMIT_MIN                        (float)(11.0)    /* (V); Minimum VBUS voltage limit for starting the motor */
#define VBUS_LIMIT_MAX                        (float)(13.0)    /* (V); Maximum VBUS voltage limit for starting the motor */
#define VBUS_OVP_LIMIT                        (float)(15.0)    /* (V); Maximum VBUS voltage limit to trigger over-voltage condition */
#define VBUS_UVP_LIMIT                        (float)(10.0)    /* (V); Minimum VBUS voltage limit to trigger under-voltage condition */

#define LSCSA_SLOPE_CALIBRATION_FACTOR        (float)(1)    /* LS-CSA slope correction factor; Calibrated based on experimental data */
#define LSCSA_OFFSET_CALIBRATION_FACTOR       (int16_t)(0)    /* LS-CSA offset correction factor; Calibrated based on experimental data */
#define CURRENT_SLOPE_CORRECTION_FACTOR       (float)(1)    /* Slope correction in final current output vs input; Experimentally determined */
#define CURRENT_OFFSET_CORRECTION_FACTOR      (int16_t)(100)    /* Offset correction in final current output vs input; Experimentally determined */

/* ADC configuration */
#define ADC_VREF                              CY_USBPD_ADC_VREF_VDDD    /* < CY_USBPD_ADC_VREF_VDDD > Or < CY_USBPD_ADC_VREF_PROG > (2.0V) */
#define ADC_VREF_VOLT                         (VDDD)    /* V */
#define ADC_RESULT_MAX                        (255u)

/* Timer configurations */
#define SW_TIMERS_NUM                         (4u)    /* Max number of software timers */
#define TIMER_CLK_FREQ                        (1u)    /* MHz */
#define TIMER_PERIOD_CNT                      (30u)    /* (us); Should be greater than 10us; decides the accuracy of software-timers */
#define TIMER_PERIOD_OFFSET_ERROR             (1u)    /* (us); Don't use floating point values */
#define SEC_IN_MIN                            (60u)    /* Seconds in minute */
#define MILLISECONDS                          (1000u)    /* milliseconds in second */
#define MICROSECONDS                          (1000000u)    /* microseconds in second */

/* PWM configurations */
#define PWM_DUTY_MIN                          (10u)    /* % */
#define PWM_DUTY_MAX                          (99u)    /* % */
#define PWM_PERIOD_CNT                        (3200u)    /* Period count required for 15kHz PWM with 48MHz peripheral clock */
#define PWM_COMPARE_MIN                       (PWM_DUTY_MIN * PWM_PERIOD_CNT / 100)
#define PWM_COMPARE_MAX                       (PWM_DUTY_MAX * PWM_PERIOD_CNT / 100)
#define PWM_STARTUP_COMPARE_CNT               (PWM_STARTUP_DUTY * PWM_PERIOD_CNT / 100)

/* Control configurations */
#define HALL_CNT_MOTOR_FAIL                   (10u)    /* No. of hall pulses absence to be monitored before asserting motor failure */
#define REF_RPM_MIN                           (4000u)    /* Minimum reference RPM corresponding to lowest analog input voltage command */
#define REF_RPM_MAX                           (20000u)    /* Maximum reference RPM corresponding to highest analog input voltage command */
#define FIR_RPM_SAMPLE_CNT                    (10u)    /* No. of samples used for FIR digital filtering of RPM */
#define KP                                    (float)(0.3)    /* Proportional gain; Decides the rise-time of the system */
#define KI                                    (float)(0.0005)    /* Integral gain; Eliminates the steady state error of the system response */
#define INTG_LIMIT                            (int16_t)(PWM_PERIOD_CNT)    /* Integral boundary value to prevent wind-up */
#define CTRL_UPDATE_INVL                      (3500u)    /* microseconds */
#define STARTUP_ROTOR_LOCK_INVL               (500000u)    /* microseconds */

/* Commutation timing configurations */
#define COMM_ADVANCE_HALL_CNT                 (15u)    /* Enter the no. of hall edges after which commutation advancing begins after startup; Low value < 10 may cause missing commutations during startup */
#define COMM_ADVANCE_OFFSET_CALIBRATE         (25u)    /* Offset delay from hall interrupt to comm_adv timer trigger (microseconds) */
#define FIR1_SAMPLE_CNT                       (200u)    /* No. of samples used for FIR-1 digital filtering of current */
#define FIR2_SAMPLE_CNT                       (255u)    /* No. of samples used for FIR-2 digital filtering of current */
#define ADV_OFFSET_LIMIT                      (int16_t)(10)    /* Phase advancement % offset value to shift the Phase vs RPM relation plot */
#define COMM_ADV_MIN                          (int16_t)(7)    /* Minimum possible % commutation phase advancement; value should be > 5 to prevent missing commutations */
#define COMM_VS_RPM_SLOPE                     (float)(0.00033)    /* Slope of % commutation phase advancement vs RPM curve; COMM_VS_RPM_SLOPE = (comm_adv - COMM_ADV_MIN)/(rpm - 2500) */
#define MIN_CURRENT_RISE_STEP                 (int16_t)(0)    /* Threshold (mA) to detect a current change following commutation phase change; Eliminates noise in current sensing */
#define CURRENT_CHANGE_WINDOW                 (int16_t)(10)    /* Threshold (mA) for change in current sample value to update commutation phase in adaptive control; This window helps to stabilize the commutation phase in adaptive control */
#define CURRENT_COMPARE_INTVL                 (1500000u)    /* microseconds; Delay-time to measure change in current after commutation change */
#define ADAPTIVE_TUNING_INTVL                 (3000000u)    /* microseconds; Time interval to run-adaptive tuning regularly */
#define COMM_CHANGE_STEP                      (int8_t)(3)    /* % change in comm_advancement in each adaptive tuning cycle */
#define ACCELERATION_LIMIT                    (int32_t)(200)    /* Maximum acceleration/deceleration (RPM/s) limit beyond which adaptive tuning is disabled */

/* UART serial data print configurations */
#define DEBUG_PRINT                           (0u)    /* Set to '1u' to view the motor control parameters on UART terminal (230400bps) */
#if DEBUG_PRINT
#define UART_PRINT_INVL                       (100000u)    /* microseconds */
#define PRINT_SYSTEM_DATA                     (1u)    /* View the system data such as RPM, PWM duty, VBUS voltage on the UART serial monitor */
#define PRINT_PI_CONTROL_DATA                 (0u)    /* View run-time PI-control data on the UART serial monitor */
#define PRINT_COMM_ADVANCE_DATA               (0u)    /* View dynamic commutation advancement data on the UART serial monitor */
#endif /* DEBUG_PRINT*/

/* User control configurations */
#define PWM_STARTUP_DUTY                      (25u)    /* % Duty cycle of PWM to be applied for initial startup */
#define DIR_INVERT                            (0u)    /* Set to '1u' to reverse the direction of rotation/reverse commutation */
#define PWM_DEADBAND_ENABLE                   (1u)    /* Enable PWM dead-band during commutation to allow free-wheeling and to reduce current spike */
#define PWM_DEADBAND_PERCENT                  (15u)    /* PWM dead-band time as a percentage of Hall signal interval */

#define FIXED_PWM_DUTY_ENABLE                 (0u)    /* Enable to run the motor in open loop with fixed PWM duty cycle */
#if FIXED_PWM_DUTY_ENABLE
#define FIXED_PWM_DUTY_VALUE                  (30u)    /* Enter the fixed PWM duty cycle in % for open loop operation */
#define FIXED_PWM_COMPARE_CNT                 (FIXED_PWM_DUTY_VALUE * PWM_PERIOD_CNT / 100)
#endif /* FIXED_PWM_DUTY_ENABLE */

#define INTERNAL_RPM_SEL                      (0u)    /* Set '1u' to enable internal fixed RPM */
#if INTERNAL_RPM_SEL
#define FIXED_REF_RPM                         (10000u)    /* Specify the fixed RPM value for internal RPM select mode; Range: REF_RPM_MIN to REF_RPM_MAX */
#endif /* INTERNAL_RPM_SEL */

#define COMM_ADVANCE_ENABLE                   (0u)    /* Enable/disable commutation timing control */
#if COMM_ADVANCE_ENABLE
#define FIXED_COMM_ADVANCE_VALUE              (int16_t)(10)    /* Specify the fixed value of commutation phase advancement (%) */

#define LINEAR_COMM_ADVANCE_ENABLE            (1u)    /* Enable/disable variable commutation phase advancement control */
#if LINEAR_COMM_ADVANCE_ENABLE
#define ADAPTIVE_COMM_CTRL_ENABLE             (0u)    /* Default = 0u; Enable/disable adaptive commutation phase control based on current measurement */
                                                      /* Enabling this may require system-specific tuning of few parameters to work properly */
#endif /* LINEAR_COMM_ADVANCE_ENABLE */
#endif /* COMM_ADVANCE_ENABLE */

/* [] END OF FILE */

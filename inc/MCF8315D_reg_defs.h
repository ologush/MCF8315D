#ifndef MOTOR_REG_DEFS
#define MOTOR_REG_DEFS

/* EEPROM Algorithm Registers */
#define MCF8315_EEPROM_ISD_CONFIG_REG        0x080
#define MCF8315_EEPROM_REV_DRIVE_CONFIG_REG  0x082
#define MCF8315_EEPROM_MOTOR_STARTUP_1_REG   0x084
#define MCF8315_EEPROM_MOTOR_STARTUP_2_REG   0x086
#define MCF8315_EEPROM_CLOSED_LOOP1_REG      0x088
#define MCF8315_EEPROM_CLOSED_LOOP2_REG      0x08A
#define MCF8315_EEPROM_CLOSED_LOOP3_REG      0x08C
#define MCF8315_EEPROM_MOTOR_PARAMS_REG      0x08E
#define MCF8315_EEPROM_REF_PROFILES1_REG     0x094
#define MCF8315_EEPROM_REF_PROFILES2_REG     0x096
#define MCF8315_EEPROM_REF_PROFILES3_REG     0x098
#define MCF8315_EEPROM_REF_PROFILES4_REG     0x09A
#define MCF8315_EEPROM_REF_PROFILES5_REG     0x09C
#define MCF8315_EEPROM_REF_PROFILES6_REG     0x09E

/* EEPROM Fault Configuration Registers */
#define MCF8315_EEPROM_FAULT_CONFIG1_REG     0x090
#define MCF8315_EEPROM_FAULT_CONFIG2_REG     0x092

/* EEPROM Hardware Configuration Registers */
#define MCF8315_EEPROM_PIN_CONFIG_REG        0x0A4
#define MCF8315_EEPROM_DEVICE_CONFIG1_REG    0x0A6
#define MCF8315_EEPROM_DEVICE_CONFIG2_REG    0x0A8
#define MCF8315_EEPROM_PERI_CONFIG1_REG      0x0AA
#define MCF8315_EEPROM_GD_CONFIG1_REG        0x0AC
#define MCF8315_EEPROM_GD_CONFIG2_REG        0x0AE

/* EEPROM Internal Algorithm Configuration Registers*/
#define MCF8315_EEPROM_INT_ALGO_1_REG        0x0A0
#define MCF8315_EEPROM_INT_ALGO_2_REG        0x0A2

/* RAM Register Map */

/* Fault Status Registers */
#define MCF8315_GATE_DRIVER_FAULT_STATUS_REG    0x0E0
#define MCF8315_CONTROLLER_FAULT_STATUS_REG     0x0E2
#define MCF8315_EEPROM_FAULT_STATUS_REG         0x32C

/* System Status Registers */
#define MCF8315_ALGO_STATUS_REG                 0x0E4
#define MCF8315_MTR_PARAMS_REG                  0x0E6
#define MCF8315_ALGO_STATUS_MPET_REG            0x0E8

/* Device Control Registers */
#define MCF8315_ALGO_CTRL1_REG                  0x0EA

/* Algorithm Control Registers */
#define MCF8315_ALGO_DEBUG1_REG               0x0EC
#define MCF8315_ALGO_DEBUG2_REG               0x0EE
#define MCF8315_CURRENT_PI_REG               0x0F0
#define MCF8315_SPEED_PI_REG                 0x0F2
#define MCF8315_DAC_1_REG                    0x0F4
#define MCF8315_DAC_2_REG                    0x0F6
#define MCF8315_EEPROM_SECURITY_REG          0x0F8

/* Algorithm Variables Registers */
#define MCF8315_ALGORITHM_STATE_REG            0x18E
#define MCF8315_FG_SPEED_FDBK_REG              0x194
#define MCF8315_VBETA_REG                      0x400
#define MCF8315_BUS_CURRENT_REG                0x40C
#define MCF8315_PHASE_CURRENT_A_REG            0x444
#define MCF8315_PHASE_CURRENT_B_REG            0x446
#define MCF8315_PHASE_CURRENT_C_REG            0x448
#define MCF8315_CSA_GAIN_FEEDBACK_REG          0x46C
#define MCF8315_VOLTAGE_GAIN_FEEDBACK_REG      0x477
#define MCF8315_VM_VOLTAGE_REG                 0x47C
#define MCF8315_PHASE_VOLTAGE_VA_REG           0x484
#define MCF8315_PHASE_VOLTAGE_VB_REG           0x486
#define MCF8315_PHASE_VOLTAGE_VC_REG           0x488
#define MCF8315_SIN_COMMUTATION_ANGLE_REG      0x4BC
#define MCF8315_COS_COMMUTATION_ANGLE_REG      0x4BE
#define MCF8315_IALPHA_REG                     0x4DC
#define MCF8315_IBETA_REG                      0x4DE
#define MCF8315_VALPHA_REG                     0x4E0
#define MCF8315_ID_REG                         0x4EC
#define MCF8315_IQ_REG                         0x4EE
#define MCF8315_VD_REG                         0x4F0
#define MCF8315_VQ_REG                         0x4F2
#define MCF8315_IQ_REF_ROTOR_ALIGN_REG         0x4F2
#define MCF8315_SPEED_REF_OPEN_LOOP_REG        0x540
#define MCF8315_IQ_REF_OPEN_LOOP_REG           0x550
#define MCF8315_SPEED_REF_CLOSED_LOOP_REG      0x5D2
#define MCF8315_ID_REF_CLOSED_LOOP_REG         0x612
#define MCF8315_IQ_REF_CLOSED_LOOP_REG         0x614
#define MCF8315_ISD_STATE_REG                  0x6AE
#define MCF8315_ISD_SPEED_REG                  0x6B8
#define MCF8315_IPD_STATE_REG                  0x6EA
#define MCF8315_IPD_ANGLE_REG                  0x72E
#define MCF8315_ED_REG                         0x772
#define MCF8315_EQ_REG                         0x774
#define MCF8315_SPEED_FDBK_REG                 0x782
#define MCF8315_THETA_EST_REG                  0x786

#endif
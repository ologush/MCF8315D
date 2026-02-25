#ifndef MCF8315D_EEPROM_defs
#define MCF8315D_EEPROM_defs

// ISD_CONFIG
#define ISD_EN 1
#define BRAKE_EN 1
#define HIZ_EN 0
#define RVS_DR_EN 1
#define RESYNC_EN 1
#define FW_DRV_RESYN_THR 0x3
#define BRK_MODE 0
#define BRK_CONFIG 0
#define BRK_CURR_THR 0x4
#define BRK_TIME 0x6
#define HIZ_TIME 0x6
#define STAT_DETECT_THR 0
#define REV_DRV_HANDOFF_THR 0x3
#define REV_DRV_OPEN_LOOP_CURRENT 0

// REV_DRIVE_CONFIG
#define REV_DRV_OPEN_LOOP_ACCEL_A1 0x2
#define REV_DRV_OPEN_LOOP_ACCEL_A2 0x2
#define ACTIVE_BRAKE_CURRENT_LIMIT 0x1
#define ACTIVE_BRAKE_KP 0 // Come back to this
#define ACTIVE_BRAKE_KI 0 // Come back to this

// MOTOR_STARTUP1
#define MTR_STARTUP 0x2
#define ALIGN_SLOW_RAMP_RATE 1
#define ALIGN_TIME 0x2
#define ALIGN_OR_SLOW_CURRENT_ILIMIT 0x4
#define IPD_CLK_FREQ 0x2
#define IPD_CURR_THR 0x5
#define IPD_RLS_MODE 0x1
#define IPD_ADV_ANGLE 0x3
#define IPD_REPEAT 0x1
#define IQ_RAMP_EN 0x1
#define ACTIVE_BRAKE_EN 0x1
#define REV_DRV_CONFIG 0x0

// MOTOR_STARTUP2
#define OL_ILIMIT 0x4
#define OL_ACC_A1 0x4
#define OL_ACC_A2 0x4
#define AUTO_HANDOFF_EN 0x1
#define OPN_CL_HANDOFF_THR 0x9
#define ALIGN_ANGLE 0x0
#define SLOW_FIRST_CYC_FREQ 0xE
#define FIRST_CYCLE_FREQ_SEL 0x0
#define THETA_ERROR_RAMP_RATE 0x4

// CLOSED_LOOP1
#define OVERMODULATION_ENABLE 0x0
#define CL_ACC 0x6
#define CL_DEC 0x6
#define PWM_FREQ_OUT 0x0
#define PWM_MODE 0x0
#define FG_SEL 0x0
#define FG_DIV 0x0
#define FG_CONFIG 0x0
#define FG_BEMF_THR 0x0
#define AVS_EN 0x1
#define DEADTIME_COMP_EN 0x0
#define LOW_SPEED_RECIRC_BRAKE_EN 0x0

// CLOSED_LOOP2
#define MTR_STOP 0x0
#define MTR_STOP_BRK_TIME 0x8
#define ACT_SPIN_THR 0x5
#define BRAKE_SPEED_THRESHOLD 0x0
#define MOTOR_RES 0x0 // Will obtain with MPET
#define MOTOR_IND // Will obtain with MPET

// CLOSED_LOOP3
#define MOTOR_BEMF_CONST // Will obtain with MPET
#define CURR_LOOP_KP // WIll obtain with MPET
#define CURR_LOOP_KI // WIll obtain with mPET


#endif
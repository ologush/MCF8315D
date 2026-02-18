#ifndef MCF8315D
#define MCF8315D

// Includes

#include <stdint.h>

/* Macros */

#define MCF8315D_I2C_ADDRESS    0x01

#define D_LEN_16_BIT      0x00
#define D_LEN_32_BIT      0x01
#define D_LEN_64_BIT      0x02

#define OP_RW_WRITE      0x00
#define OP_RW_READ       0x01

#define CRC_EN_DISABLE  0x00
#define CRC_EN_ENABLE   0x01

#define OP_RW_MASK        0x800000
#define CRC_EN_MASK       0x400000
#define D_LEN_MASK        0x300000
#define MEM_SEC_MASK      0x0F0000
#define MEM_PAGE_MASK     0x00F000
#define MEM_ADDR_MASK     0x000FFF

#define MAX_SPEED          7200.0f // Placeholder value


/* Typedefs */

typedef enum {
    MOTOR_CTRL_ERR_OK,
    MOTOR_CTRL_ERR_ERROR
} MOTOR_ERRORS_e;

typedef enum {
    MCF_IDLE,
    MCF_TX_R,
    MCF_TX_W,
    MCF_RX,
    MCF_DONE,
    MCF_ERROR
} MCF8315_STATE_e;

typedef struct {
    //Actual measured values
    float motor_resistance;
    float motor_inductance;
    float motor_bemf_constant;

    float current_loop_ki;
    float current_loop_kp;

    float speed_loop_ki;
    float speed_loop_kp;

    //LUT Locations of nearest values
    uint8_t motor_resistance_hex;
    uint8_t motor_inductance_hex;
    uint8_t motor_bemf_constant_hex;
} motor_parameters_s;

typedef struct {
    uint8_t read_write_bit  :   1;
    uint8_t target_id       :   7;
    uint8_t mem_sec         :   4;
    uint8_t d_len           :   2;
    uint8_t crc_en          :   1;
    uint8_t op_rw           :   1;
    uint8_t test            :   4;
    uint16_t mem_addr       :   12;


    union {
        uint16_t data_16;
        uint32_t data_32;
        uint64_t data_64;
    } data_u;

} motor_data_word_s;

typedef struct {
    uint8_t reg_address;
    uint32_t reg_value;
} eeprom_register_s;

/* Public function prototypes */
MOTOR_ERRORS_e motor_ctrl_init(I2C_HandleTypeDef *hi2c);
MOTOR_ERRORS_e motor_parameter_extraction(motor_parameters_s *motor_params);
MOTOR_ERRORS_e write_config_to_eeprom();
MOTOR_ERRORS_e read_config_from_eeprom();
MOTOR_ERRORS_e motor_startup_sequence(void);
MOTOR_ERRORS_e motor_set_speed(float speed_rpm);
MOTOR_ERRORS_e motor_get_speed(float *speed_rpm);
MOTOR_ERRORS_e handle_fault(void);
MOTOR_ERRORS_e get_fault(uint32_t *gate_driver_fault, uint32_t *controller_fault);
MOTOR_ERRORS_e clear_fault(void);
MOTOR_ERRORS_e extract_motor_params(motor_parameters_s *extracted_params);
MOTOR_ERRORS_e run_mpet(void);

MOTOR_ERRORS_e find_target_id(void);

#endif
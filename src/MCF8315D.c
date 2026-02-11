// Includes
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_i2c.h"
#include "../inc/MCF8315D.h"
#include <math.h>
#include "../inc/MCF8315D_reg_defs.h"

//Replace with specific STM32 in use


/* Private Variables */
static eeprom_register_s eeprom_default_config[] = {
    {MCF8315_EEPROM_ISD_CONFIG_REG,        0x7C700484},
    {MCF8315_EEPROM_REV_DRIVE_CONFIG_REG,  0x00000000},
    {MCF8315_EEPROM_MOTOR_STARTUP_1_REG,   0x40000096},
    {MCF8315_EEPROM_MOTOR_STARTUP_2_REG,   0x00000000},
    {MCF8315_EEPROM_CLOSED_LOOP1_REG,      0x00000000},
    {MCF8315_EEPROM_CLOSED_LOOP2_REG,      0x00000000},
    {MCF8315_EEPROM_CLOSED_LOOP3_REG,      0x00000000},
    {MCF8315_EEPROM_MOTOR_PARAMS_REG,      0x00000000},
    {MCF8315_EEPROM_REF_PROFILES1_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES2_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES3_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES4_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES5_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES6_REG,     0x00000000}
};

static I2C_HandleTypeDef *hi2c_motor_ctrl;

// Private function prototypes
static MOTOR_ERRORS_e calculate_crc(motor_data_word_s *data_word);
static MOTOR_ERRORS_e initial_eeprom_config(void);
static MOTOR_ERRORS_e read_eeprom_config(uint32_t *config_data);

static MOTOR_ERRORS_e MCF8315_write_register(uint16_t reg_address, uint64_t reg_value, uint8_t length);
static MOTOR_ERRORS_e MCF8315_read_register(uint16_t reg_address, uint64_t *reg_value, uint8_t length);

static MOTOR_ERRORS_e MCF8315_write_register(uint16_t reg_address, uint64_t reg_value, uint8_t length) {

    uint8_t tx_buffer_len = 4;
    switch(length) {
        case D_LEN_16_BIT:
            tx_buffer_len += 2;
            break;
        case D_LEN_32_BIT:
            tx_buffer_len += 4;
            break;
        case D_LEN_64_BIT:
            tx_buffer_len += 8;
            break;
        default:
            return MOTOR_CTRL_ERR_ERROR;
    }
    
    union {
        uint8_t buffer[tx_buffer_len];
        motor_data_word_s data_word;
    } tx_union;

    tx_union.data_word.target_id = MCF8315D_I2C_ADDRESS;
    tx_union.data_word.read_write_bit = OP_RW_WRITE;
    tx_union.data_word.op_rw = OP_RW_WRITE;
    tx_union.data_word.crc_en = CRC_EN_DISABLE;
    tx_union.data_word.d_len = length;
    tx_union.data_word.mem_sec = 0;
    tx_union.data_word.mem_addr = reg_address << 4; // Fix works for now with how the bitfields are set. May have to fix in the future

    switch (length) {
        case D_LEN_16_BIT:
            tx_union.data_word.data_u.data_16 = (uint16_t)reg_value;
            break;
        case D_LEN_32_BIT:
            tx_union.data_word.data_u.data_32 = (uint32_t)reg_value;
            break;
        case D_LEN_64_BIT:
            tx_union.data_word.data_u.data_64 = reg_value;
            break;
        default:
            return MOTOR_CTRL_ERR_ERROR;
    }

    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(hi2c_motor_ctrl, tx_union.buffer[0], tx_union.buffer + 1, tx_buffer_len - 1, HAL_MAX_DELAY);
    if(ret != HAL_OK) {
        return MOTOR_CTRL_ERR_ERROR;
    }
    return MOTOR_CTRL_ERR_OK;
}

static MOTOR_ERRORS_e MCF8315_read_register(uint16_t reg_address, uint64_t *reg_value, uint8_t length) {

    uint8_t real_length = 0;

    switch(length) {
        case D_LEN_16_BIT:
            real_length = 2;
            break;
        case D_LEN_32_BIT:
            real_length = 4;
            break;
        case D_LEN_64_BIT:
            real_length = 8;
            break;
        default:
            return MOTOR_CTRL_ERR_ERROR;
    }

    union {
        uint8_t buffer[8];
        motor_data_word_s data_word;
    } tx_union = {.buffer = 0};

    tx_union.data_word.target_id = MCF8315D_I2C_ADDRESS;
    tx_union.data_word.read_write_bit = OP_RW_WRITE;
    tx_union.data_word.op_rw = OP_RW_READ;
    tx_union.data_word.crc_en = CRC_EN_DISABLE;
    tx_union.data_word.d_len = length;
    tx_union.data_word.mem_sec = 0;
    tx_union.data_word.test = 0;
    tx_union.data_word.mem_addr = reg_address << 4; // Fix works for now with how the bitfields are set. May have to fix in the future

    union {
        uint8_t buffer[real_length];
        uint64_t data_value;
    } rx_union;
    HAL_StatusTypeDef e;
    e = HAL_I2C_Master_Transmit(hi2c_motor_ctrl, tx_union.buffer[0], tx_union.buffer + 1, 3, HAL_MAX_DELAY);
    if (e == HAL_ERROR) {
        return MOTOR_CTRL_ERR_ERROR;
    }
    HAL_I2C_Master_Receive(hi2c_motor_ctrl, tx_union.buffer[0], rx_union.buffer, real_length, HAL_MAX_DELAY);

    *reg_value = rx_union.data_value;

    return MOTOR_CTRL_ERR_OK;
}

static MOTOR_ERRORS_e calculate_crc(motor_data_word_s *data_word) {

    /* Todo: Implement Algorithm */
    return MOTOR_CTRL_ERR_OK;
}



static MOTOR_ERRORS_e read_eeprom_config(uint32_t *config_data) {

    // Set speed ref to 0, skeptical that this is the right value but it is what is written in the datasheet
    MCF8315_write_register(MCF8315_ALGO_DEBUG1_REG, 0x08000000, D_LEN_32_BIT);

    clear_fault();

    // Read EEPROM into its corresponding shadow registers
    MCF8315_write_register(MCF8315_ALGO_CTRL1_REG, 0x40000000, D_LEN_32_BIT);

    HAL_Delay(200); //Wait for EEPROM read to complete

    // This checks to ensure that the EEPROM was read successfully
    union {
        uint64_t data_64;
        uint32_t data_32;
    } read_check_union;

    MCF8315_read_register(MCF8315_ALGO_CTRL1_REG, &read_check_union.data_64, D_LEN_32_BIT);

    if (read_check_union.data_32 != 0x00000000) {
        return MOTOR_CTRL_ERR_ERROR;
    }

    for(uint8_t i = MCF8315_EEPROM_ISD_CONFIG_REG; i<= MCF8315_EEPROM_GD_CONFIG2_REG; i+=2) {

        union {
            uint64_t data_64;
            uint32_t data_32;
        } eeprom_value_union;

        MCF8315_read_register(i, &eeprom_value_union.data_64, D_LEN_32_BIT);

        uint8_t index = (i - MCF8315_EEPROM_ISD_CONFIG_REG) / 2;

        config_data[(i - MCF8315_EEPROM_ISD_CONFIG_REG) / 2] = eeprom_value_union.data_32;
    }

    return MOTOR_CTRL_ERR_OK;
}

/* Global function definitions */

MOTOR_ERRORS_e motor_ctrl_init(I2C_HandleTypeDef *hi2c)
{
    hi2c_motor_ctrl = hi2c;

    uint8_t target_id = 0;
    target_id = find_target_id();
    if (target_id == 0xFF) {
        return MOTOR_CTRL_ERR_ERROR;
    }

    handle_fault();

    uint32_t config_data[14];
    read_eeprom_config(config_data);
    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e motor_startup_sequence(void) {
    //First set speed ref > 0
    uint32_t motor_speed = 100.0f;
    motor_set_speed(motor_speed);

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e motor_set_speed(float speed_rpm) {

    if(speed_rpm > MAX_SPEED) {
        return MOTOR_CTRL_ERR_ERROR;
    }

    uint32_t speed_mask = 0x8000FFFF; // Mask to clear speed bits

    //Convert speed from RPM to register value
    uint16_t speed = (uint16_t)roundf((speed_rpm / MAX_SPEED) * 65535.0f);

    uint32_t current_register_value;
    
    union {
        uint64_t data_64;
        uint32_t data_32;
    } current_speed_union;

    MCF8315_read_register(MCF8315_ALGO_DEBUG1_REG, &current_speed_union.data_64, D_LEN_32_BIT);

    current_speed_union.data_32 &= speed_mask;
    current_speed_union.data_32 |= ((uint32_t)speed << 16);

    uint32_t set_speed = ((uint32_t)speed << 16);

    MCF8315_write_register(MCF8315_ALGO_DEBUG1_REG, set_speed, D_LEN_32_BIT);

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e motor_get_speed(float *speed_rpm) {

    union {
        uint64_t data_64;
        uint32_t data_32;
    } speed_union;

    MCF8315_read_register(MCF8315_SPEED_FDBK_REG, &speed_union.data_64, D_LEN_32_BIT);

    *speed_rpm = (float)(((float)speed_union.data_32/pow(2, 27)) * MAX_SPEED * 60);

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e extract_motor_params(motor_parameters_s *extracted_params) {

    union {
        uint64_t data_64;
        uint32_t data_32;
        uint8_t data_buffer[4];
    } reg_value_union;

    MCF8315_read_register(MCF8315_MTR_PARAMS_REG, &reg_value_union.data_64, D_LEN_32_BIT);

    extracted_params->motor_inductance_hex = reg_value_union.data_buffer[1];
    extracted_params->motor_bemf_constant_hex = reg_value_union.data_buffer[2];
    extracted_params->motor_resistance_hex = reg_value_union.data_buffer[3];

    uint32_t current_loop_data;
    uint32_t speed_loop_data;

    MCF8315_read_register(MCF8315_CURRENT_PI_REG, &reg_value_union.data_64, D_LEN_32_BIT);
    current_loop_data = reg_value_union.data_32;

    MCF8315_read_register(MCF8315_SPEED_PI_REG, &reg_value_union.data_64, D_LEN_32_BIT);
    speed_loop_data = reg_value_union.data_32;


    extracted_params->current_loop_ki = 1000*((0xFF0000 & current_loop_data) >> 16)/(pow(10,((0x3000000 & current_loop_data) >> 18)));
    extracted_params->current_loop_kp = (current_loop_data & 0xFF)/(pow(10,((current_loop_data & 0x300) >> 8)));

    extracted_params->speed_loop_ki = 0.1*((0xFF0000 & speed_loop_data) >> 16)/(pow(10,((0x3000000 & current_loop_data) >> 18)));
    extracted_params->speed_loop_kp = 0.01*(0xFF & speed_loop_data)/(pow(10,((0x300 & speed_loop_data) >> 8)));

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e run_mpet(void) {

    MCF8315_write_register(MCF8315_ALGO_DEBUG2_REG, 0x0000001F, D_LEN_32_BIT); // Start motor parameter extraction

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e handle_fault(void) {

    uint32_t gate_driver_fault = 0;
    uint32_t controller_fault = 0;
    get_fault(&gate_driver_fault, &controller_fault);

    // Implement fault handling logic

    clear_fault();
}

MOTOR_ERRORS_e get_fault(uint32_t *gate_driver_fault, uint32_t *controller_fault) {

    union {
        uint64_t data_64;
        uint32_t data_32;
    } fault_union = {.data_64 = 0};

    MCF8315_read_register(MCF8315_GATE_DRIVER_FAULT_STATUS_REG, &fault_union.data_64, D_LEN_32_BIT);
    *gate_driver_fault = fault_union.data_32;

    MCF8315_read_register(MCF8315_CONTROLLER_FAULT_STATUS_REG, &fault_union.data_64, D_LEN_32_BIT);
    *controller_fault = fault_union.data_32;

    return MOTOR_CTRL_ERR_OK;

}

MOTOR_ERRORS_e clear_fault(void) {

    union {
        uint64_t data_64;
        uint32_t data_32;
    } fault_data_union;

    MCF8315_read_register(MCF8315_ALGO_CTRL1_REG, &fault_data_union.data_64, D_LEN_32_BIT);
    fault_data_union.data_32 = fault_data_union.data_32 & 0xDFFFFFFF;

    MCF8315_write_register(MCF8315_ALGO_CTRL1_REG, fault_data_union.data_32, D_LEN_32_BIT);

    return MOTOR_CTRL_ERR_OK;
}

uint8_t find_target_id() {

    for (uint8_t i = 1; i < 2; i++) {
        
        uint8_t ret = HAL_I2C_IsDeviceReady(hi2c_motor_ctrl, (uint16_t)(i << 1), 3, 5);

        if (ret == HAL_OK) {
            return i;
        }
    }

    return 0xFF;

}
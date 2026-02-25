// Includes
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_i2c.h"
#include "../inc/MCF8315D.h"
#include <math.h>
#include "../inc/MCF8315D_reg_defs.h"

#define EEPROM_TIMEOUT 1000

#define MPET_ROUTINE
#define MPET_TIMEOUT_ms 1000000

#define POLE_PAIRS 4
#define MAX_SPEED_HZ 0x1000
#define MAX_SPEED_RPM 250 // TODO: Change

/* Private Variables */
static eeprom_register_s eeprom_default_config[] = {
    {MCF8315_EEPROM_ISD_CONFIG_REG,        0x64F34CA0},
    {MCF8315_EEPROM_REV_DRIVE_CONFIG_REG,  0xA8200000},
    {MCF8315_EEPROM_MOTOR_STARTUP_1_REG,   0x8B080790},
    {MCF8315_EEPROM_MOTOR_STARTUP_2_REG,   0x23026014},
    {MCF8315_EEPROM_CLOSED_LOOP1_REG,      0x903981B8},
    {MCF8315_EEPROM_CLOSED_LOOP2_REG,      0x0BADCA70},
    {MCF8315_EEPROM_CLOSED_LOOP3_REG,      0x00000000},
    {MCF8315_EEPROM_CLOSED_LOOP4_REG,      0x00100002},
    {MCF8315_EEPROM_REF_PROFILES1_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES2_REG,     0x00000000},
    {MCF8315_EEPROM_REF_PROFILES3_REG,     0x80000002},
    {MCF8315_EEPROM_REF_PROFILES4_REG,     0x80068000},
    {MCF8315_EEPROM_REF_PROFILES5_REG,     0x80000010},
    {MCF8315_EEPROM_REF_PROFILES6_REG,     0x800001A3},
    {MCF8315_EEPROM_FAULT_CONFIG1_REG,     0x000004B0},
    {MCF8315_EEPROM_FAULT_CONFIG2_REG,     0xBED9909E},
    {MCF8315_EEPROM_PIN_CONFIG_REG,        0x08200309},
    {MCF8315_EEPROM_DEVICE_CONFIG1_REG,    0x83E8E000},
    {MCF8315_EEPROM_DEVICE_CONFIG2_REG,    0xF55349F8},
    {MCF8315_EEPROM_PERI_CONFIG1_REG,      0x83085C00},
    {MCF8315_EEPROM_GD_CONFIG1_REG,        0x9C461900},
    {MCF8315_EEPROM_GD_CONFIG2_REG,        0x00840000},
    {MCF8315_EEPROM_INT_ALGO_1_REG,        0x00000000},
    {MCF8315_EEPROM_INT_ALGO_2_REG,        0x89750005},
};

static I2C_HandleTypeDef *hi2c_motor_ctrl;
static MCF8315_STATE_e state;

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

    uint8_t tx_buffer[tx_buffer_len];
    tx_buffer[0] = MCF8315D_I2C_ADDRESS << 1;
    tx_buffer[1] = length << 4;
    tx_buffer[2] = (reg_address & 0xF00) >> 8;
    tx_buffer[3] = reg_address & 0x0FF;
    
    uint8_t *reg_bytes = (uint8_t*)&reg_value;
    for(uint8_t i = 0; i < (tx_buffer_len - 4); i++) {
        tx_buffer[i + 4] = reg_bytes[i];
    }

    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(hi2c_motor_ctrl, tx_buffer[0], tx_buffer + 1, tx_buffer_len - 1, HAL_MAX_DELAY);
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

    uint8_t tx_buffer[4];

    tx_buffer[0] = MCF8315D_I2C_ADDRESS << 1;
    tx_buffer[1] = (1 << 7) | (length << 4);
    tx_buffer[2] = (reg_address & 0xF00) >> 8;
    tx_buffer[3] = (reg_address & 0xFF);
    
    union {
        uint8_t buffer[8];
        uint64_t data_value;
    } rx_union;

    rx_union.data_value = 0;

    HAL_StatusTypeDef e;

    state = MCF_TX_R;
    e = HAL_I2C_Master_Seq_Transmit_IT(hi2c_motor_ctrl, tx_buffer[0], tx_buffer + 1, 3, I2C_FIRST_FRAME);
    while (state == MCF_TX_R);
    state = MCF_RX;
    e = HAL_I2C_Master_Seq_Receive_IT(hi2c_motor_ctrl, tx_buffer[0], rx_union.buffer, real_length, I2C_LAST_FRAME);
    while (state != MCF_DONE);
    state = MCF_IDLE;
    
    *reg_value = rx_union.data_value;

    return MOTOR_CTRL_ERR_OK;
}

static MOTOR_ERRORS_e calculate_crc(motor_data_word_s *data_word) {

    /* Todo: Implement Algorithm */
    return MOTOR_CTRL_ERR_OK;
}



static MOTOR_ERRORS_e read_eeprom_config(uint32_t *config_data) {

    MCF8315_read_eeprom();

    for(uint8_t i = MCF8315_EEPROM_ISD_CONFIG_REG; i<= MCF8315_EEPROM_GD_CONFIG2_REG; i+=2) {

        union {
            uint64_t data_64;
            uint32_t data_32;
        } eeprom_value_union;

        MCF8315_read_register(i, &eeprom_value_union.data_64, D_LEN_32_BIT);

        uint8_t index = (i - MCF8315_EEPROM_ISD_CONFIG_REG) / 2;

        config_data[index] = eeprom_value_union.data_32;
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

    uint32_t eeprom_data[24] = {0};
    read_eeprom_config(eeprom_data);

#ifdef MPET_ROUTINE
    run_mpet();
#else
// Load parameters from EEPROM
#endif



    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e MCF8315_set_speed(float speed_rpm) {

    if(speed_rpm > MAX_SPEED_RPM) {
        return MOTOR_CTRL_ERR_ERROR;
    }

    uint16_t speed = (uint16_t)roundf((speed_rpm / MAX_SPEED_RPM) * 32768.0f);
    
    uint64_t current_speed;

    MCF8315_read_register(MCF8315_ALGO_DEBUG1_REG, &current_speed, D_LEN_32_BIT);

    current_speed &= 0x8000FFFF;
    current_speed |= ((uint32_t)speed << 16);

    MCF8315_write_register(MCF8315_ALGO_DEBUG1_REG, current_speed, D_LEN_32_BIT);

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

    uint64_t reg_value;

    // Set Max Speed (should be done in a different setup)
    MCF8315_read_register(MCF8315_EEPROM_CLOSED_LOOP4_REG, &reg_value, D_LEN_32_BIT);
    reg_value = (reg_value & 0xFFFFC000) | 0xFFF; //(MAX_SPEED_HZ * 6); // Max speed is 2730 Hz (Unsure RPS until pole pair number is determined)
    MCF8315_write_register(MCF8315_EEPROM_CLOSED_LOOP4_REG, reg_value, D_LEN_32_BIT);

    // Enable MPET and writing the results to the shadow ram
    MCF8315_read_register(MCF8315_EEPROM_ISD_CONFIG_REG, &reg_value, D_LEN_32_BIT);
    reg_value = (reg_value & 0xFFFFFE3F);
    MCF8315_write_register(MCF8315_EEPROM_ISD_CONFIG_REG, reg_value, D_LEN_32_BIT);
    
    MCF8315_set_speed(0);
    set_speed_mode(MCF_SPEED_MODE_I2C);

    // Setting various parameters, will need to move this to its own function



    handle_fault();
    MCF8315_write_register(MCF8315_ALGO_DEBUG2_REG, 0x0000003F, D_LEN_32_BIT); // Start motor parameter extraction

    uint32_t timeout_timer = HAL_GetTick();
    do {
        MCF8315_read_register(MCF8315_ALGO_STATUS_MPET_REG, &reg_value, D_LEN_32_BIT);
        if(HAL_GetTick() - timeout_timer > MPET_TIMEOUT_ms) {
            return MOTOR_CTRL_ERR_ERROR;
        }
    } while((reg_value & 0xF0000000) != 0xF0000000);

    return MOTOR_CTRL_ERR_OK;
}

MOTOR_ERRORS_e handle_fault(void) {

    uint32_t gate_driver_fault = 0;
    uint32_t controller_fault = 0;
    get_fault(&gate_driver_fault, &controller_fault);

    // Implement fault handling logic

    MCF8315_clear_fault();
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

MOTOR_ERRORS_e MCF8315_clear_fault(void) {

    union {
        uint64_t data_64;
        uint32_t data_32;
    } fault_data_union;

    MCF8315_read_register(MCF8315_ALGO_CTRL1_REG, &fault_data_union.data_64, D_LEN_32_BIT);
    MCF8315_write_register(MCF8315_ALGO_CTRL1_REG, 0x30000000, D_LEN_32_BIT);

    return MOTOR_CTRL_ERR_OK;
}

uint8_t find_target_id() {

    for (uint8_t i = 0; i < 127; i++) {
        
        uint8_t ret = HAL_I2C_IsDeviceReady(hi2c_motor_ctrl, (uint16_t)(i << 1), 3, 5);

        if (ret == HAL_OK) {
            return i;
        }
    }

    return 0xFF;

}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {

    if (hi2c != hi2c_motor_ctrl) return;

    switch (state) {
        case MCF_TX_R:
            state = MCF_RX;
            break;
        case MCF_TX_W:
            state = MCF_DONE;
            break;
        defualt:
            break;
    }

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {

    if (hi2c != hi2c_motor_ctrl) return;

    switch (state) {
        case MCF_RX:
            state = MCF_DONE;
            break;
        default:
            break;
    }

}

MOTOR_ERRORS_e set_speed_mode(MCF8315_SPEED_MODE_e speed_mode) {
    
    uint64_t reg_value;

    MCF8315_read_register(MCF8315_EEPROM_PIN_CONFIG_REG, &reg_value, D_LEN_32_BIT);
    reg_value = (reg_value & 0xFFFFFFFC) | speed_mode;
    MCF8315_write_register(MCF8315_EEPROM_PIN_CONFIG_REG, reg_value, D_LEN_32_BIT);

}

MOTOR_ERRORS_e MCF8315_read_eeprom(void) {
    
    // Place device in idle/standby state
    MCF8315_write_register(MCF8315_ALGO_DEBUG1_REG, 0x08000000, D_LEN_32_BIT);

    MCF8315_clear_fault();

    // Read EEPROM into its corresponding shadow registers
    MCF8315_write_register(MCF8315_ALGO_CTRL1_REG, 0x40000000, D_LEN_32_BIT);

    uint32_t time = HAL_GetTick();
    uint64_t data;

    do {
        MCF8315_read_register(MCF8315_ALGO_CTRL1_REG, &data, D_LEN_32_BIT);
        if (time - HAL_GetTick() > EEPROM_TIMEOUT) {
            return MOTOR_CTRL_ERR_ERROR;
        }
    } while ((uint32_t) data != 0x00000000);


}
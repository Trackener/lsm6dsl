#include "LSM6DSL.h"

#include <functional>
#include <iostream>
#include <cassert>
#include <cstring>
#include <math.h>

#define WARNING(x) printf("[WARNING] "); printf(x)
#define INFO(x) printf("[INFO] "); printf(x)
#define ERROR(x) printf("[ERROR] "); printf(x)

#define LSM6DSL_ACC_GYRO_FUNC_CFG_ACCESS              0X01

#define LSM6DSL_ACC_GYRO_SENSOR_SYNC_TIME             0X04
#define LSM6DSL_ACC_GYRO_SENSOR_RES_RATIO             0X05

#define LSM6DSL_ACC_GYRO_FIFO_CTRL1                   0X06
#define LSM6DSL_ACC_GYRO_FIFO_CTRL2                   0X07
#define LSM6DSL_ACC_GYRO_FIFO_CTRL3                   0X08
#define LSM6DSL_ACC_GYRO_FIFO_CTRL4                   0X09
#define LSM6DSL_ACC_GYRO_FIFO_CTRL5                   0X0A

#define LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G             0X0B
#define LSM6DSL_ACC_GYRO_INT1_CTRL                    0X0D
#define LSM6DSL_ACC_GYRO_INT2_CTRL                    0X0E
#define LSM6DSL_ACC_GYRO_WHO_AM_I_REG                 0X0F
#define LSM6DSL_ACC_GYRO_CTRL1_XL                     0X10
#define LSM6DSL_ACC_GYRO_CTRL2_G                      0X11
#define LSM6DSL_ACC_GYRO_CTRL3_C                      0X12
#define LSM6DSL_ACC_GYRO_CTRL4_C                      0X13
#define LSM6DSL_ACC_GYRO_CTRL5_C                      0X14
#define LSM6DSL_ACC_GYRO_CTRL6_G                      0X15
#define LSM6DSL_ACC_GYRO_CTRL7_G                      0X16
#define LSM6DSL_ACC_GYRO_CTRL8_XL                     0X17
#define LSM6DSL_ACC_GYRO_CTRL9_XL                     0X18
#define LSM6DSL_ACC_GYRO_CTRL10_C                     0X19

#define LSM6DSL_ACC_GYRO_MASTER_CONFIG                0X1A
#define LSM6DSL_ACC_GYRO_WAKE_UP_SRC                  0X1B
#define LSM6DSL_ACC_GYRO_TAP_SRC                      0X1C
#define LSM6DSL_ACC_GYRO_D6D_SRC                      0X1D
#define LSM6DSL_ACC_GYRO_STATUS_REG                   0X1E

#define LSM6DSL_ACC_GYRO_OUT_TEMP_L                   0X20
#define LSM6DSL_ACC_GYRO_OUT_TEMP_H                   0X21
#define LSM6DSL_ACC_GYRO_OUTX_L_G                     0X22
#define LSM6DSL_ACC_GYRO_OUTX_H_G                     0X23
#define LSM6DSL_ACC_GYRO_OUTY_L_G                     0X24
#define LSM6DSL_ACC_GYRO_OUTY_H_G                     0X25
#define LSM6DSL_ACC_GYRO_OUTZ_L_G                     0X26
#define LSM6DSL_ACC_GYRO_OUTZ_H_G                     0X27
#define LSM6DSL_ACC_GYRO_OUTX_L_XL                    0X28
#define LSM6DSL_ACC_GYRO_OUTX_H_XL                    0X29
#define LSM6DSL_ACC_GYRO_OUTY_L_XL                    0X2A
#define LSM6DSL_ACC_GYRO_OUTY_H_XL                    0X2B
#define LSM6DSL_ACC_GYRO_OUTZ_L_XL                    0X2C
#define LSM6DSL_ACC_GYRO_OUTZ_H_XL                    0X2D
#define LSM6DSL_ACC_GYRO_SENSORHUB1_REG               0X2E
#define LSM6DSL_ACC_GYRO_SENSORHUB2_REG               0X2F
#define LSM6DSL_ACC_GYRO_SENSORHUB3_REG               0X30
#define LSM6DSL_ACC_GYRO_SENSORHUB4_REG               0X31
#define LSM6DSL_ACC_GYRO_SENSORHUB5_REG               0X32
#define LSM6DSL_ACC_GYRO_SENSORHUB6_REG               0X33
#define LSM6DSL_ACC_GYRO_SENSORHUB7_REG               0X34
#define LSM6DSL_ACC_GYRO_SENSORHUB8_REG               0X35
#define LSM6DSL_ACC_GYRO_SENSORHUB9_REG               0X36
#define LSM6DSL_ACC_GYRO_SENSORHUB10_REG              0X37
#define LSM6DSL_ACC_GYRO_SENSORHUB11_REG              0X38
#define LSM6DSL_ACC_GYRO_SENSORHUB12_REG              0X39
#define LSM6DSL_ACC_GYRO_FIFO_STATUS1                 0X3A
#define LSM6DSL_ACC_GYRO_FIFO_STATUS2                 0X3B
#define LSM6DSL_ACC_GYRO_FIFO_STATUS3                 0X3C
#define LSM6DSL_ACC_GYRO_FIFO_STATUS4                 0X3D
#define LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_L              0X3E
#define LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_H              0X3F
#define LSM6DSL_ACC_GYRO_TIMESTAMP0_REG               0X40
#define LSM6DSL_ACC_GYRO_TIMESTAMP1_REG               0X41
#define LSM6DSL_ACC_GYRO_TIMESTAMP2_REG               0X42

#define LSM6DSL_ACC_GYRO_TIMESTAMP_L                  0X49
#define LSM6DSL_ACC_GYRO_TIMESTAMP_H                  0X4A

#define LSM6DSL_ACC_GYRO_STEP_COUNTER_L               0X4B
#define LSM6DSL_ACC_GYRO_STEP_COUNTER_H               0X4C

#define LSM6DSL_ACC_GYRO_SENSORHUB13_REG              0X4D
#define LSM6DSL_ACC_GYRO_SENSORHUB14_REG              0X4E
#define LSM6DSL_ACC_GYRO_SENSORHUB15_REG              0X4F
#define LSM6DSL_ACC_GYRO_SENSORHUB16_REG              0X50
#define LSM6DSL_ACC_GYRO_SENSORHUB17_REG              0X51
#define LSM6DSL_ACC_GYRO_SENSORHUB18_REG              0X52

#define LSM6DSL_ACC_GYRO_FUNC_SRC                     0X53
#define LSM6DSL_ACC_GYRO_TAP_CFG1                     0X58
#define LSM6DSL_ACC_GYRO_TAP_THS_6D                   0X59
#define LSM6DSL_ACC_GYRO_INT_DUR2                     0X5A
#define LSM6DSL_ACC_GYRO_WAKE_UP_THS                  0X5B
#define LSM6DSL_ACC_GYRO_WAKE_UP_DUR                  0X5C
#define LSM6DSL_ACC_GYRO_FREE_FALL                    0X5D
#define LSM6DSL_ACC_GYRO_MD1_CFG                      0X5E
#define LSM6DSL_ACC_GYRO_MD2_CFG                      0X5F

#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_X_L              0X66
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_X_H              0X67
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Y_L              0X68
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Y_H              0X69
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Z_L              0X6A
#define LSM6DSL_ACC_GYRO_OUT_MAG_RAW_Z_H              0X6B

#define LSM6DSL_ACC_GYRO_X_OFS_USR                    0X73
#define LSM6DSL_ACC_GYRO_Y_OFS_USR                    0X74
#define LSM6DSL_ACC_GYRO_Z_OFS_USR                    0X75

#define BDU_BIT (0x01 << 0x06)
#define IF_INC  (0x01 << 0x02)

//LOOKUP Table regs.

//RESV registers
uint8_t registers_resv[] = {0x00, 0x02, 0x02, 0x0C, 0x1F, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
		0x56, 0x57, 0x62, 0x61, 0x62, 0x63, 0x64, 0x65, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71,
		0x72, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F};

uint8_t registers_readonly[] = {0x0f, 0x1B, 0x1C, 0x1D, 0x1E, 0x20, 0x21, 0x22, 0x23, 0x24,
		0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
		0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x49, 0x4A, 0x4B, 0x4C, 0x4D,
		0x4E, 0x4F, 0x50, 0x51, 0x51, 0x53, 0x54, 0x55, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B};


// FIFO_STATUS2

// Number of unread words
#define DIFF_FIFO (0xB)

//0: FIFO contains data
// 1: FIFO is empty
#define FIFO_EMPTY (0x1 << 4)

// 0: FIFO is not full
// 1: FIFO will be full at the next ODR
#define FIFO_FULL_SMART (0x1 << 5)

// 0: FIFO is not completely filled
// 1: FIFO is completely filled
#define OVER_RUN (0x1 << 6)

// 0: FIFO filling is lower than watermark level,
// 1: FIFO filling is equal to or higher than the watermark level
#define WATER_M (0x1 << 7)

#define REG_ADDRESS_MASK (0xFE)
#define MODE_MASK (0x7)
#define FIFO_ODR_MASK (0x78)

#define WRITE_TO_REG (0x0)
#define READ_TO_REG (0x1)
#define READ_WRITE_BIT_MASK (0x1)

#define REG_ADDRESS_MASK (0xFE)

#define DEVICE_CODE (0x11010110)

#define MAX_FREQUENCY (10000000)

#define DATA_READY_PULSES_MICROSECOND (75)

LSM6DSL::LSM6DSL() :
        mode_(FIFO_MODE_BYPASS),
        fifo_odr_(0),
        fifo_(),
        out_l_was_read(false),
        out_h_was_read(false),
        should_stop_(false),
        timer_id_(0),
        odr_(),
        output_regs_update_(false){
    int cs_pin_number = GetPinNumber("cs");
    int sck_pin_number = GetPinNumber("sck");
    int si_pin_number = GetPinNumber("si");
    int so_pin_number = GetPinNumber("so");
    sa0_pin_number_ = GetPinNumber("sa0");
    int1_pin_number_ = GetPinNumber("int1");
    int2_pin_number_ = GetPinNumber("int2");
    SpiSlaveConfig spi_config = {
          .mosi_pin_number = si_pin_number,
          .miso_pin_number = so_pin_number,
          .ss_pin_number = cs_pin_number,
          .sclk_pin_number = sck_pin_number,
          .supported_spi_modes = SPI_MODE_0 | SPI_MODE_3,
          .max_frequency = MAX_FREQUENCY,
          .bit_order = MSB_FIRST
    };

    spi_slave_ = CreateSpiSlave(spi_config);
    MemReset();
    odr_ = {{0,0}, {1,12.5}, {2,26}, {3,52}, {4,104}, {5,208}, {6,416}, {7,833}, {8,1660}, {9,3330}, {10,6660}};
}

void LSM6DSL::Main() {
    while (!should_stop_) {
        uint8_t opcode = 0;
        if (spi_slave_->Transmit(&opcode, nullptr,  1) == 0) {
            continue;
        }
        uint32_t reg_address = (opcode & REG_ADDRESS_MASK) >> 0x1;
        if ((opcode & READ_WRITE_BIT_MASK) == WRITE_TO_REG) {
            ReadDataFromMaster(reg_address);
        } else {
            WriteDataToMaster(reg_address);
        }
    }
}

void LSM6DSL::WriteToFifoCntrl5(uint8_t value) {
    fifo_odr_ = odr_[((value & FIFO_ODR_MASK) >> 3)];
    UpdateOdr();

    uint8_t current_mode = mode_;
    mode_ = (fifo_mode_t)(value & MODE_MASK);
    if ((mode_ != FIFO_MODE_BYPASS) && (mode_ != FIFO_MODE_FIFO)) {
        throw std::logic_error("LSM6DSL: Unsupported mode " + std::to_string(mode_));
    }

    if (mode_ == FIFO_MODE_BYPASS) {
        ClearFifo();
    } else if (mode_ == FIFO_MODE_FIFO && current_mode != FIFO_MODE_FIFO) {
        LoadDataIntoOutputRegs();
    }
    memory_[LSM6DSL_ACC_GYRO_FIFO_CTRL5] = value;
}

// Loads data from data-generator to fifo (rate: ODR_FIFO_ defined in WriteToFifoCntrl5)
void LSM6DSL::UpdateOdr() {
    if (fifo_odr_ == 0){
        if (timer_id_ != 0) {
            CancelTimedCallback(timer_id_);
            timer_id_ = 0;
        }
    } else if (timer_id_ != 0) {
        UpdateTimedCallback(timer_id_, ((pow(10, 9))/fifo_odr_));
    } else {
        timer_id_ = AddTimedCallback((pow(10, 9))/(fifo_odr_), std::bind(&LSM6DSL::TimerCallback, this), false);
    }
}

void LSM6DSL::WriteToDrdyPulseCfg(uint8_t value) {
    if ((value & (0x1 << 7)) > 0) {
        // Set data load mode
    }
    memory_[LSM6DSL_ACC_GYRO_D6D_SRC] = value;
}

void LSM6DSL::WriteToCtrl3C(uint8_t value) {
	if (value & (0x01)) {
		//SW_RESET
		INFO("SW Reset\r\n");
		value &= ~0x01;
	}
	if (value & (0x01 << 7)) {
		INFO("Rebooting memory content\r\n");
		MemReset();
	} else {
		memory_[LSM6DSL_ACC_GYRO_CTRL3_C] = value;
	}
}

void LSM6DSL::TimerCallback() {
    if(fifo_.size() < MAX_FIFO_SIZE){
        uint16_t sample =  GetNextInt16FromDataGenerator("accelerometer");
        fifo_.push(sample);
        if (!output_regs_update_) {
            LoadDataIntoOutputRegs();
        }
    }
}

void LSM6DSL::LoadDataIntoOutputRegs() {
    if (fifo_.empty()) {
        output_regs_update_ = false;
        return;
    }
    uint16_t value = fifo_.front();
    fifo_.pop();
    memory_[LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_L] = (uint8_t)(value & 0xFF);
    out_l_was_read = false;
    memory_[LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_H] = (uint8_t)((value & 0xFF00) >> 8);
    out_h_was_read = false;
    output_regs_update_ = true;
}

void LSM6DSL::ClearFifo() {
    std::queue<uint16_t> empty;
    std::swap(fifo_, empty);
    out_l_was_read = false;
    out_h_was_read = false;
}

void LSM6DSL::WriteDataToMaster(uint32_t start_reg_address) {
    uint32_t current_reg_address = start_reg_address;

    // Check if we should read from this reg
    if (memchr(registers_resv, (uint8_t) start_reg_address, sizeof(registers_resv))) {
        WARNING("Accessing resv registere!\r\n");
    }

    // Reading
    while (spi_slave_->IsSsActive()) {
        uint8_t sample = memory_[current_reg_address];
        if (spi_slave_->Transmit(nullptr, &sample, 1) == 0) {
            continue;
        }

        if(current_reg_address == LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_L) {
            out_l_was_read = true;
        } else if (current_reg_address == LSM6DSL_ACC_GYRO_FIFO_DATA_OUT_H) {
            out_h_was_read = true;
        }

        if (IsBduBitSet() && out_h_was_read && out_l_was_read) {
            LoadDataIntoOutputRegs();
        }

        if (!IsBduBitSet() && (out_h_was_read || out_l_was_read)) {
            // Implement each data should be loaded into regs and under which conditions
            LoadDataIntoOutputRegs();
        }

        if (IsIfIncSet()) {
        	//Iterate over registers if resv
        	do {
        		current_reg_address = (current_reg_address + 1) % MEMORY_SIZE;
        	} while (memchr(registers_resv, current_reg_address, sizeof(registers_resv)));
        }
    }
}

void LSM6DSL::ReadDataFromMaster(uint32_t start_reg_address) {
    uint32_t current_reg_address = start_reg_address;
    uint8_t data;
    // Reading
    while (spi_slave_->IsSsActive()) {
        if (spi_slave_->Transmit(&data, nullptr, 1) == 0) {
            continue;
        }
        switch (current_reg_address) {
            case (LSM6DSL_ACC_GYRO_FIFO_CTRL5): {
                WriteToFifoCntrl5(data);
                break;
            }

            case (LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G): {
                WriteToDrdyPulseCfg(data);
                break;
            }

            case (LSM6DSL_ACC_GYRO_CTRL3_C): {
            	WriteToCtrl3C(data);
            	break;
            }

            default: {
               if (memchr(registers_readonly, current_reg_address, sizeof(registers_readonly)) ||
            		   memchr(registers_resv, current_reg_address, sizeof(registers_resv))) {
            	   WARNING("Writing to read only/resv register\r\n");
               } else {
            	   memory_[current_reg_address] = data;
               }
            }
        }

        if (IsIfIncSet()) {
            current_reg_address = (current_reg_address + 1) % MEMORY_SIZE;
        }
    }
}

bool LSM6DSL::IsBduBitSet() {
    return (memory_[LSM6DSL_ACC_GYRO_CTRL3_C] & BDU_BIT) > 0;
}

bool LSM6DSL::IsIfIncSet() {
    return (memory_[LSM6DSL_ACC_GYRO_CTRL3_C] & IF_INC) > 0;
}

void LSM6DSL::Stop() {
    should_stop_ = true;
}

void LSM6DSL::MemReset() {
    memset(memory_, 0x00, MEMORY_SIZE);
    memory_[LSM6DSL_ACC_GYRO_WHO_AM_I_REG] = 0b01101010;
    memory_[LSM6DSL_ACC_GYRO_CTRL3_C] = 0b00000100;
}

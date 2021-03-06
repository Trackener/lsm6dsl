#pragma once
#include "ModelingFramework.h"
#include <map>
#include <queue>

#define MEMORY_SIZE (0x75)
#define MAX_FIFO_SIZE (4096)

class LSM6DSL : public ExternalPeripheral {
  public:
    LSM6DSL();
    void Main() override;
    void Stop() override;
    LSM6DSL(const LSM6DSL&) = delete;
    LSM6DSL& operator=(const LSM6DSL&) = delete;

  private:

    typedef enum {
        FIFO_MODE_BYPASS         = 0x00, // Bypass mode. Reset mode, fifo is disables
        FIFO_MODE_FIFO 		     = 0x01, //FIFO mode. Stops collecting data when FIFO is full.
        FIFO_MODE_STREAM 		 = 0x02, // Reserved
        FIFO_MODE_STF 	         = 0x03, // Continuous mode until trigger is deasserted, then FIFO mode.
        FIFO_MODE_BTS 		     = 0x04, // Bypass mode until trigger is deasserted, then Continuous mode
        FIFO_MODE_DYN_STREAM 	 = 0x05, // Reserved
        FIFO_MODE_DYN_STREAM_2   = 0x06, // Continuous mode. If the FIFO is full, the new sample overwrites the older one
        FIFO_MODE_BTF 		     = 0x07,  // Reserved
    } fifo_mode_t;

    typedef enum {
        READ_ONLY,
        WRITE_ONLY,
        READ_WRITE,
        NONE_READ_WRITE
    } read_write_t;

    typedef enum {
        ACC_FS_2G  = 0x00,
        ACC_FS_4G  = 0x04,
        ACC_FS_8G  = 0x08,
        ACC_FS_16G = 0x0C
    } eACC_FS;


    typedef enum {
        G_FS_125DPS  = 0x02,
        G_FS_250DPS  = 0x00,
        G_FS_500DPS  = 0x04,
        G_FS_1000DPS = 0x08,
        G_FS_2000DPS = 0x0C
    } eGYRO_FS;

    void WriteToFifoCntrl5(uint8_t value);
    void WriteToDrdyPulseCfg(uint8_t value);
    void WriteToCtrl3C(uint8_t value);
    void TimerCallback();
    void LoadDataIntoOutputRegs();
    void ClearFifo();
    bool IsIfIncSet();
    bool IsBduBitSet();
    void WriteDataToMaster(uint32_t start_reg);
    void ReadDataFromMaster(uint32_t start_reg_address);
    void MemReset();
    void UpdateOdr();

    fifo_mode_t mode_;
    float fifo_odr_;
    std::queue<uint16_t> fifo_; // data set is reserved for gyroscope data
    iSpiSlaveV1* spi_slave_ {};
    uint8_t memory_[MEMORY_SIZE] {};
    bool out_l_was_read;
    bool out_h_was_read;
    int sa0_pin_number_ {};
    int int1_pin_number_ {};
    int int2_pin_number_ {};
    bool should_stop_;
    int timer_id_;
    std::map<uint8_t, float> odr_;
    bool output_regs_update_;
};

extern "C" ExternalPeripheral *PeripheralFactory() {
    return new LSM6DSL();
}

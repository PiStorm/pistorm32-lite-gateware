#ifndef PS_PI_MODEL_H
#define PS_PI_MODEL_H

#include <cstdint>
#include <functional>

class Vpistorm;

struct uint128_t {
    uint64_t hi;
    uint64_t lo;
};

// PiStorm Register offsets
#define REG_DATA_LO 0
#define REG_DATA_HI 1
#define REG_ADDR_LO 2
#define REG_ADDR_HI 3
#define REG_STATUS  4
#define REG_CONTROL 4
#define REG_SLOT    5
#define REG_VERSION 7

// REG_STATUS flags
#define STATUS_IS_BM        (1 << 0)
#define STATUS_RESET        (1 << 1)
#define STATUS_HALT         (1 << 2)
#define STATUS_IPL_MASK     (7 << 3)
#define STATUS_IPL_SHIFT    3
#define STATUS_TERM_NORMAL  (1 << 6)
#define STATUS_REQ_ACTIVE   (1 << 7)

// REG_CONTROL flags
#define CONTROL_REQ_BM        (1 << 0)
#define CONTROL_DRIVE_RESET   (1 << 1)
#define CONTROL_DRIVE_HALT    (1 << 2)
#define CONTROL_DRIVE_INT2    (1 << 3)
#define CONTROL_DRIVE_INT6    (1 << 4)
#define CONTROL_INC_EXEC_SLOT (1 << 5)
#define CONTROL_ENABLE_PREFETCH (1 << 6)

// Transaction definitions
#define TXN_SIZE_SHIFT 8
#define TXN_RW_SHIFT   10
#define TXN_FC_SHIFT   11

#define SIZE_BYTE 0
#define SIZE_WORD 1
#define SIZE_LONG 3

#define TXN_READ  (1 << TXN_RW_SHIFT)
#define TXN_WRITE (0 << TXN_RW_SHIFT)

class PiStormPiModel {
public:
    using TickFunc = std::function<void(int cycles)>;

    PiStormPiModel(Vpistorm* dut, TickFunc tick_fn);
    ~PiStormPiModel();

    void init();

    // Raw Register R/W
    void write_ps_reg(uint32_t address, uint16_t data);
    uint16_t read_ps_reg(uint32_t address);
    bool wait_txn(int timeout_cycles = 20000);

    // Control Register helpers
    void ps_set_control(uint16_t value);
    void ps_clr_control(uint16_t value);
    uint16_t read_status();

    // 2-slot mode control
    void set_use_2slot(bool enable);
    bool get_use_2slot() const { return use_2slot_; }
    int  get_next_slot() const { return next_slot_; }
    void flush_pending_writes();

    // High level memory access matching Emu68 ps_protocol.c
    uint8_t  ps32_read_8(uint32_t address, uint8_t fc = 1);
    uint16_t ps32_read_16(uint32_t address, uint8_t fc = 1);
    uint32_t ps32_read_32(uint32_t address, uint8_t fc = 1);
    uint64_t ps32_read_64(uint32_t address, uint8_t fc = 1);
    uint128_t ps32_read_128(uint32_t address, uint8_t fc = 1);

    void ps32_write_8(uint32_t address, uint8_t data, uint8_t fc = 1);
    void ps32_write_16(uint32_t address, uint16_t data, uint8_t fc = 1);
    void ps32_write_32(uint32_t address, uint32_t data, uint8_t fc = 1);
    void ps32_write_64(uint32_t address, uint64_t data, uint8_t fc = 1);
    void ps32_write_128(uint32_t address, uint128_t data, uint8_t fc = 1);

    // Serial debug lines
    void set_serial(bool dat, bool clk);

    // Direct slot inspection
    bool is_slot_active(int slot) const { return slot_active_[slot & 1] != 0; }

private:
    uint32_t ps32_do_read_access_1s(uint32_t address, uint32_t size, uint8_t fc);
    uint32_t ps32_do_read_access_2s(uint32_t address, uint32_t size, uint8_t fc);

    void ps32_do_write_access_1s(uint32_t address, uint32_t data, uint32_t size, uint8_t fc);
    void ps32_do_write_access_2s(uint32_t address, uint32_t data, uint32_t size, uint8_t fc);

    Vpistorm* dut_;
    TickFunc  tick_;

    bool use_2slot_ = true;
    int next_slot_ = 0;
    int slot_active_[2] = {0, 0};
    int write_pending_1s_ = 0;
};

#endif // PS_PI_MODEL_H

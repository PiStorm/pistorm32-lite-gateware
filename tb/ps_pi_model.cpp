#include "ps_pi_model.h"
#include "Vpistorm.h"
#include "Vpistorm_golden.h"
#include "Vpistorm.h"
#include <iostream>

template <typename TDut>
PiStormPiModelT<TDut>::PiStormPiModelT(TDut* dut, TickFunc tick_fn)
    : dut_(dut), tick_(tick_fn)
{
    init();
}

template <typename TDut>
PiStormPiModelT<TDut>::~PiStormPiModelT() = default;

template <typename TDut>
void PiStormPiModelT<TDut>::init() {
    dut_->PI_RD = 1;
    dut_->PI_WR = 1;
    dut_->PI_A = 0;
    dut_->PI_D_IN = 0;
    dut_->PI_SER_DAT = 0;
    dut_->PI_SER_CLK = 0;

    next_slot_ = 0;
    slot_active_[0] = 0;
    slot_active_[1] = 0;
    write_pending_1s_ = 0;
}

template <typename TDut>
void PiStormPiModelT<TDut>::write_ps_reg(uint32_t address, uint16_t data) {
    dut_->PI_A = address & 7;
    dut_->PI_D_IN = data;
    dut_->PI_WR = 1;
    tick_(2);

    // Active low pulse on PI_WR (matching Pi GPIO clear/set sequence)
    dut_->PI_WR = 0;
    tick_(4);

    dut_->PI_WR = 1;
    tick_(2);
}

template <typename TDut>
uint16_t PiStormPiModelT<TDut>::read_ps_reg(uint32_t address) {
    dut_->PI_A = address & 7;
    dut_->PI_RD = 1;
    dut_->PI_WR = 1;
    tick_(2);

    // Active low pulse on PI_RD
    dut_->PI_RD = 0;
    tick_(4);

    uint16_t data = dut_->PI_D_OUT;

    dut_->PI_RD = 1;
    tick_(2);

    return data;
}

template <typename TDut>
bool PiStormPiModelT<TDut>::wait_txn(int timeout_cycles) {
    int count = 0;
    while (dut_->PI_TXN_IN_PROGRESS && count < timeout_cycles) {
        tick_(2);
        count++;
    }
    return (count < timeout_cycles);
}

template <typename TDut>
void PiStormPiModelT<TDut>::ps_set_control(uint16_t value) {
    write_ps_reg(REG_CONTROL, 0x8000 | (value & 0x7FFF));
}

template <typename TDut>
void PiStormPiModelT<TDut>::ps_clr_control(uint16_t value) {
    write_ps_reg(REG_CONTROL, value & 0x7FFF);
}

template <typename TDut>
uint16_t PiStormPiModelT<TDut>::read_status() {
    return read_ps_reg(REG_STATUS);
}

template <typename TDut>
void PiStormPiModelT<TDut>::set_use_2slot(bool enable) {
    use_2slot_ = enable;
    if (use_2slot_) {
        ps_clr_control(CONTROL_INC_EXEC_SLOT);
        write_ps_reg(REG_SLOT, 0);
        ps_set_control(CONTROL_INC_EXEC_SLOT);
        slot_active_[0] = 0;
        slot_active_[1] = 0;
        next_slot_ = 0;
    } else {
        ps_clr_control(CONTROL_INC_EXEC_SLOT);
        write_ps_reg(REG_SLOT, 0);
        write_pending_1s_ = 0;
    }
}

template <typename TDut>
void PiStormPiModelT<TDut>::flush_pending_writes() {
    if (use_2slot_) {
        for (int s = 0; s < 2; s++) {
            if (slot_active_[s]) {
                write_ps_reg(REG_SLOT, s);
                wait_txn();
                slot_active_[s] = 0;
            }
        }
    } else {
        if (write_pending_1s_) {
            wait_txn();
            write_pending_1s_ = 0;
        }
    }
}

template <typename TDut>
void PiStormPiModelT<TDut>::set_serial(bool dat, bool clk) {
    dut_->PI_SER_DAT = dat ? 1 : 0;
    dut_->PI_SER_CLK = clk ? 1 : 0;
    tick_(1);
}

// Single-slot read (fallback / legacy mode)
template <typename TDut>
uint32_t PiStormPiModelT<TDut>::ps32_do_read_access_1s(uint32_t address, uint32_t size, uint8_t fc) {
    write_ps_reg(REG_ADDR_LO, address & 0xFFFF);
    if (write_pending_1s_) wait_txn();

    write_ps_reg(REG_ADDR_HI, TXN_READ | (fc << TXN_FC_SHIFT) | (size << TXN_SIZE_SHIFT) | ((address >> 16) & 0xFF));

    wait_txn();

    uint32_t data = read_ps_reg(REG_DATA_LO);
    if (size == SIZE_BYTE) {
        data &= 0xFF;
    } else if (size == SIZE_LONG) {
        data |= (uint32_t(read_ps_reg(REG_DATA_HI)) << 16);
    }

    write_pending_1s_ = 0;
    return data;
}

// 2-slot read (pipelined mode)
template <typename TDut>
uint32_t PiStormPiModelT<TDut>::ps32_do_read_access_2s(uint32_t address, uint32_t size, uint8_t fc) {
    write_ps_reg(REG_SLOT, next_slot_);
    if (slot_active_[next_slot_]) {
        wait_txn();
    }

    write_ps_reg(REG_ADDR_LO, address & 0xFFFF);
    write_ps_reg(REG_ADDR_HI, TXN_READ | (fc << TXN_FC_SHIFT) | (size << TXN_SIZE_SHIFT) | ((address >> 16) & 0xFF));

    wait_txn();

    uint32_t data = read_ps_reg(REG_DATA_LO);
    if (size == SIZE_BYTE) {
        data &= 0xFF;
    } else if (size == SIZE_LONG) {
        data |= (uint32_t(read_ps_reg(REG_DATA_HI)) << 16);
    }

    slot_active_[next_slot_] = 0;
    next_slot_ = (next_slot_ + 1) & 1;

    return data;
}

// Single-slot write
template <typename TDut>
void PiStormPiModelT<TDut>::ps32_do_write_access_1s(uint32_t address, uint32_t data, uint32_t size, uint8_t fc) {
    write_ps_reg(REG_DATA_LO, data & 0xFFFF);
    if (size == SIZE_LONG) {
        write_ps_reg(REG_DATA_HI, (data >> 16) & 0xFFFF);
    }

    write_ps_reg(REG_ADDR_LO, address & 0xFFFF);
    if (write_pending_1s_) wait_txn();

    write_ps_reg(REG_ADDR_HI, TXN_WRITE | (fc << TXN_FC_SHIFT) | (size << TXN_SIZE_SHIFT) | ((address >> 16) & 0xFF));

    if (address >= 0x00BF0000 && address <= 0x00DFFFFF) {
        wait_txn();
        write_pending_1s_ = 0;
    } else {
        write_pending_1s_ = 1;
    }
}

// 2-slot write
template <typename TDut>
void PiStormPiModelT<TDut>::ps32_do_write_access_2s(uint32_t address, uint32_t data, uint32_t size, uint8_t fc) {
    write_ps_reg(REG_SLOT, next_slot_);
    if (slot_active_[next_slot_]) {
        wait_txn();
    }

    write_ps_reg(REG_DATA_LO, data & 0xFFFF);
    if (size == SIZE_LONG) {
        write_ps_reg(REG_DATA_HI, (data >> 16) & 0xFFFF);
    }

    write_ps_reg(REG_ADDR_LO, address & 0xFFFF);
    write_ps_reg(REG_ADDR_HI, TXN_WRITE | (fc << TXN_FC_SHIFT) | (size << TXN_SIZE_SHIFT) | ((address >> 16) & 0xFF));

    if (address >= 0x00BF0000 && address <= 0x00DFFFFF) {
        wait_txn();
        slot_active_[next_slot_] = 0;
    } else {
        slot_active_[next_slot_] = 1;
    }

    next_slot_ = (next_slot_ + 1) & 1;
}

template <typename TDut>
uint8_t PiStormPiModelT<TDut>::ps32_read_8(uint32_t address, uint8_t fc) {
    return use_2slot_ ? ps32_do_read_access_2s(address, SIZE_BYTE, fc)
                      : ps32_do_read_access_1s(address, SIZE_BYTE, fc);
}

template <typename TDut>
uint16_t PiStormPiModelT<TDut>::ps32_read_16(uint32_t address, uint8_t fc) {
    return use_2slot_ ? ps32_do_read_access_2s(address, SIZE_WORD, fc)
                      : ps32_do_read_access_1s(address, SIZE_WORD, fc);
}

template <typename TDut>
uint32_t PiStormPiModelT<TDut>::ps32_read_32(uint32_t address, uint8_t fc) {
    return use_2slot_ ? ps32_do_read_access_2s(address, SIZE_LONG, fc)
                      : ps32_do_read_access_1s(address, SIZE_LONG, fc);
}

template <typename TDut>
uint64_t PiStormPiModelT<TDut>::ps32_read_64(uint32_t address, uint8_t fc) {
    if (use_2slot_) {
        // Pipelined 64-bit read across both slots
        write_ps_reg(REG_SLOT, next_slot_);
        if (slot_active_[next_slot_]) wait_txn();

        write_ps_reg(REG_ADDR_LO, address & 0xFFFF);
        write_ps_reg(REG_ADDR_HI, TXN_READ | (fc << TXN_FC_SHIFT) | (SIZE_LONG << TXN_SIZE_SHIFT) | ((address >> 16) & 0xFF));

        wait_txn();

        uint64_t data = (uint64_t(read_ps_reg(REG_DATA_HI)) << 48);
        data |= (uint64_t(read_ps_reg(REG_DATA_LO)) << 32);

        slot_active_[next_slot_] = 0;
        next_slot_ = (next_slot_ + 1) & 1;

        address += 4;
        write_ps_reg(REG_SLOT, next_slot_);
        write_ps_reg(REG_ADDR_LO, address & 0xFFFF);
        write_ps_reg(REG_ADDR_HI, TXN_READ | (fc << TXN_FC_SHIFT) | (SIZE_LONG << TXN_SIZE_SHIFT) | ((address >> 16) & 0xFF));

        wait_txn();

        slot_active_[next_slot_] = 0;
        next_slot_ = (next_slot_ + 1) & 1;

        data |= (uint64_t(read_ps_reg(REG_DATA_HI)) << 16);
        data |= uint64_t(read_ps_reg(REG_DATA_LO));
        return data;
    } else {
        uint64_t d0 = ps32_do_read_access_1s(address, SIZE_LONG, fc);
        uint64_t d1 = ps32_do_read_access_1s(address + 4, SIZE_LONG, fc);
        return (d0 << 32) | d1;
    }
}

template <typename TDut>
uint128_t PiStormPiModelT<TDut>::ps32_read_128(uint32_t address, uint8_t fc) {
    uint128_t res;
    if (use_2slot_) {
        // First 64-bit chunk
        res.hi = ps32_read_64(address, fc);
        // Second 64-bit chunk
        res.lo = ps32_read_64(address + 8, fc);
    } else {
        res.hi = (uint64_t(ps32_do_read_access_1s(address, SIZE_LONG, fc)) << 32) |
                  ps32_do_read_access_1s(address + 4, SIZE_LONG, fc);
        res.lo = (uint64_t(ps32_do_read_access_1s(address + 8, SIZE_LONG, fc)) << 32) |
                  ps32_do_read_access_1s(address + 12, SIZE_LONG, fc);
    }
    return res;
}

template <typename TDut>
void PiStormPiModelT<TDut>::ps32_write_8(uint32_t address, uint8_t data, uint8_t fc) {
    if (use_2slot_) {
        ps32_do_write_access_2s(address, data, SIZE_BYTE, fc);
    } else {
        ps32_do_write_access_1s(address, data, SIZE_BYTE, fc);
    }
}

template <typename TDut>
void PiStormPiModelT<TDut>::ps32_write_16(uint32_t address, uint16_t data, uint8_t fc) {
    if (use_2slot_) {
        ps32_do_write_access_2s(address, data, SIZE_WORD, fc);
    } else {
        ps32_do_write_access_1s(address, data, SIZE_WORD, fc);
    }
}

template <typename TDut>
void PiStormPiModelT<TDut>::ps32_write_32(uint32_t address, uint32_t data, uint8_t fc) {
    if (use_2slot_) {
        ps32_do_write_access_2s(address, data, SIZE_LONG, fc);
    } else {
        ps32_do_write_access_1s(address, data, SIZE_LONG, fc);
    }
}

template <typename TDut>
void PiStormPiModelT<TDut>::ps32_write_64(uint32_t address, uint64_t data, uint8_t fc) {
    if (use_2slot_) {
        // First long word
        write_ps_reg(REG_SLOT, next_slot_);
        if (slot_active_[next_slot_]) wait_txn();

        write_ps_reg(REG_DATA_LO, (data >> 32) & 0xFFFF);
        write_ps_reg(REG_DATA_HI, (data >> 48) & 0xFFFF);
        write_ps_reg(REG_ADDR_LO, address & 0xFFFF);
        write_ps_reg(REG_ADDR_HI, TXN_WRITE | (fc << TXN_FC_SHIFT) | (SIZE_LONG << TXN_SIZE_SHIFT) | ((address >> 16) & 0xFF));

        slot_active_[next_slot_] = 1;
        next_slot_ = (next_slot_ + 1) & 1;

        // Second long word
        address += 4;
        write_ps_reg(REG_SLOT, next_slot_);
        if (slot_active_[next_slot_]) wait_txn();

        write_ps_reg(REG_DATA_LO, data & 0xFFFF);
        write_ps_reg(REG_DATA_HI, (data >> 16) & 0xFFFF);
        write_ps_reg(REG_ADDR_LO, address & 0xFFFF);
        write_ps_reg(REG_ADDR_HI, TXN_WRITE | (fc << TXN_FC_SHIFT) | (SIZE_LONG << TXN_SIZE_SHIFT) | ((address >> 16) & 0xFF));

        if (address >= 0x00BF0000 && address <= 0x00DFFFFF) {
            wait_txn();
            slot_active_[next_slot_] = 0;
        } else {
            slot_active_[next_slot_] = 1;
        }
        next_slot_ = (next_slot_ + 1) & 1;
    } else {
        ps32_do_write_access_1s(address, (data >> 32) & 0xFFFFFFFF, SIZE_LONG, fc);
        ps32_do_write_access_1s(address + 4, data & 0xFFFFFFFF, SIZE_LONG, fc);
    }
}

template <typename TDut>
void PiStormPiModelT<TDut>::ps32_write_128(uint32_t address, uint128_t data, uint8_t fc) {
    ps32_write_64(address, data.hi, fc);
    ps32_write_64(address + 8, data.lo, fc);
}

// Explicit template instantiations
template class PiStormPiModelT<Vpistorm>;
template class PiStormPiModelT<Vpistorm_golden>;

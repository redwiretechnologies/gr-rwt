#include "cic_filter_fpga.h"

namespace gr {
namespace rwt {

cic_filter_fpga::cic_filter_fpga(
    std::shared_ptr<base_device_common> common,
    uint32_t filter_id,
    bool debug,
    bool always_write) :
    rwt_registers_common(common, debug),
    m_filter_id(filter_id),
    m_always_write(always_write),
    decimation_stage(0),
    decimation_arbitrary(0),
    decimation_coefficient(0),
    correction_enable(0)
{}

void
cic_filter_fpga::set_filter_id()
{
    m_common->write_reg(REG_BLK_CIC, REG_CIC_FILTER_ID, m_filter_id);
}

void
cic_filter_fpga::set_decimation(uint32_t value_u32)
{
    decimation_stage = value_u32;
    if(m_debug) printf("  Set decimation reg to %d\n", decimation_stage);

    if (decimation_stage == (uint32_t) CIC_DEC_1e1) {
        decimation_coefficient = (uint32_t) CIC_COEFF_1e1;
    }else if (decimation_stage == (uint32_t) CIC_DEC_1e2) {
        decimation_coefficient = (uint32_t) CIC_COEFF_1e2;
    }else if (decimation_stage == (uint32_t) CIC_DEC_1e3) {
        decimation_coefficient = (uint32_t) CIC_COEFF_1e3;
    }else if (decimation_stage == (uint32_t) CIC_DEC_1e4) {
        decimation_coefficient = (uint32_t) CIC_COEFF_1e4;
    }else if (decimation_stage == (uint32_t) CIC_DEC_1e5) {
        decimation_coefficient = (uint32_t) CIC_COEFF_1e5;
    }else {
        decimation_coefficient = (uint32_t) CIC_COEFF_1e0;
    }
    if (m_always_write)
        write_cic_params();
}

void
cic_filter_fpga::set_decimation_arbitrary(uint32_t value_u32)
{
    decimation_arbitrary = value_u32;
    if (m_always_write)
        write_cic_params();
}

void
cic_filter_fpga::set_correction_enable(uint32_t value_u32)
{
    correction_enable = value_u32;
    if (m_always_write)
        write_cic_params();
}

void
cic_filter_fpga::set_bypass_enable(uint32_t value_u32)
{
    bypass_enable = (value_u32 == 1);
    if (m_always_write)
        write_cic_params();
}

void
cic_filter_fpga::write_cic_params() {
    set_filter_id();
    m_common->write_reg(REG_BLK_CIC, REG_CIC_DECIMATION_STAGE_EN, decimation_stage);
    m_common->write_reg(REG_BLK_CIC, REG_CIC_CORRECTION_COEFF_A, decimation_coefficient);
    m_common->write_reg(REG_BLK_CIC, REG_CIC_CORRECTION_COEFF_B, decimation_coefficient);
    m_common->write_reg(REG_BLK_CIC, REG_CIC_CORRECTION_EN, correction_enable);
    m_common->write_reg(REG_BLK_CIC, REG_CIC_DECIMATION_RAT, decimation_arbitrary);
    reset();
}

int
cic_filter_fpga::handle_registers (
    const std::string key,
    const std::string value)
{
    uint32_t value_u32;

    if (key == "decimation_arbitrary") {
        value_u32 = (uint32_t)strtoul(value.c_str(), NULL, 0);
        set_decimation_arbitrary(value_u32);
    }else if (key == "decimation_ratio") {
        value_u32 = (uint32_t)strtoul(value.c_str(), NULL, 0);
        set_decimation(value_u32);
    }else if (key == "correction_enable") {
        value_u32 = (uint32_t)strtoul(value.c_str(), NULL, 0);
        set_correction_enable(value_u32);
    }else if (key == "reset") {
        reset();
    }else if (key == "bypass_enable") {
        value_u32 = (uint32_t)strtoul(value.c_str(), NULL, 0);
        set_bypass_enable(value_u32);
    }else if (key == "write_cic_params") {
        write_cic_params();
    }
    else
        return -1;
    return 0;
}

void
cic_filter_fpga::reset() {
    m_common->write_reg(REG_BLK_CIC, REG_CIC_RESET, 0xFFFF);
    if(!bypass_enable)
        m_common->write_reg(REG_BLK_CIC, REG_CIC_RESET, 0x0000);
}

void
cic_filter_fpga::read_registers() {
    read_help(REG_BLK_CIC, REG_CIC_DECIMATION_RAT, "DECIMATION RATIO");
    read_help(REG_BLK_CIC, REG_CIC_DECIMATION_STAGE_EN, "DECIMATION ENABLE");
    read_help(REG_BLK_CIC, REG_CIC_CORRECTION_EN, "CORRECTION ENABLE");
    read_help(REG_BLK_CIC, REG_CIC_CORRECTION_COEFF_A, "CORRECTION A COEFFICIENT");
    read_help(REG_BLK_CIC, REG_CIC_CORRECTION_COEFF_B, "CORRECTION B COEFFICIENT");
    read_help(REG_BLK_CIC, REG_CIC_RESET, "CIC RESET");
    read_help(REG_BLK_CIC, REG_CIC_FILTER_ID, "CIC FILTER ID");
}

void
cic_filter_fpga::test_registers() {
    reg_test_help(REG_BLK_CIC, REG_CIC_DECIMATION_RAT, 3, "decimation ratio");
    reg_test_help(REG_BLK_CIC, REG_CIC_DECIMATION_STAGE_EN, 2, "decimation enable");
    reg_test_help(REG_BLK_CIC, REG_CIC_CORRECTION_EN, 3, "correction enable");
    reg_test_help(REG_BLK_CIC, REG_CIC_CORRECTION_COEFF_A, 4, "correction A coefficient");
    reg_test_help(REG_BLK_CIC, REG_CIC_CORRECTION_COEFF_B, 5, "correction B coefficient");
    reg_test_help(REG_BLK_CIC, REG_CIC_RESET, 1, "cic reset");
    reg_test_help(REG_BLK_CIC, REG_CIC_FILTER_ID, 10, "cic filter id");
}

}
}

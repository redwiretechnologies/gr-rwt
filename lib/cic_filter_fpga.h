#include "rwt_registers_common.h"
#include "common_registers.h"

#ifndef INCLUDED_RWT_CIC_FILTER_FPGA_H
#define INCLUDED_RWT_CIC_FILTER_FPGA_H

namespace gr {
namespace rwt {

#define REG_CIC_BLKID                REG_USER_BLKID

#define REG_BLK_CIC                  REG_CIC_BLKID

#define REG_CIC_RESET                12
#define REG_CIC_DECIMATION_RAT       13
#define REG_CIC_DECIMATION_STAGE_EN  14
#define REG_CIC_CORRECTION_EN        15
#define REG_CIC_CORRECTION_COEFF_A   16
#define REG_CIC_CORRECTION_COEFF_B   17
#define REG_CIC_FILTER_ID            18

#define CIC_DEC_1e0                  0
#define CIC_DEC_1e1                  1
#define CIC_DEC_1e2                  2
#define CIC_DEC_1e3                  3
#define CIC_DEC_1e4                  6
#define CIC_DEC_1e5                  7

/* Define Correction Coefficients for each value of decimation
 *   Correction coefficients are 16 bits wide
 *   They are of the form 1.1.14 (sign, integer, fractional bits)
 */
#define CIC_COEFF_1e0                0b0100000000000000         //1.00
#define CIC_COEFF_1e1                0b0100001100110011         //1.05
#define CIC_COEFF_1e2                0b0100011001100110         //1.10
#define CIC_COEFF_1e3                0b0100100110011001         //1.15
#define CIC_COEFF_1e4                0b0100110011001100         //1.20
#define CIC_COEFF_1e5                0b0101000010100011         //1.26

class cic_filter_fpga : public rwt_registers_common
{
public:
    cic_filter_fpga(
        std::shared_ptr<base_device_common> common,
        uint32_t filter_id,
        bool debug,
        bool always_write);
    int handle_registers(
        const std::string key,
        const std::string value);
    void read_registers();
    void test_registers();
    void set_decimation(uint32_t value_u32);
    void reset();

    const char *handlers[6] = {
        "decimation_arbitrary",
        "decimation_ratio",
        "correction_enable",
        "reset",
	    "bypass_enable",
        "write_cic_params"
    };

private:
    void set_decimation_arbitrary(uint32_t value_u32);
    void set_correction_enable(uint32_t value_u32);
    void set_bypass_enable(uint32_t value_u32);
    void set_filter_id();
    void write_cic_params();

    uint32_t m_filter_id;
    uint32_t decimation_arbitrary;
    uint32_t decimation_stage;
    uint32_t decimation_coefficient;
    uint32_t correction_enable;
    bool bypass_enable;
    bool m_always_write;
};

}
}

#endif


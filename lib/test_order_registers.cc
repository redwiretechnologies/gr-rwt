#include "test_order_registers.h"

namespace gr {
namespace rwt {

test_order_registers::test_order_registers(
    std::shared_ptr<base_device_common> common,
    bool debug) :
    rwt_registers_common(common, debug),
    test_order(true)
{}

void
test_order_registers::set_test_order(uint32_t value_u32)
{
    test_order = (value_u32 == 1);
    m_common->write_reg(REG_BLK_USER, REG_TEST_ORDER, test_order);
}

int
test_order_registers::handle_registers (
    const std::string key,
    const std::string value)
{
    uint32_t value_u32;

    if (key == "test_order") {
        value_u32 = (uint32_t)strtoul(value.c_str(), NULL, 0);
        set_test_order(value_u32);
    } else if (key == "count_probe") {
        read_help(REG_BLK_USER, REG_COUNT_PROBE, "COUNT PROBE");
    } else
        return -1;
    return 0;
}

void
test_order_registers::read_registers() {
    read_help(REG_BLK_USER, REG_TEST_ORDER, "TEST ORDER");
    read_help(REG_BLK_USER, REG_COUNT_PROBE, "COUNT PROBE");
}

void
test_order_registers::test_registers() {
    reg_test_help(REG_BLK_USER, REG_TEST_ORDER, 1, "test order");
}

}
}

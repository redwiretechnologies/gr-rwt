#include "rwt_registers_common.h"
#include "common_registers.h"

#ifndef INCLUDED_RWT_TEST_ORDER_REGISTERS_H
#define INCLUDED_RWT_TEST_ORDER_REGISTERS_H

namespace gr {
namespace rwt {

#define REG_TEST_ORDER  19
#define REG_COUNT_PROBE 20

class test_order_registers : public rwt_registers_common
{
public:
    test_order_registers(
        std::shared_ptr<base_device_common> common,
        bool debug);
    int handle_registers(
        const std::string key,
        const std::string value);
    void read_registers();
    void test_registers();

    const char *handlers[1] = {
        "test_order"
    };

private:
    void set_test_order(uint32_t value_u32);

    bool test_order;
};

}
}

#endif


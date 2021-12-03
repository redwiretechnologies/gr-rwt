#include "base_device_common.h"
#include <vector>
#include <string>
#include <sstream>
#include <map>

#ifndef INCLUDED_RWT_REGISTERS_COMMON_CLASS_H
#define INCLUDED_RWT_REGISTERS_COMMON_CLASS_H

namespace gr {
namespace rwt {

class rwt_registers_common
{
public:
    rwt_registers_common(
        std::shared_ptr<base_device_common> common,
        bool debug);

    void reg_test_help(
        uint32_t reg_blk,
        uint32_t reg_offset,
        uint32_t write_val,
        std::string name);
    void read_help(
        uint32_t reg_blk,
        uint32_t reg_offset,
        std::string name);

	void parse_list(
        std::string value,
        std::string type,
        void *vec);

    void pmt_to_map(
        std::map<std::string, std::string> *m,
        pmt::pmt_t p);

    const char *handlers[5] = {
        "use_ext_pps",
        "time_next_pps",
        "time_now",
        "sample_idx_next_pps",
        "sample_idx_now"
    };

    std::shared_ptr<base_device_common> m_common;
    bool m_debug;
};

}
}

#endif

#include "rwt_registers_common.h"

namespace gr {
namespace rwt {

rwt_registers_common::rwt_registers_common(
    std::shared_ptr<base_device_common> common,
    bool debug) :
    m_common(common),
    m_debug(debug)
{}

void
rwt_registers_common::reg_test_help(uint32_t reg_blk, uint32_t reg_offset, uint32_t write_val, std::string name) {
    uint32_t value_u32, temp;

    temp = m_common->read_reg(reg_blk, reg_offset);
    m_common->write_reg(reg_blk, reg_offset, write_val);
    value_u32 = m_common->read_reg(reg_blk, reg_offset);
    printf("%d is in the %s register. it should be %d\n", value_u32, name.c_str(), write_val);
    m_common->write_reg(reg_blk, reg_offset, temp);
}

void
rwt_registers_common::read_help(uint32_t reg_blk, uint32_t reg_offset, std::string name) {
    uint32_t value_u32;
    value_u32 = m_common->read_reg(reg_blk, reg_offset);
    printf("%d is in the %s register\n", value_u32, name.c_str());
}

void
rwt_registers_common::parse_list(std::string value, std::string type, void *vec) {

    std::stringstream ss(value);

    if(type == "string") {
        std::vector<std::string> *inside_vec;
        inside_vec = (std::vector<std::string> *) vec;
        inside_vec->resize(0);
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ' ');
            inside_vec->push_back(substr);
        }
    }
    else if(type == "float") {
        std::vector<float> *inside_vec;
        inside_vec = (std::vector<float> *) vec;
        inside_vec->resize(0);
        float tmp;
        while (ss.good()) {
            ss >> tmp;
            inside_vec->push_back(tmp);
        }
    }
    else if(type == "int") {
        std::vector<int> *inside_vec;
        inside_vec = (std::vector<int> *) vec;
        inside_vec->resize(0);
        int tmp;
        while (ss.good()) {
            ss >> tmp;
            inside_vec->push_back(tmp);
        }
    }
    else if(type == "hex") {
        std::vector<uint32_t> *inside_vec;
        inside_vec = (std::vector<uint32_t> *) vec;
        inside_vec->resize(0);
        uint32_t tmp;
        while (ss.good()) {
            ss >> std::hex >> tmp;
            inside_vec->push_back(tmp);
        }
    }
    else if(type == "bool") {
        std::vector<bool> *inside_vec;
        inside_vec = (std::vector<bool> *) vec;
        inside_vec->resize(0);
        int tmp;
        while (ss.good()) {
            ss >> tmp;
            inside_vec->push_back((tmp != 0));
        }
    }
}

void
rwt_registers_common::pmt_to_map(std::map<std::string, std::string> *m, pmt::pmt_t p)
{
    /* If the PMT is a list, assume it's a list of pairs and recurse for each */
    /* Works for dict too */
    try {
        /* Because of PMT is just broken you and can't distinguish between
         * pair and dict, we have to call length() and see if it will throw
         * or not ... */
        if (pmt::length(p) > 0) {
            for (int i=0; i<pmt::length(p); i++)
                pmt_to_map(m, pmt::nth(i, p));
            return;
        }
    } catch(...) { }

    if (pmt::is_pair(p)) {
        pmt::pmt_t key(pmt::car(p));
        pmt::pmt_t val(pmt::cdr(p));

        if (!pmt::is_symbol(key) || !pmt::is_symbol(val))
            return;

        m->insert(std::pair<std::string, std::string>(pmt::symbol_to_string(key), pmt::symbol_to_string(val)));
    }
}

}
}

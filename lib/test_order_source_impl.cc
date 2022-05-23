/* -*- c++ -*- */
/*
 * Copyright 2019 $Red Wire Technologies.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include "test_order_source_impl.h"
#include "common_tag_types.h"
#include "common_registers.h"

namespace gr {
namespace rwt {

#define BLK_ID 0x0002
#define BLK_VER_MIN 0x0002
#define BLK_VER_MAX 0x0002

#define DEBUG 0

test_order_source::sptr
test_order_source::make(
    const pmt::pmt_t config,
    unsigned int reg_base_addr,
    const char *filter,
    bool auto_filter,
    const char *personality,
    bool force_reload,
    unsigned int buffer_size,
    const char *phy_name,
    const char *rx_name,
    const char *tx_name)
{
  return gnuradio::get_initial_sptr
    (new test_order_source_impl(
        config,
        reg_base_addr,
        filter,
        auto_filter,
        personality,
        force_reload,
        buffer_size,
        phy_name,
        rx_name,
        tx_name));
}


/*
 * The private constructor
 */
test_order_source_impl::test_order_source_impl(
    const pmt::pmt_t config,
    unsigned int reg_base_addr,
    const char *filter,
    bool auto_filter,
    const char *personality,
    bool force_reload,
    unsigned int buffer_size,
    const char *phy_name,
    const char *rx_name,
    const char *tx_name) :
    rwt_base_block(
        "test_order_source",
        gr::io_signature::make(0, 0, 0),
        gr::io_signature::make(
            1, 1,
            sizeof(uint32_t))),
    rwt_base_block_impl(
        "test_order_source",
        gr::io_signature::make(0, 0, 0),
        gr::io_signature::make(
            1, 1,
            sizeof(uint32_t)),
        buffer_size,
        0,
        reg_base_addr,
        true,
        false,
        personality,
        force_reload,
        phy_name,
        rx_name,
        tx_name),
    m_escape(0xaaaaaaaaaaaaaaaa),
    m_in_constructor(true),
    m_autofilter(auto_filter),
    m_samplerate(5000000),
    m_alloc_len(0),
    m_pkt_data(NULL)
{

    common_registers_rwt = new rwt_registers_common(m_common, DEBUG);
    cic_frontend = new cic_filter_fpga(m_common, 0, DEBUG, true);
    test_order = new test_order_registers(m_common, DEBUG);

    const char *handlers[] = {
        "escape",
        "samplerate"
    };

    const int alignment_multiple =
        volk_get_alignment() / sizeof(uint32_t);
    set_alignment(std::max(1, alignment_multiple));
    set_output_multiple(2);

    for (int i = 0; i < (sizeof(handlers)/sizeof(char *)); i++) {
        set_config_handler(
            handlers[i],
            boost::bind(&test_order_source_impl::ctrl_reg_handler, this, _1, _2));
    }
    for (int i = 0; i < (sizeof(common_registers_rwt->handlers)/sizeof(char *)); i++) {
        set_config_handler(
            common_registers_rwt->handlers[i],
            boost::bind(&test_order_source_impl::ctrl_reg_handler, this, _1, _2));
    }
    for (int i = 0; i < (sizeof(cic_frontend->handlers)/sizeof(char *)); i++) {
        set_config_handler(
            cic_frontend->handlers[i],
            boost::bind(&test_order_source_impl::ctrl_reg_handler, this, _1, _2));
    }
    for (int i = 0; i < (sizeof(test_order->handlers)/sizeof(char *)); i++) {
        set_config_handler(
            test_order->handlers[i],
            boost::bind(&test_order_source_impl::ctrl_reg_handler, this, _1, _2));
    }

    m_common->check_user_blkid(
        BLK_ID,
        BLK_VER_MIN,
        BLK_VER_MAX,
        true);

    m_time_helper.setup(m_common, false);
    m_source->setup_tags(false, m_escape);
    config_msg_handler(config);

    if (!m_common->setup_filter(filter, m_samplerate, m_autofilter))
        throw std::runtime_error("Unable to set filter");
    cic_frontend->reset();
    cic_frontend->handle_registers("bypass_enable", "1");
    cic_frontend->handle_registers("write_cic_params", "");
    test_order->handle_registers("test_order", "1");
    m_in_constructor = false;
}

/*
 * Our virtual destructor.
 */
test_order_source_impl::~test_order_source_impl()
{
    free_volk_buffers();
    free(common_registers_rwt);
    free(cic_frontend);
}

void
test_order_source_impl::set_samplerate(const std::string value)
{
    unsigned long val;
    int count;
    std::string s = "";
    val = (unsigned long)strtoul(value.c_str(), NULL, 0);

    for(count=0; count<8; count++) {
        if(val < MIN_SAMPLE_RATE) {
            val *= 10;
            if(count == 3)
                count = 5;
            s += "0";
        }
        else
            break;
    }

    if(count == 8) {
        printf("Can't set samplerate to %s!!!\n", value.c_str());
        return;
    }

    const std::string val2 = value + s;
    m_common->set_attr("samplerate", val2);
    if(DEBUG) printf("  Set samplerate to %ld\n", val);
    cic_frontend->set_decimation(count);

    m_samplerate = val;
    if (m_autofilter && !m_in_constructor) {
        if(!m_common->setup_filter(NULL, m_samplerate, m_autofilter))
            std::cerr << "Error setting filter for AD9361.\n";
    }
}

void
test_order_source_impl::ctrl_reg_handler(
    const std::string key,
    const std::string value)
{

    uint32_t value_u32;

    if (key == "samplerate") {
        set_samplerate(value);
    }else if (key == "escape") {
        m_escape = (uint64_t)strtoull(value.c_str(), NULL, 0);;
        m_source->setup_tags(false, m_escape);
    }else if (cic_frontend->handle_registers(key, value) == 0) {
        return;
    }else if (test_order->handle_registers(key, value) == 0) {
        return;
    } else {
        m_time_helper.ctrl_reg_handler(key, value, m_samplerate);
    }
}

pmt::pmt_t
test_order_source_impl::get_time()
{
    return m_time_helper.get_time_pmt(
        time_helper::INDEX_ADC,
        m_samplerate);
}


uint64_t
test_order_source_impl::get_sample_idx()
{
    return m_time_helper.get_sample_idx(time_helper::INDEX_ADC);
}


void
test_order_source_impl::alloc_volk_buffers(int nitems)
{
    const unsigned int alignment = volk_get_alignment();

    free_volk_buffers();

    m_pkt_data = (uint32_t *)volk_malloc(sizeof(uint32_t) * nitems, alignment);

    if (!m_pkt_data) {
        throw std::runtime_error(
            "test_order_source: Failed to allocate volk buffers");
    }

    m_alloc_len = nitems;
}


int
test_order_source_impl::work(
    int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    bool next_tag_valid;
    uint8_t next_tag_type;
    uint64_t next_tag_value;
    size_t max_pkt_size;
    size_t pkt_idx = 0;
    size_t pkt_read_count;
    int ret;
    gr::tag_t tag;

    max_pkt_size = noutput_items;

    if (m_alloc_len < max_pkt_size) {
        alloc_volk_buffers(max_pkt_size);
    }

    while (pkt_idx < max_pkt_size) {
        ret = m_source->read_until_next_tag_or_end(
            false,
            &m_pkt_data[pkt_idx],
            4,
            max_pkt_size - pkt_idx,
            &next_tag_valid,
            &next_tag_type,
            &next_tag_value,
            &pkt_read_count);

        if (ret == -ETIMEDOUT) {
            std::cerr << "Warning: test_order_source: timed out.\n";
            break;
        } else if (ret < 0) {
            std::cerr << "Warning: iio:source Buffer refill failed.\n";
            return -1;
        }

        if (!pkt_read_count && pkt_idx) {
            /* Didn't read anything last try, but data is already available.
               So go ahead and send it. */
            break;
        }

        pkt_idx += pkt_read_count;

        if (next_tag_valid) {
            switch (next_tag_type) {
            case RWT_TAG_IDX_TIME:
                tag.key = RWT_TAG_PMT_TIME;
                break;
            case RWT_TAG_IDX_PPS:
                tag.key = RWT_TAG_PMT_PPS;
                break;
            case RWT_TAG_IDX_OVERFLOW:
                tag.key = RWT_TAG_PMT_OVERFLOW;
                break;
            default:
                next_tag_valid = false;
            }
        }

        if (next_tag_valid) {
            tag.offset = nitems_written(0) + pkt_idx / 2;

            tag.value = pmt::from_uint64(next_tag_value);
            tag.srcid = pmt::PMT_F;
            m_tags.push_back(tag);
        }
    }

    if (!pkt_idx)
        return 0;

    noutput_items = pkt_idx;
    buffer_number++;

    while (!m_tags.empty()) {
        tag = m_tags.front();

        if (tag.offset < (nitems_written(0) + noutput_items)) {
            add_item_tag(0, tag);

            m_tags.pop_front();
        } else {
            /* The list is sorted by offset, so when a tag past what's been
               sent we are done.*/
            break;
        }
    }

    int *all_output_items = (int*) output_items[0];
    for(int i=0; i<noutput_items; i++)
        all_output_items[i] = m_pkt_data[i];

    if (m_pkt_data[noutput_items-1] - m_pkt_data[0] != noutput_items) {

        for(int i=1; i<noutput_items-1; i++) {
            if(m_pkt_data[i] - m_pkt_data[i-1] != 1) {
                printf("Buffer#=%d, i=%d/%d, diff=%d\n", buffer_number,
                        i, noutput_items, m_pkt_data[i] - m_pkt_data[i-1]);
            }
        }
    }

    // Tell runtime system how many output items we produced.
    return noutput_items;
}


bool
test_order_source_impl::start()
{
    bool ret;

    ret = rwt_base_block::start();
    if (!ret)
        return ret;

    ret = m_source->start(true, false);
    if (!ret)
        return ret;

    return ret;
}


bool
test_order_source_impl::stop()
{
    m_source->stop();
    rwt_base_block::stop();
    free_volk_buffers();
    return true;
}


} /* namespace rwt */
} /* namespace gr */

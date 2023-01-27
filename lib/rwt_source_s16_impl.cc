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
#include "rwt_source_s16_impl.h"
#include "common_tag_types.h"
#include "common_registers.h"

namespace gr {
namespace rwt {

#define BLK_ID 0x0002
#define BLK_VER_MIN 0x0002
#define BLK_VER_MAX 0x0002

#define DEBUG 0
rwt_source_s16::sptr
rwt_source_s16::make(
    const pmt::pmt_t config,
    bool ch1_en,
    bool ch2_en,
    unsigned int reg_base_addr,
    const char *filter,
    bool use_tags,
    bool auto_filter,
    const char *personality,
    bool force_reload,
    unsigned int buffer_size,
    const char *phy_name = "ad9361-phy",
    const char *rx_name = "cf-ad9361-lpc",
    const char *tx_name = "cf-ad9361-dds-core-lpc")
{
  return gnuradio::get_initial_sptr
    (new rwt_source_s16_impl(
        config,
        ch1_en,
        ch2_en,
        reg_base_addr,
        filter,
        use_tags,
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
rwt_source_s16_impl::rwt_source_s16_impl(
    const pmt::pmt_t config,
    bool ch1_en,
    bool ch2_en,
    unsigned int reg_base_addr,
    const char *filter,
    bool use_tags,
    bool auto_filter,
    const char *personality,
    bool force_reload,
    unsigned int buffer_size,
    const char *phy_name,
    const char *rx_name,
    const char *tx_name) :
    rwt_base_block(
        "rwt_source_s16",
        gr::io_signature::make(0, 0, 0),
        gr::io_signature::make(
            2 * ((int)ch1_en + (int)ch2_en),
            2 * ((int)ch1_en + (int)ch2_en),
            sizeof(short))),
    rwt_base_block_impl(
        "rwt_source_s16",
        gr::io_signature::make(0, 0, 0),
        gr::io_signature::make(
            2 * ((int)ch1_en + (int)ch2_en),
            2 * ((int)ch1_en + (int)ch2_en),
            sizeof(short)),
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
    m_ch1_en(ch1_en),
    m_ch2_en(ch2_en),
    m_use_tags(use_tags),
    m_autofilter(auto_filter),
    m_samplerate(5000000),
    m_alloc_len(0),
    m_pkt_data(NULL)
{

    common_registers_rwt = new rwt_registers_common(m_common, DEBUG);
    cic_frontend = new cic_filter_fpga(m_common, 0, DEBUG, true);
    if (!ch1_en && !ch2_en) {
        throw std::runtime_error(
            "Channel 1, channel 2, or both must be enabled.");
    }

    const char *handlers[] = {
        "escape",
        "samplerate"
    };

    const int alignment_multiple =
        volk_get_alignment() / sizeof(short);
    set_alignment(std::max(1, alignment_multiple));
    set_output_multiple(8);

    m_both_ch_en = ch1_en && ch2_en;

    for (int i = 0; i < (sizeof(handlers)/sizeof(char *)); i++) {
        set_config_handler(
            handlers[i],
            std::bind(&rwt_source_s16_impl::ctrl_reg_handler, this, std::placeholders::_1, std::placeholders::_2));
    }
    for (int i = 0; i < (sizeof(common_registers_rwt->handlers)/sizeof(char *)); i++) {
        set_config_handler(
            common_registers_rwt->handlers[i],
            std::bind(&rwt_source_s16_impl::ctrl_reg_handler, this, std::placeholders::_1, std::placeholders::_2));
    }
    for (int i = 0; i < (sizeof(cic_frontend->handlers)/sizeof(char *)); i++) {
        set_config_handler(
            cic_frontend->handlers[i],
            std::bind(&rwt_source_s16_impl::ctrl_reg_handler, this, std::placeholders::_1, std::placeholders::_2));
    }

    m_common->check_user_blkid(
        BLK_ID,
        BLK_VER_MIN,
        BLK_VER_MAX,
        true);

    m_time_helper.setup(m_common, m_use_tags);

    m_source->setup_tags(m_use_tags, m_escape);
    config_msg_handler(config);

    if (!m_common->setup_filter(filter, m_samplerate, m_autofilter))
        throw std::runtime_error("Unable to set filter");
    cic_frontend->reset();
    m_in_constructor = false;
}

/*
 * Our virtual destructor.
 */
rwt_source_s16_impl::~rwt_source_s16_impl()
{
    free_volk_buffers();
    free(common_registers_rwt);
    free(cic_frontend);
}

void
rwt_source_s16_impl::set_samplerate(const std::string value)
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
rwt_source_s16_impl::ctrl_reg_handler(
    const std::string key,
    const std::string value)
{

    uint32_t value_u32;

    if (key == "samplerate") {
        set_samplerate(value);
    }else if (key == "escape") {
        m_escape = (uint64_t)strtoull(value.c_str(), NULL, 0);;
        m_source->setup_tags(m_use_tags, m_escape);
    }else if (cic_frontend->handle_registers(key, value) == 0) {
        return;
    } else {
        m_time_helper.ctrl_reg_handler(key, value, m_samplerate);
    }
}


pmt::pmt_t
rwt_source_s16_impl::get_time()
{
    return m_time_helper.get_time_pmt(
        time_helper::INDEX_ADC,
        m_samplerate);
}


uint64_t
rwt_source_s16_impl::get_sample_idx()
{
    return m_time_helper.get_sample_idx(time_helper::INDEX_ADC);
}


void
rwt_source_s16_impl::alloc_volk_buffers(int nitems)
{
    const unsigned int alignment = volk_get_alignment();

    free_volk_buffers();

    m_pkt_data = (short *)volk_malloc(sizeof(short) * nitems, alignment);

    if (!m_pkt_data) {
        throw std::runtime_error(
            "rwt_source_s16: Failed to allocate volk buffers");
    }

    m_alloc_len = nitems;
}


int
rwt_source_s16_impl::work(
    int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    short *out0_i = (short *) output_items[0];
    short *out0_q = (short *) output_items[1];
    short *out1_i = (short *) output_items[2];
    short *out1_q = (short *) output_items[3];

    bool next_tag_valid;
    uint8_t next_tag_type;
    uint64_t next_tag_value;
    size_t max_pkt_size;
    size_t pkt_idx = 0;
    size_t pkt_read_count;
    int ret;
    gr::tag_t tag;

    (void)input_items;

    max_pkt_size = m_both_ch_en ? noutput_items * 4 : noutput_items * 2;

    if (m_alloc_len < max_pkt_size) {
        alloc_volk_buffers(max_pkt_size);
    }

    while (pkt_idx < max_pkt_size) {
        ret = m_source->read_until_next_tag_or_end(
            false,
            &m_pkt_data[pkt_idx],
            2,
            max_pkt_size - pkt_idx,
            &next_tag_valid,
            &next_tag_type,
            &next_tag_value,
            &pkt_read_count);

        if (ret == -ETIMEDOUT) {
            std::cerr << "Warning: rwt_source_s16: timed out.\n";
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
            if (!m_both_ch_en) {
                tag.offset = nitems_written(0) + pkt_idx / 2;
            } else {
                tag.offset = nitems_written(0) + pkt_idx / 4;
            }

            tag.value = pmt::from_uint64(next_tag_value);
            tag.srcid = pmt::PMT_F;
            m_tags.push_back(tag);
        }
    }

    if (!pkt_idx)
        return 0;

    /* Convert the types from short to complex numbers.

       @todo: Is it worth creating a custom volk kernel that optimizes with
              neon intrinsics. */
    if (!m_both_ch_en) {
        /* pkt_idx is # short interleaved samples, whereas noutput_items
           needs to be # short samples per channel. */
        noutput_items = pkt_idx / 2;

        /* complex s16 -> real,imag s16 */
        volk_16ic_deinterleave_16i_x2(
            out0_i, out0_q, (const lv_16sc_t*)m_pkt_data, noutput_items);

    } else {
        /* pkt_idx is total # short interleaved samples, whereas noutput_items
           needs to be # samples per channel. */
        noutput_items = pkt_idx / 4;

        /* The data is i0,q0,i1,q1, so we have to deinterleave it.
           There are no volk intrinsics to handle this (or atleast I couldn't
           find one). */
        short *data_ptr = m_pkt_data;
        for (int i = 0; i < noutput_items; i++) {
            *out1_i++ = *data_ptr++;
            *out1_q++ = *data_ptr++;
            *out0_i++ = *data_ptr++;
            *out0_q++ = *data_ptr++;
        }
    }

    while (!m_tags.empty()) {
        tag = m_tags.front();

        if (tag.offset < (nitems_written(0) + noutput_items)) {
            add_item_tag(0, tag);

            if (m_both_ch_en) {
                add_item_tag(1, tag);
            }

            m_tags.pop_front();
        } else {
            /* The list is sorted by offset, so when a tag offset is in the
               future, the loop is done.*/
            break;
        }
    }

    // Tell runtime system how many output items we produced.
    return noutput_items;
}


bool
rwt_source_s16_impl::start()
{
    bool ret;

    ret = rwt_base_block::start();
    if (!ret)
        return ret;

    ret = m_source->start(m_ch1_en, m_ch2_en);
    if (!ret)
        return ret;

    return ret;
}


bool
rwt_source_s16_impl::stop()
{
    m_source->stop();
    rwt_base_block::stop();
    free_volk_buffers();
    return true;
}


} /* namespace rwt */
} /* namespace gr */

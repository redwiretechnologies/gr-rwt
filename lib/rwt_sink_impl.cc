/* -*- c++ -*- */
/*
 * Copyright 2019 Red Wire Technologies.
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

#include <errno.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include "rwt_sink_impl.h"
#include "common_tag_types.h"
#include "common_registers.h"

#define BLK_ID 0x0002
#define BLK_VER_MIN 0x0001
#define BLK_VER_MAX 0x0002

namespace gr {
namespace rwt {

rwt_sink::sptr
rwt_sink::make(
    const pmt::pmt_t config,
    bool ch1_en,
    bool ch2_en,
    unsigned int reg_base_addr,
    const char *filter,
    bool use_tags,
    bool auto_filter,
    bool force_reload,
    unsigned int buffer_size)
{
  return gnuradio::get_initial_sptr
      (new rwt_sink_impl(
          config, ch1_en, ch2_en, reg_base_addr,
          filter, use_tags, auto_filter, force_reload, buffer_size));
}


/*
 * The private constructor
 */
rwt_sink_impl::rwt_sink_impl(
    const pmt::pmt_t config,
    bool ch1_en,
    bool ch2_en,
    unsigned int reg_base_addr,
    const char *filter,
    bool use_tags,
    bool auto_filter,
    bool force_reload,
    unsigned int buffer_size) :
    rwt_base_block(
        "rwt_sink",
        gr::io_signature::make(
            (int)ch1_en + (int)ch2_en,
            (int)ch1_en + (int)ch2_en,
            sizeof(gr_complex)),
        gr::io_signature::make(0, 0, 0)),
    rwt_base_block_impl(
        "rwt_sink",
        gr::io_signature::make(
            (int)ch1_en + (int)ch2_en,
            (int)ch1_en + (int)ch2_en,
            sizeof(gr_complex)),
        gr::io_signature::make(0, 0, 0),
        0,
        buffer_size,
        reg_base_addr,
        false,
        true,
        "default",
        force_reload),
    m_escape(0xaaaaaaaaaaaaaaaa),
    m_in_constructor(true),
    m_ch1_en(ch1_en),
    m_ch2_en(ch2_en),
    m_use_tags(use_tags),
    m_autofilter(auto_filter),
    m_samplerate(5000000),
    m_in0_16ic(NULL),
    m_in1_16ic(NULL),
    m_alloc_len(0),
    m_data(NULL),
    m_data_len(0),
    m_data_idx(0)
{
    if (!ch1_en && !ch2_en) {
        throw std::runtime_error(
            "Channel 1, channel 2, or both must be enabled.");
    }

    const char *handlers[] = {
        "samplerate",
        "escape",
        "use_ext_pps",
        "time_next_pps",
        "time_now",
        "sample_idx_next_pps",
        "sample_idx_now",
    };

    const int alignment_multiple =
        volk_get_alignment() / sizeof(short);
    set_alignment(std::max(1, alignment_multiple));
    set_output_multiple(4);

    m_both_ch_en = ch1_en && ch2_en;

    for (int i = 0; i < (sizeof(handlers)/sizeof(char *)); i++) {
        set_config_handler(
            handlers[i],
            boost::bind(&rwt_sink_impl::ctrl_reg_handler, this, _1, _2));
    }

    m_common->check_user_blkid(
        BLK_ID,
        BLK_VER_MIN,
        BLK_VER_MAX,
        true);

    m_time_helper.setup(m_common, m_use_tags);

    m_sink->setup_tags(m_use_tags, m_escape);
    config_msg_handler(config);

    if (!m_common->setup_filter(filter, m_samplerate, m_autofilter))
        throw std::runtime_error("Unable to set filter");
    m_in_constructor = false;
}


/*
 * Our virtual destructor.
 */
rwt_sink_impl::~rwt_sink_impl()
{
    free_volk_buffers();
}


void
rwt_sink_impl::ctrl_reg_handler(
    const std::string key,
    const std::string value)
{

    if (key == "samplerate") {

        m_samplerate = (unsigned long)strtoul(value.c_str(), NULL, 0);
        m_common->set_attr("samplerate", value);

        if (m_autofilter && !m_in_constructor) {
            if(!m_common->setup_filter(NULL, m_samplerate, m_autofilter))
                std::cerr << "Error setting filter for AD9361.\n";
        }

    }else if (key == "escape") {
        m_escape = (uint64_t)strtoull(value.c_str(), NULL, 0);
        m_sink->setup_tags(m_use_tags, m_escape);

    } else {
        m_time_helper.ctrl_reg_handler(key, value, m_samplerate);
    }
}


pmt::pmt_t
rwt_sink_impl::get_time()
{
    return m_time_helper.get_time_pmt(
        time_helper::INDEX_DAC,
        m_samplerate);
}


uint64_t
rwt_sink_impl::get_sample_idx()
{
    return m_time_helper.get_sample_idx(time_helper::INDEX_DAC);
}


void
rwt_sink_impl::alloc_volk_buffers(int nitems)
{
    const unsigned int alignment = volk_get_alignment();

    free_volk_buffers();

    m_in0_16ic = (short *)volk_malloc(sizeof(short) * nitems * 2, alignment);

    if (!m_in0_16ic) {
        throw std::runtime_error(
            "rwt_sink: Failed to allocate volk buffers");
    }

    if (m_both_ch_en) {
        m_in1_16ic = (short *)volk_malloc(sizeof(short) * nitems * 2, alignment);

        if (!m_in1_16ic) {
            throw std::runtime_error(
                "rwt_sink: Failed to allocate volk buffers");
        }
    }

    m_alloc_len = nitems;
}

bool
rwt_sink_impl::push_data(uint64_t value)
{
    int ret;

    m_data[m_data_idx] = htole64(value);
    m_data_idx++;

    if (m_data_idx >= m_data_len) {
        do {
            ret = m_sink->send_buffer();
        } while (ret == -ETIMEDOUT);

        if (ret < 0) {
            m_data_idx = 0;
            return false;
        }

        m_data_idx = 0;
        m_data = m_sink->get_buffer_data(&m_data_len);
    }

    return true;
}


int
rwt_sink_impl::work(
    int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    const gr_complex *in0 = (const gr_complex *) input_items[0];
    const gr_complex *in1 = (const gr_complex *) input_items[1];
    uint64_t value;
    int i;
    bool ret;
    std::vector<tag_t> tags;
    int tag_idx;
    int next_tag_offset;
    uint64_t tag_value;
    uint8_t tag_type;

    (void)output_items;

    if (m_alloc_len < noutput_items) {
        alloc_volk_buffers(noutput_items);
    }

    /* complex<float> -> complex<short> */
    volk_32f_s32f_convert_16i(
        m_in0_16ic, (const float *)in0, 32768., noutput_items * 2);

    if (m_both_ch_en) {
        volk_32f_s32f_convert_16i(
            m_in1_16ic, (const float *)in1, 32768., noutput_items * 2);
    }

    /* Get tags within the range. */
    if (m_use_tags) {
        get_tags_in_range(
            tags,
            0,
            nitems_read(0),
            nitems_read(0) + noutput_items);
    }

    tag_idx = 0;
    if (tags.size()) {
        next_tag_offset = (int)(tags[tag_idx].offset - nitems_read(0));
    } else {
        next_tag_offset = noutput_items;
    }

    i = 0;
    uint16_t *ptr0_16i = (uint16_t *)m_in0_16ic;
    uint16_t *ptr1_16i = (uint16_t *)m_in1_16ic;
    while (i < noutput_items) {
        if (m_both_ch_en) {
            value = ((uint64_t)(*ptr0_16i++)) << 16;
            value |= ((uint64_t)(*ptr0_16i++)) << 0;
            value |= ((uint64_t)(*ptr1_16i++)) << 48;
            value |= ((uint64_t)(*ptr1_16i++)) << 32;
        } else {
            value = ((uint64_t)(*ptr0_16i++)) << 48;
            value |= ((uint64_t)(*ptr0_16i++)) << 32;
            value |= ((uint64_t)(*ptr0_16i++)) << 16;
            value |= ((uint64_t)(*ptr0_16i++)) << 0;
        }

        while (i >= next_tag_offset) {
            if (tags[tag_idx].key == RWT_TAG_PMT_TX_TIME) {
                uint64_t sample_idx;
                uint64_t secs;
                uint64_t nsecs;

                tag_type = RWT_TAG_IDX_HOLD;

                secs = pmt::to_uint64(pmt::car(tags[tag_idx].value));
                nsecs = pmt::to_uint64(pmt::cdr(tags[tag_idx].value));

                sample_idx = secs * m_samplerate;
                sample_idx += (nsecs * m_samplerate) / 1000000000;
                tag_value = sample_idx;

            } else if (tags[tag_idx].key == RWT_TAG_PMT_TX_SAMPLE) {
                tag_type = RWT_TAG_IDX_HOLD;
                tag_value = pmt::to_uint64(tags[tag_idx].value);
            } else if (tags[tag_idx].key == RWT_TAG_PMT_TX_SOB) {
                tag_type = RWT_TAG_IDX_SOB;
                tag_value = 0;
            } else if (tags[tag_idx].key == RWT_TAG_PMT_TX_EOB) {
                tag_type = RWT_TAG_IDX_EOB;
                tag_value = 0;
            } else {
                tag_type = 0;
            }

            tag_idx++;
            if (tag_idx < tags.size()) {
                next_tag_offset = (int)(tags[tag_idx].offset - nitems_read(0));
            } else {
                next_tag_offset = noutput_items;
            }

            if (tag_type == 0) {
                continue;
            }

            tag_value &= (((uint64_t)1) << 56) - 1;
            tag_value |= ((uint64_t)tag_type) << 56;

            if (!push_data(m_escape))
                return -1;

            if (!push_data(tag_value))
                return -1;
        }

        if (!push_data(value))
            return -1;

        if (m_use_tags && (value == m_escape)) {
            if (!push_data(0))
                return -1;
        }

        if (m_both_ch_en) {
            i++;
        } else {
            i += 2;
        }

    }

    consume_each(noutput_items);
    return 0;
}


bool
rwt_sink_impl::start()
{
    bool ret;

    ret = rwt_base_block::start();
    if (!ret)
        return ret;

    ret = m_sink->start(m_ch1_en, m_ch2_en);
    if (!ret)
        return ret;

    m_data = m_sink->get_buffer_data(&m_data_len);
    m_data_idx = 0;

    return ret;
}

bool
rwt_sink_impl::stop()
{
    m_sink->stop();
    rwt_base_block::stop();
    free_volk_buffers();
    return true;
}

} /* namespace rwt */
} /* namespace gr */

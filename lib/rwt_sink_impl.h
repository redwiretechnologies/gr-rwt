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

#ifndef INCLUDED_RWT_FMCOMMS_SINK_IMPL_H
#define INCLUDED_RWT_FMCOMMS_SINK_IMPL_H

#include <rwt/rwt_sink.h>
#include "rwt_base_block_impl.h"
#include "time_helper.h"

namespace gr {
namespace rwt {

class rwt_sink_impl : public rwt_sink, public rwt_base_block_impl
{

 public:
    rwt_sink_impl(
        const pmt::pmt_t config,
        bool ch1_en,
        bool ch2_en,
        unsigned int reg_base_addr,
        const char *filter,
        bool use_tags,
        bool auto_filter,
        bool force_reload,
        unsigned int buffer_size);

    ~rwt_sink_impl();

    void ctrl_reg_handler(
        const std::string key,
        const std::string value);

    int work(
        int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

    pmt::pmt_t get_time();
    uint64_t get_sample_idx();

    bool start();
    bool stop();

private:
    void alloc_volk_buffers(int nitems);

    inline void free_volk_buffers() {
        if (m_in0_16ic) volk_free(m_in0_16ic);
        if (m_in1_16ic) volk_free(m_in1_16ic);
        m_in0_16ic = NULL;
        m_in1_16ic = NULL;
        m_alloc_len = 0;
    }

    bool push_data(uint64_t value);

    uint64_t m_escape;
    bool m_in_constructor;
    bool m_ch1_en;
    bool m_ch2_en;
    bool m_both_ch_en;
    bool m_use_tags;
    bool m_autofilter;
    unsigned long m_samplerate;
    short *m_in0_16ic;
    short *m_in1_16ic;
    size_t m_alloc_len;
    uint64_t *m_data;
    size_t m_data_len;
    size_t m_data_idx;
    time_helper m_time_helper;
};

} // namespace rwt
} // namespace gr

#endif /* INCLUDED_RWT_FMCOMMS_SINK_IMPL_H */

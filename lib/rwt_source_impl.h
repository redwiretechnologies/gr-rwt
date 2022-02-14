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

#ifndef INCLUDED_RWT_FMCOMMS_SOURCE_IMPL_H
#define INCLUDED_RWT_FMCOMMS_SOURCE_IMPL_H

#include <list>

#include <rwt/rwt_source.h>
#include "rwt_base_block_impl.h"
#include "rwt_registers_common.h"
#include "cic_filter_fpga.h"
#include "time_helper.h"


namespace gr {
namespace rwt {

class rwt_source_impl : public rwt_source, public rwt_base_block_impl
{
public:
    rwt_source_impl(
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
        const char *tx_name);

    ~rwt_source_impl();

    int work(
        int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

    void ctrl_reg_handler(
        const std::string key,
        const std::string value);

    void set_samplerate(const std::string value);
    bool start();
    bool stop();

    pmt::pmt_t get_time();
    uint64_t get_sample_idx();

private:
    void alloc_volk_buffers(int nitems);

    void free_volk_buffers() {
        if (m_pkt_data) volk_free(m_pkt_data);
        if (m_data_interleaved) volk_free(m_data_interleaved);
        m_pkt_data = NULL;
        m_data_interleaved = NULL;
        m_alloc_len = 0;
    }

    uint64_t m_escape;
    bool m_in_constructor;
    bool m_ch1_en;
    bool m_ch2_en;
    bool m_both_ch_en;
    bool m_use_tags;
    bool m_autofilter;
    unsigned long m_samplerate;

    size_t m_alloc_len;
    short *m_pkt_data;
    float *m_data_interleaved;

    time_helper m_time_helper;

    rwt_registers_common *common_registers_rwt;
    cic_filter_fpga *cic_frontend;

    std::list< gr::tag_t > m_tags;

};

} // namespace rwt
} // namespace gr

#endif /* INCLUDED_RWT_FMCOMMS_SOURCE_IMPL_H */

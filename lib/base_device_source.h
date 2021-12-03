/* -*- c++ -*- */
/* This file is derived from the gr-iio base_device_source.cc. This
 * following is the original copyright.
 *
 *     Copyright 2014 Analog Devices Inc.
 *     Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 *     This is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; either version 3, or (at your option)
 *     any later version.
 *
 *     This software is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this software; see the file COPYING.  If not, write to
 *     the Free Software Foundation, Inc., 51 Franklin Street,
 *     Boston, MA 02110-1301, USA.
 */

#ifndef RWT_DEVICE_SOURCE_H
#define RWT_DEVICE_SOURCE_H

#include <boost/thread.hpp>

#include <stdint.h>
#include <map>
#include <string>
#include <memory>

#include <iio.h>
#include <pmt/pmt.h>
#include "base_device_common.h"


namespace gr {
namespace rwt {

class base_device_source
{
public:
    base_device_source(
        std::shared_ptr<base_device_common> common,
        unsigned int buffer_size);

    ~base_device_source();

    int buffer_next(uint64_t **data);
    void buffer_done();
    int read_until_next_tag_or_end(
        bool skip_pkt,
        void *pkt_data,
        size_t pkt_data_size,
        size_t pkt_max_size,
        bool *next_tag_valid,
        uint8_t *next_tag_type,
        uint64_t *next_tag_value,
        size_t *pkt_read_count);

    bool start(bool ch1_en, bool ch2_en);
    void setup_tags(bool use_escape, uint64_t escape);
    bool stop();

private:
    void thread_refill();

    unsigned int m_timeout;
    std::shared_ptr<base_device_common> m_common;
    struct iio_buffer *m_buf;
    struct iio_channel *m_chan0;
    struct iio_channel *m_chan1;
    struct iio_channel *m_chan2;
    struct iio_channel *m_chan3;
    unsigned int m_buffer_size;
    unsigned int m_buf_len;
    bool m_do_refill;
    bool m_thread_stopped;
    uint64_t m_escape;
    uint64_t m_use_escape;
    int m_escape_state;

    size_t m_raw_len;
    uint64_t *m_raw_data;
    size_t m_raw_idx;
    bool m_single_channel;

    boost::condition_variable m_cv;
    boost::condition_variable m_cv2;
    boost::mutex m_lock;
    boost::thread m_thread_refill;
};

} /* namespace iio */
} /* namespace gr */

#endif /* RWT_DEVICE_SOURCE_H */

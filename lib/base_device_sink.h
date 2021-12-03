/* -*- c++ -*- */
/* This file is derived from the gr-iio base_device_sink.cc. This
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

#ifndef RWT_DEVICE_SINK_H
#define RWT_DEVICE_SINK_H

#include <stdint.h>
#include <map>
#include <string>
#include <memory>

#include <iio.h>
#include <pmt/pmt.h>
#include "base_device_common.h"


namespace gr {
namespace rwt {

class base_device_sink
{
public:
    base_device_sink(
        std::shared_ptr<base_device_common> common,
        unsigned int buffer_size);

    ~base_device_sink();

    bool start(bool ch1_en, bool ch2_en);
    bool stop();

    int send_buffer();
    uint64_t *get_buffer_data(size_t *len);
    void setup_tags(bool use_tags, uint64_t escape);

private:

    std::shared_ptr<base_device_common> m_common;
    struct iio_buffer *m_buf;
    struct iio_channel *m_chan0;
    struct iio_channel *m_chan1;
    struct iio_channel *m_chan2;
    struct iio_channel *m_chan3;
    unsigned int m_buffer_size;
    unsigned int m_buf_len;
};

} /* namespace iio */
} /* namespace gr */

#endif /* RWT_DEVICE_SINK_H */

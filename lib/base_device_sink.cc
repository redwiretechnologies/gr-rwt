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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <errno.h>
#include <iostream>
#include <gnuradio/io_signature.h>
#include <gnuradio/thread/thread.h>
#include "base_device_sink.h"
#include "common_registers.h"

#include <cstdio>
#include <fstream>
#include <string>

#include <sys/mman.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <ad9361.h>

namespace gr {
namespace rwt {

base_device_sink::base_device_sink(
    std::shared_ptr<base_device_common> common,
    unsigned int buffer_size) :
    m_common(common),
    m_buf(NULL),
    m_chan0(NULL),
    m_chan1(NULL),
    m_chan2(NULL),
    m_chan3(NULL),
    m_buf_len(0)
{
    /* Buffer size is actually number 16-bit of samples I and Q. We need
       to ensure that we have a 64-bit multiple of for the length. */
    if (buffer_size % 2) {
        buffer_size++;
    }
    m_buffer_size = buffer_size;

    common->add_attr_alias("tx_freq", "out_altvoltage1_TX_LO_frequency");
    common->add_attr_alias("freq", "out_altvoltage1_TX_LO_frequency");
    common->add_attr_alias("samplerate", "out_voltage_sampling_frequency");
    common->add_attr_alias("bw", "out_voltage_rf_bandwidth");
    common->add_attr_alias("tx_bw", "out_voltage_rf_bandwidth");
    common->add_attr_alias("rfport", "out_voltage0_rf_port_select");
    common->add_attr_alias("rfport1", "out_voltage0_rf_port_select");
    common->add_attr_alias("rfport2", "out_voltage1_rf_port_select");
    common->add_attr_alias("tx_rfport1", "out_voltage0_rf_port_select");
    common->add_attr_alias("tx_rfport2", "out_voltage1_rf_port_select");
    common->add_attr_alias("tx_gain1", "out_voltage0_hardwaregain");
    common->add_attr_alias("tx_gain2", "out_voltage1_hardwaregain");
    common->add_attr_alias("tx_gain", "out_voltage0_hardwaregain");
    common->add_attr_alias("tx_gain", "out_voltage1_hardwaregain");

    m_chan0 = iio_device_find_channel(common->m_txdev, "voltage0", true);
    m_chan1 = iio_device_find_channel(common->m_txdev, "voltage1", true);
    m_chan2 = iio_device_find_channel(common->m_txdev, "voltage2", true);
    m_chan3 = iio_device_find_channel(common->m_txdev, "voltage3", true);

    if (!m_chan0 || !m_chan1 || !m_chan2 || !m_chan3)
        throw std::runtime_error("Unable to find channels!\n");

    iio_channel_disable(m_chan0);
    iio_channel_disable(m_chan1);
    iio_channel_disable(m_chan2);
    iio_channel_disable(m_chan3);
}


base_device_sink::~base_device_sink()
{
    stop();

    if (m_chan0) iio_channel_disable(m_chan0);
    if (m_chan1) iio_channel_disable(m_chan1);
    if (m_chan2) iio_channel_disable(m_chan2);
    if (m_chan3) iio_channel_disable(m_chan3);
    m_chan0 = NULL;
    m_chan1 = NULL;
    m_chan2 = NULL;
    m_chan3 = NULL;
}


int
base_device_sink::send_buffer()
{
    int ret;

    ret = iio_buffer_push(m_buf);
    if ((ret < 0) && (ret != -ETIMEDOUT)) {
        char buf[256];
        iio_strerror(-ret, buf, sizeof(buf));
        std::string error(buf);
        std::cerr << "Unable to push buffer: " << error << std::endl;
    }

    return ret;
}


uint64_t *
base_device_sink::get_buffer_data(size_t *len)
{
    uintptr_t start = (uintptr_t)iio_buffer_start(m_buf);
    uintptr_t end = (uintptr_t)iio_buffer_end(m_buf);

    *len = (end - start) / 8;
    return (uint64_t *)start;
}


bool
base_device_sink::start(bool ch1_en, bool ch2_en)
{
    if (!ch1_en && !ch2_en) {
        throw std::runtime_error("At least one channel must be enabled.\n");
    }

    if (ch1_en)
    {
        iio_channel_enable(m_chan0);
        iio_channel_enable(m_chan1);
    }
    if (ch2_en)
    {
        iio_channel_enable(m_chan2);
        iio_channel_enable(m_chan3);
    }

    m_buf = iio_device_create_buffer(m_common->m_txdev, m_buffer_size, false);
    if (!m_buf)
        throw std::runtime_error("Unable to create buffer!\n");

    return true;
}


bool
base_device_sink::stop()
{
    if (m_buf)
        iio_buffer_destroy(m_buf);
    m_buf = NULL;
    return true;
}

void
base_device_sink::setup_tags(bool use_tags, uint64_t escape)
{
    m_common->write_reg(
        REG_BLK_DAC,
        REG_DAC_ESCAPE_MSB,
        (uint32_t)(escape >> 32));

    m_common->write_reg(
        REG_BLK_DAC,
        REG_DAC_ESCAPE_LSB,
        (uint32_t)(escape & 0xffffffff));

    m_common->write_reg(REG_BLK_DAC, REG_DAC_CTL, (uint32_t)use_tags);
}


} /* namespace iio */
} /* namespace gr */

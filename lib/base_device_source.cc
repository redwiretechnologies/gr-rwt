
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <errno.h>
#include <iostream>
#include <gnuradio/io_signature.h>
#include <gnuradio/thread/thread.h>
#include "base_device_source.h"
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

static const int STATE_NORMAL = 0;
static const int STATE_ESCAPE = 1;


base_device_source::base_device_source(
    std::shared_ptr<base_device_common> common,
    unsigned int buffer_size) :
    m_timeout(2000),
    m_common(common),
    m_buf(NULL),
    m_chan0(NULL),
    m_chan1(NULL),
    m_chan2(NULL),
    m_chan3(NULL),
    m_buf_len(0),
    m_do_refill(false),
    m_thread_stopped(true),
    m_escape(0xaaaaaaaaaaaaaaaa),
    m_use_escape(false),
    m_escape_state(STATE_NORMAL),
    m_raw_len(0),
    m_raw_data(NULL),
    m_raw_idx(0),
    m_single_channel(false)
{
    /* Buffer size is actually number 16-bit of samples I and Q. We need
       to ensure that we have a 64-bit multiple of for the length. */
    if (buffer_size % 2) {
        buffer_size++;
    }
    m_buffer_size = buffer_size;

    common->add_attr_alias("rx_freq", "out_altvoltage0_RX_LO_frequency");
    common->add_attr_alias("freq", "out_altvoltage0_RX_LO_frequency");
    common->add_attr_alias("samplerate", "in_voltage_sampling_frequency");
    common->add_attr_alias("bw", "in_voltage_rf_bandwidth");
    common->add_attr_alias("rx_bw", "in_voltage_rf_bandwidth");
    common->add_attr_alias("quadrature", "in_voltage_quadrature_tracking_en");
    common->add_attr_alias("rfdc", "in_voltage_rf_dc_offset_tracking_en");
    common->add_attr_alias("bbdc", "in_voltage_bb_dc_offset_tracking_en");
    common->add_attr_alias("gain1_mode", "in_voltage0_gain_control_mode");
    common->add_attr_alias("gain1", "in_voltage0_hardwaregain");
    common->add_attr_alias("gain2_mode", "in_voltage1_gain_control_mode");
    common->add_attr_alias("gain2", "in_voltage1_hardwaregain");
    common->add_attr_alias("rx_gain1_mode", "in_voltage0_gain_control_mode");
    common->add_attr_alias("rx_gain1", "in_voltage0_hardwaregain");
    common->add_attr_alias("rx_gain2_mode", "in_voltage1_gain_control_mode");
    common->add_attr_alias("rx_gain2", "in_voltage1_hardwaregain");
    common->add_attr_alias("rfport", "in_voltage0_rf_port_select");
    common->add_attr_alias("rfport1", "in_voltage0_rf_port_select");
    common->add_attr_alias("rfport2", "in_voltage1_rf_port_select");
    common->add_attr_alias("rx_rfport1", "in_voltage0_rf_port_select");
    common->add_attr_alias("rx_rfport2", "in_voltage1_rf_port_select");

    m_chan0 = iio_device_find_channel(common->m_rxdev, "voltage0", false);
    m_chan1 = iio_device_find_channel(common->m_rxdev, "voltage1", false);
    if (!m_chan0 || !m_chan1)
        throw std::runtime_error("Unable to find channels!\n");

    m_chan2 = iio_device_find_channel(common->m_rxdev, "voltage2", false);
    m_chan3 = iio_device_find_channel(common->m_rxdev, "voltage3", false);
    if (!m_chan2 || !m_chan3)
        m_single_channel = true;

    if (m_chan0) iio_channel_disable(m_chan0);
    if (m_chan1) iio_channel_disable(m_chan1);
    if (m_chan2) iio_channel_disable(m_chan2);
    if (m_chan3) iio_channel_disable(m_chan3);
}


base_device_source::~base_device_source()
{
    stop();


    m_chan0 = NULL;
    m_chan1 = NULL;
    m_chan2 = NULL;
    m_chan3 = NULL;
}


void
base_device_source::thread_refill()
{
    boost::unique_lock<boost::mutex> lock(m_lock);
    ssize_t ret;

    for (;;) {
        while (!m_do_refill)
            m_cv.wait(lock);

        m_buf_len = 0;

        lock.unlock();
        ret = iio_buffer_refill(m_buf);
        lock.lock();

        m_do_refill = false;

        if (ret == -ETIMEDOUT)
            continue;

        if (ret < 0)
            break;

        m_buf_len = ret;
        m_cv2.notify_all();
    }

    /* -EBADF happens when the buffer is cancelled */
    if (ret != -EBADF) {

        char buf[256];
        iio_strerror(-ret, buf, sizeof(buf));
        std::string error(buf);

        std::cerr << "Unable to refill buffer: " << error << std::endl;
    }

    m_thread_stopped = true;
    m_cv2.notify_all();
}


bool
base_device_source::start(bool ch1_en, bool ch2_en)
{
    boost::unique_lock<boost::mutex> lock(m_lock);

    m_buf_len = 0;
    m_do_refill = false;
    m_thread_stopped = false;
    m_raw_len = 0;
    m_raw_data = NULL;
    m_raw_idx = 0;

    if (!ch1_en && !ch2_en) {
        throw std::runtime_error("At least one channel must be enabled.\n");
    }

    if (m_single_channel && ch2_en) {
        throw std::runtime_error(
            "Trying to enable channel 2, but only channel 1 is available!\n");
    }

    if (ch1_en) {
        iio_channel_enable(m_chan0);
        iio_channel_enable(m_chan1);
        if (!m_chan0 || !m_chan1)
            throw std::runtime_error("Unable to find channels!\n");
    }

    if (ch2_en) {
        iio_channel_enable(m_chan2);
        iio_channel_enable(m_chan3);

        if (!m_chan2 || !m_chan3)
            throw std::runtime_error("Unable to find channels!\n");
    }

    m_buf = iio_device_create_buffer(m_common->m_rxdev, m_buffer_size, false);
    if (!m_buf)
        throw std::runtime_error("Unable to create buffer!\n");


    m_thread_refill = boost::thread(&base_device_source::thread_refill, this);

    return true;
}


bool
base_device_source::stop()
{
    if (m_buf) iio_buffer_cancel(m_buf);

    boost::unique_lock<boost::mutex> lock(m_lock);
    m_do_refill = true;
    m_cv.notify_all();
    lock.unlock();

    m_thread_refill.join();

    m_raw_len = 0;
    m_raw_data = NULL;
    m_raw_idx = 0;

    if (m_buf)
        iio_buffer_destroy(m_buf);
    m_buf = NULL;

    if (m_chan0) iio_channel_disable(m_chan0);
    if (m_chan1) iio_channel_disable(m_chan1);
    if (m_chan2) iio_channel_disable(m_chan2);
    if (m_chan3) iio_channel_disable(m_chan3);
    return true;
}


void
base_device_source::setup_tags(bool use_escape, uint64_t escape)
{
    m_escape = escape;
    m_use_escape = use_escape;
    m_escape_state = STATE_NORMAL;

    m_common->write_reg(
        REG_BLK_ADC,
        REG_ADC_ESCAPE_MSB,
        (uint32_t)(escape >> 32));

    m_common->write_reg(
        REG_BLK_ADC,
        REG_ADC_ESCAPE_LSB,
        (uint32_t)(escape & 0xffffffff));

    m_common->write_reg(REG_BLK_ADC, REG_ADC_CTL, (uint32_t)use_escape);
}


int
base_device_source::buffer_next(uint64_t **data)
{
    boost::unique_lock<boost::mutex> lock(m_lock);

    *data = NULL;

    if (m_thread_stopped || !m_buf)
        return -EIO; /* EOF */

    /* No items in buffer -> ask for a refill */
    if (!m_do_refill) {
        m_do_refill = true;
        m_cv.notify_all();
    }

    while (m_do_refill) {
        bool fast_enough = m_cv2.timed_wait(
            lock,
            boost::posix_time::milliseconds(m_timeout));

        if (m_thread_stopped || !m_buf)
            return -EIO; /* EOF */

        if (!fast_enough) {
            return -ETIMEDOUT;
        }
    }

    *data = (uint64_t *)iio_buffer_start(m_buf);
    return m_buf_len / 8;
}


void
base_device_source::buffer_done()
{
    boost::unique_lock<boost::mutex> lock(m_lock);
    m_do_refill = true;
    m_cv.notify_all();
}


int
base_device_source::read_until_next_tag_or_end(
    bool skip_pkt,
    void *pkt_data,
    size_t pkt_data_size,
    size_t pkt_max_size,
    bool *next_tag_valid,
    uint8_t *next_tag_type,
    uint64_t *next_tag_value,
    size_t *pkt_read_count)
{
    int ret;
    uint64_t value;
    uint8_t next_tag_meta;
    uint8_t *pkt_data_u8 = (uint8_t *)pkt_data;
    uint16_t *pkt_data_u16 = (uint16_t *)pkt_data;
    uint32_t *pkt_data_u32 = (uint32_t *)pkt_data;
    uint64_t *pkt_data_u64 = (uint64_t *)pkt_data;
    size_t pkt_size = 0;

    *next_tag_valid = false;
    *next_tag_type = 0;
    *next_tag_value = 0;
    *pkt_read_count = 0;

    if ((!m_raw_data) || (m_raw_len == 0) || (m_raw_idx >= m_raw_len)) {
        m_raw_idx = 0;
        m_raw_len = 0;
        ret = buffer_next(&m_raw_data);
        if (ret < 0) {
            return ret;
        }
        m_raw_len = ret;
    }

    while ((m_raw_idx < m_raw_len) && (skip_pkt || (pkt_max_size > pkt_size))) {
        value = le64toh(m_raw_data[m_raw_idx]);
        m_raw_idx++;

        if (m_escape_state == STATE_ESCAPE) {
            if (value == 0) {
                value = m_escape;
                m_escape_state = STATE_NORMAL;

            } else {
                next_tag_meta = value >> 56;
                *next_tag_valid = true;
                *next_tag_type = next_tag_meta & 0x7f;
                *next_tag_value = value & ((((uint64_t)1) << 56) - 1);
                if (!(next_tag_meta & 0x80))
                    m_escape_state = STATE_NORMAL;
                break;
            }
        } else if (m_use_escape && (value == m_escape)) {
            m_escape_state = STATE_ESCAPE;
            continue;
        }

        /* If pkt_data is NULL, this drops the packet. */
        if (skip_pkt)
            continue;

        switch(pkt_data_size) {
        case 1:
            *pkt_data_u8++ = (value >> 56) & 0xff;
            *pkt_data_u8++ = (value >> 48) & 0xff;
            *pkt_data_u8++ = (value >> 40) & 0xff;
            *pkt_data_u8++ = (value >> 32) & 0xff;
            *pkt_data_u8++ = (value >> 24) & 0xff;
            *pkt_data_u8++ = (value >> 16) & 0xff;
            *pkt_data_u8++ = (value >> 8) & 0xff;
            *pkt_data_u8++ = (value >> 0) & 0xff;
            pkt_size += 8;
            break;
        case 2:
            *pkt_data_u16++ = (value >> 48) & 0xffff;
            *pkt_data_u16++ = (value >> 32) & 0xffff;
            *pkt_data_u16++ = (value >> 16) & 0xffff;
            *pkt_data_u16++ = (value >> 0) & 0xffff;
            pkt_size += 4;
            break;
        case 4:
            *pkt_data_u32++ = (value >> 32) & 0xffff;
            *pkt_data_u32++ = (value >> 0) & 0xffff;
            pkt_size += 2;
            break;
        case 8:
            *pkt_data_u64++ = value;
            pkt_size++;
            break;
        default:
            throw std::runtime_error(
                "handle_source_data(): data_size must be 1,2,4, or 8.");
        }
    }

    *pkt_read_count = pkt_size;
    return 0;
}

} /* namespace iio */
} /* namespace gr */

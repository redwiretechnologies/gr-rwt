/* -*- c++ -*- */
/* This file is derived from the gr-iio base_device_common.cc. This
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

#ifndef RWT_DEVICE_COMMON_H
#define RWT_DEVICE_COMMON_H

#include <stdint.h>
#include <map>
#include <string>
#include <memory>

#include <pmt/pmt.h>
#include <iio.h>

namespace gr {
namespace rwt {

class base_device_common
{
public:
    base_device_common(
        unsigned int reg_base_addr,
        const char *personality,
        bool force_reload,
        const char *phy_name,
        const char *rx_name,
        const char *tx_name);
    ~base_device_common();

    void write_reg_raw(uint32_t offset, uint32_t value);
    uint32_t read_reg_raw(uint32_t offset);
    void write_reg(uint32_t blk_id, uint32_t addr, uint32_t value);
    uint32_t read_reg(uint32_t blk_id, uint32_t addr);
    void write_reg_u64(
        uint32_t blk_id,
        uint32_t addr_msb,
        uint32_t addr_lsb,
        uint64_t value);

    uint64_t read_reg_u64(
        uint32_t blk_id,
        uint32_t addr_msb,
        uint32_t addr_lsb);

    void write_reg_ctrl_mask(
        uint32_t blk_id,
        uint32_t addr,
        uint32_t mask,
        bool set);

    bool set_attrs(const pmt::pmt_t &config);
    bool set_attrs(const std::map<std::string, std::string> &config);
    bool set_attr(
        const std::string &key,
        const std::string &val);

    bool load_filter_file(std::string &filter);
    bool setup_filter(
        const char *filter,
        unsigned long samplerate,
        bool auto_filter);

    void add_attr_alias(
        const std::string alias,
        const std::string fullname);

    int load_personality(const std::string &personality, bool force);

    bool check_user_blkid(
        uint16_t blk_id,
        uint16_t version_min,
        uint16_t version_max,
        bool assert_on_fail);

protected:

    bool set_attr_internal(
        struct iio_channel *chn,
        const char *attr,
        const char *val);

    struct iio_context *m_ctx;
    struct iio_device *m_phy;
    struct iio_device *m_rxdev;
    struct iio_device *m_txdev;
    std::multimap<const std::string, const std::string> m_attr_alias;
    int m_regs_fd;
    void *m_regs_base;
    uint8_t *m_regs;

    friend class base_device_source;
    friend class base_device_sink;
};


} /* namespace iio */
} /* namespace gr */

#endif /* RWT_DEVICE_COMMON_H */

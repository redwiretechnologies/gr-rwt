/*
 *
 *  Copyright 2020 Red Wire Technology
 *  Copyright 2018 Red Wire Technologies
 * This file is derived from the gr-iio base_device_common.cc. This
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

#include <stdlib.h>
#include <iostream>
#include <boost/format.hpp>
#include <gnuradio/io_signature.h>
#include <gnuradio/thread/thread.h>
#include "base_device_common.h"
#include "common_registers.h"

#include <cstdio>
#include <fstream>
#include <string>
#include <map>

#include <sys/mman.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <ad9361.h>

#define MAP_SIZE 8192UL
#define MAP_MASK (MAP_SIZE - 1)

namespace gr {
namespace rwt {

static bool g_fpga_loaded = false;
static std::string g_fpga_loaded_personality;

base_device_common::base_device_common(
    unsigned int reg_base_addr, const char *personality, bool force_reload) :
    m_ctx(NULL),
    m_phy(NULL),
    m_rxdev(NULL)
{

    if (load_personality(personality, force_reload) != 0) {
        throw std::runtime_error("Failed to load rwt image!\n");
    }

    m_ctx = iio_create_default_context();
    if (!m_ctx)
        throw std::runtime_error("Unable to create context");

    if (iio_context_get_devices_count(m_ctx) <= 0)
        throw std::runtime_error("No Device");

    m_phy =  iio_context_find_device(m_ctx, "ad9361-phy");
    if (!m_phy)
        throw std::runtime_error("No phy found");

    m_rxdev = iio_context_find_device(m_ctx, "cf-ad9361-lpc");
    if (!m_rxdev)
        throw std::runtime_error("Missing RX iio device.");

    m_txdev = iio_context_find_device(m_ctx, "cf-ad9361-dds-core-lpc");
    if (!m_txdev)
        throw std::runtime_error("Missing TX iio device.");

    m_regs_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (m_regs_fd < 0)
        throw std::runtime_error("Could not open /dev/mem");

    m_regs_base = mmap(
        0,
        MAP_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        m_regs_fd,
        reg_base_addr & ~MAP_MASK);
    if (m_regs_base == MAP_FAILED)
        throw std::runtime_error("mmap() failed.");

    m_regs = ((uint8_t *)m_regs_base) + (reg_base_addr & MAP_MASK);
}

base_device_common::~base_device_common()
{
    g_fpga_loaded = false;
    munmap(m_regs_base, MAP_SIZE);
    close(m_regs_fd);
}

void
base_device_common::add_attr_alias(
    const std::string alias,
    const std::string fullname)
{
    m_attr_alias.insert({alias, fullname});
}

void
base_device_common::write_reg_raw(uint32_t offset, uint32_t value)
{
    uint32_t *addr = (uint32_t *)(m_regs + offset);
    *addr = value;
}

uint32_t
base_device_common::read_reg_raw(uint32_t offset)
{
    uint32_t *addr = (uint32_t *)(m_regs + offset);
    return *addr;
}

void
base_device_common::write_reg(uint32_t blk_id, uint32_t addr, uint32_t value)
{
    uint32_t offset = ((blk_id & 0x3) << 11) | ((addr & 0xff) << 2);
    write_reg_raw(offset, value);
}

uint32_t
base_device_common::read_reg(uint32_t blk_id, uint32_t addr)
{
    uint32_t offset = ((blk_id & 0x3) << 11) | ((addr & 0xff) << 2);
    return read_reg_raw(offset);
}

void
base_device_common::write_reg_u64(
    uint32_t blk_id,
    uint32_t addr_msb,
    uint32_t addr_lsb,
    uint64_t value)
{
    /* Order matters for some registers. The convention is that if order
       matters, MSB must be set before the LSB. */
    write_reg(
        blk_id,
        addr_msb,
        (uint32_t)(value >> 32));
    write_reg(
        blk_id,
        addr_lsb,
        (uint32_t)value);
}

uint64_t
base_device_common::read_reg_u64(
    uint32_t blk_id,
    uint32_t addr_msb,
    uint32_t addr_lsb)
{
    uint32_t msb;
    uint32_t msb_prev;
    uint32_t lsb;

    /* Do loop ensures 64-bit value doesn't have rollover issues. */
    msb = read_reg(blk_id, addr_msb);
    do {
        msb_prev = msb;
        lsb = read_reg(blk_id, addr_lsb);
        msb = read_reg(blk_id, addr_msb);
    } while (msb != msb_prev);

    return (((uint64_t)msb) << 32) | ((uint64_t)lsb);
}

void
base_device_common::write_reg_ctrl_mask(
    uint32_t blk_id,
    uint32_t addr,
    uint32_t mask,
    bool set)
{
    uint32_t ctl_reg;
    ctl_reg = read_reg(blk_id, addr);
    if (set)
        ctl_reg |= mask;
    else
        ctl_reg &= ~mask;
    write_reg(blk_id, addr, ctl_reg);
}


bool
base_device_common::set_attrs(const pmt::pmt_t &config)
{
    bool status = true;

    /* If the PMT is a list, assume it's a list of pairs and recurse for each */
    /* Works for dict too */
    try {
        /* Because of PMT is just broken you and can't distinguish between
         * pair and dict, we have to call length() and see if it will throw
         * or not ... */
        if (pmt::length(config) > 0) {
            for (int i=0; i<pmt::length(config); i++) {
                if (~set_attrs(pmt::nth(i, config)))
                    status = false;
            }
            return status;
        }
    } catch(...) { }

    if (pmt::is_pair(config)) {
        pmt::pmt_t key(pmt::car(config));
        pmt::pmt_t val(pmt::cdr(config));

        if (!pmt::is_symbol(key) || !pmt::is_symbol(val))
            return false;

        return set_attr(
            pmt::symbol_to_string(key),
            pmt::symbol_to_string(val));
    }

    return false;
}

bool
base_device_common::set_attrs(const std::map<std::string, std::string> &config)
{
    bool status = true;
    std::map<std::string, std::string>::const_iterator it;

    for(it = config.begin(); it != config.end(); ++it) {
        if (!set_attr(it->first, it->second)) {
            status = false;
        }
    }
    return status;
}


bool
base_device_common::set_attr(
    const std::string &key,
    const std::string &val)
{
    int ret;
    struct iio_channel *chn = NULL;
    const char *attr = NULL;

    std::cout << "set_attr(" << key << ", " << val << ")\n";

    auto range = m_attr_alias.equal_range(key);

    if (range.first == range.second) {
        ret = iio_device_identify_filename(m_phy, key.c_str(), &chn, &attr);
        if (ret)
            return false;
        return set_attr_internal(chn, attr, val.c_str());
    }

    for (auto it = range.first; it != range.second; it++) {
        ret = iio_device_identify_filename(m_phy, it->second.c_str(), &chn, &attr);
        if (ret)
            continue;

        set_attr_internal(chn, attr, val.c_str());
    }

    return true;
}


bool
base_device_common::set_attr_internal(
    struct iio_channel *chn,
    const char *attr,
    const char *val)
{
    int ret;

    std::cout << "set_attr_internal(" << attr << ", " << val << ")\n";

    if (chn)
        ret = iio_channel_attr_write(chn, attr, val);
    else if (iio_device_find_attr(m_phy, attr))
        ret = iio_device_attr_write(m_phy, attr, val);
    else
        ret = iio_device_debug_attr_write(m_phy, attr, val);

    if (ret < 0) {
        std::cerr << "Unable to write attribute " << attr
                  <<  ": " << ret << std::endl;
        return false;
    }
    return true;
}


bool
base_device_common::setup_filter(
    const char *filter,
    unsigned long samplerate,
    bool auto_filter)
{
    if (auto_filter) {
        if (ad9361_set_bb_rate(m_phy, samplerate))
            return false;
    } else if (filter && filter[0]) {
        std::string filt(filter);
        return load_filter_file(filt);
    }
    return true;
}

bool
base_device_common::load_filter_file(std::string &filter)
{
    if (filter.empty() || !iio_device_find_attr(m_phy, "filter_fir_config"))
        return false;

    std::ifstream ifs(filter.c_str(), std::ifstream::binary);
    if (!ifs)
        return false;

    /* Here, we verify that the filter file contains data for both RX+TX. */
    {
        char buf[256];

        do {
            ifs.getline(buf, sizeof(buf));
        } while (!(buf[0] == '-' || (buf[0] >= '0' && buf[0] <= '9')));

        std::string line(buf);
        if (line.find(',') == std::string::npos)
            throw std::runtime_error("Incompatible filter file");
    }

    ifs.seekg(0, ifs.end);
    int length = ifs.tellg();
    ifs.seekg(0, ifs.beg);

    char *buffer = new char [length];

    ifs.read(buffer, length);
    ifs.close();

    int ret = iio_device_attr_write_raw(
        m_phy, "filter_fir_config", buffer, length);

    delete[] buffer;
    return ret > 0;
}

int
base_device_common::load_personality(const std::string &personality, bool force)
{
    std::ostringstream cmd;

    /* This prevents swapping the bitfile out from under an already
       initialized block. If we've already loaded the bitfile,
       we ignore further attempts. If it tries to load a different
       bitfile, it's an error. */
    if (g_fpga_loaded) {
        if (personality != g_fpga_loaded_personality) {
            throw std::runtime_error(
                "Trying to load two different bitfiles in "
                "the same flowgraph is unsupported");
        }
        return 0;
    }

    g_fpga_loaded = true;
    g_fpga_loaded_personality = personality;

    cmd << "fpgaloader switch " << personality;
    if (force) {
        cmd << " -f ";
    }

    return system(cmd.str().c_str());
}

bool
base_device_common::check_user_blkid(
    uint16_t blk_id,
    uint16_t version_min,
    uint16_t version_max,
    bool assert_on_fail)
{
    uint32_t blk_info = read_reg(REG_BLK_USER, REG_USER_BLKID);
    uint16_t id;
    uint16_t ver;
    bool failed;

    id = blk_info >> 16;
    ver = blk_info & 0xffff;

    failed = (blk_id != id) || (ver < version_min) || (ver > version_max);

    if (failed && assert_on_fail) {
        std::ostringstream msg;
        msg << boost::format(
            "Block ID did not match. Received: 0x%04x ver=%d"
            ", Expected: 0x%04x (%d-%d)")
            % id % ver % blk_id % version_min % version_max;
        throw std::runtime_error(msg.str());
    }
    return failed;
}


} /* namespace iio */
} /* namespace gr */

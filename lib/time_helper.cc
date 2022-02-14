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

#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include "base_device_common.h"
#include "time_helper.h"
#include "common_registers.h"

namespace gr {
namespace rwt {

time_helper::time_helper() :
    m_common(NULL)
{
}


void
time_helper::setup(
    std::shared_ptr<base_device_common> common,
    bool use_tags)
{
    m_common = common;

    /* Disable all tags are disabled. */
    m_common->write_reg_ctrl_mask(
        REG_BLK_USER,
        REG_DEFAULT_CTL,
        MASK_DEFAULT_CTL_OVERFLOW_ENA |
        MASK_DEFAULT_CTL_HOLD_ENA |
        MASK_DEFAULT_CTL_PPS_ENA,
        use_tags);

    /* Use internal pps by default. */
    m_common->write_reg_ctrl_mask(
        REG_BLK_USER,
        REG_DEFAULT_CTL,
        MASK_DEFAULT_CTL_WHICH_PPS,
        PPS_USE_INTERNAL);
}

bool
time_helper::ctrl_reg_handler(
    const std::string key,
    const std::string value,
    unsigned long samplerate)
{
    uint64_t secs;
    uint32_t nsecs;
    uint64_t sample_idx;
    unsigned long use_ext_pps;

    if (key == "use_ext_pps") {
        use_ext_pps = strtoul(value.c_str(), NULL, 0);
        m_common->write_reg_ctrl_mask(
            REG_BLK_USER,
            REG_DEFAULT_CTL,
            MASK_DEFAULT_CTL_WHICH_PPS,
            !!use_ext_pps);

    } else if (key == "time_next_pps") {
        sscanf(value.c_str(), "%" SCNu64 ",%" SCNu32, &secs, &nsecs);
        set_time(TIME_MODE_NEXT_PPS, samplerate, secs, nsecs);
    } else if (key == "time_now") {
        sscanf(value.c_str(), "%" SCNu64 ",%" SCNu32, &secs, &nsecs);
        set_time(TIME_MODE_NOW, samplerate, secs, nsecs);
    } else if (key == "sample_idx_next_pps") {
        sscanf(value.c_str(), "%" SCNu64, &sample_idx);
        set_sample_idx(sample_idx, TIME_MODE_NEXT_PPS);
    } else if (key == "sample_idx_now") {
        sscanf(value.c_str(), "%" SCNu64, &sample_idx);
        set_sample_idx(sample_idx, TIME_MODE_NOW);
    } else {
        return false;
    }

    return true;
}


void
time_helper::set_time(
    bool mode,
    uint32_t samplerate,
    uint64_t secs,
    uint32_t nsecs)
{
    uint64_t sample_idx;

    sample_idx = secs * samplerate;
    sample_idx += (((uint64_t)nsecs) * ((uint64_t)samplerate)) / 1000000000;
    set_sample_idx(sample_idx, mode);
}


void
time_helper::set_sample_idx(
    uint64_t sample_idx,
    bool mode)
{
    m_common->write_reg_ctrl_mask(
        REG_BLK_USER,
        REG_DEFAULT_CTL,
        MASK_DEFAULT_CTL_SAMPLE_IDX,
        mode);

    m_common->write_reg_u64(
        REG_BLK_USER,
        REG_DEFAULT_SET_SAMPLE_IDX_MSB,
        REG_DEFAULT_SET_SAMPLE_IDX_LSB,
        sample_idx);
}


void
time_helper::get_time(
    bool which,
    uint32_t samplerate,
    uint64_t *secs,
    uint32_t *nsecs)
{
    uint64_t sample_idx = get_sample_idx(which);
    uint64_t leftover;

    if (!samplerate)
        return;

    *secs = sample_idx / samplerate;

    leftover = sample_idx % samplerate;
    *nsecs = (uint32_t)(leftover * 1000000000 / ((uint64_t)samplerate));
}

pmt::pmt_t
time_helper::get_time_pmt(
    bool which,
    unsigned long samplerate)
{
    uint64_t secs;
    uint32_t nsecs;

    get_time(
        which,
        samplerate,
        &secs,
        &nsecs);

    return pmt::cons(pmt::from_uint64(secs), pmt::from_uint64(nsecs));
}

uint64_t
time_helper::get_sample_idx(bool which)
{
    uint64_t ret;

    if (which == INDEX_ADC) {
        ret = m_common->read_reg_u64(
            REG_BLK_USER,
            REG_DEFAULT_GET_SAMPLE_IDX_ADC_MSB,
            REG_DEFAULT_GET_SAMPLE_IDX_ADC_LSB);
    } else {
        ret = m_common->read_reg_u64(
            REG_BLK_USER,
            REG_DEFAULT_GET_SAMPLE_IDX_DAC_MSB,
            REG_DEFAULT_GET_SAMPLE_IDX_DAC_LSB);
    }

    return ret;
}


} /* namespace rwt */
} /* namespace gr */

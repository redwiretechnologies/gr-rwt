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

#ifndef FMCOMMS_TIME_HELPER_H
#define FMCOMMS_TIME_HELPER_H

#include <stdint.h>

#include "base_device_common.h"


namespace gr {
namespace rwt {

class time_helper
{
public:
    time_helper();
    void setup(
        std::shared_ptr<base_device_common> common,
        bool use_tags);

    bool ctrl_reg_handler(
        const std::string key,
        const std::string value,
        unsigned long samplerate);

    void set_time(
        bool mode,
        uint32_t samplerate,
        uint64_t secs,
        uint32_t nsecs);

    void set_sample_idx(
        uint64_t sample_idx,
        bool mode);

    void get_time(
        bool which,
        uint32_t samplerate,
        uint64_t *secs,
        uint32_t *nsecs);

    pmt::pmt_t get_time_pmt(
        bool which,
        unsigned long samplerate);

    uint64_t get_sample_idx(bool which);

    const static bool TIME_MODE_NOW = false;
    const static bool TIME_MODE_NEXT_PPS = true;

    const static bool INDEX_ADC = false;
    const static bool INDEX_DAC = true;

    const static bool PPS_USE_INTERNAL = false;
    const static bool PPS_USE_EXTERNAL = true;

private:
    std::shared_ptr<base_device_common> m_common;
};

} /* namespace iio */
} /* namespace gr */

#endif /* FMCOMMS_TIME_HELPER_H */

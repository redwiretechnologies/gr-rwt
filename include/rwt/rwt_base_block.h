/* -*- c++ -*- */
/*
 * Copyright 2018 Red Wire Technologies.
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

#ifndef RWT_RWT_BASE_BLOCK_H
#define RWT_RWT_BASE_BLOCK_H

#include <rwt/api.h>

#include <stdint.h>
#include <map>
#include <string>
#include <memory>

#include <boost/function.hpp>
#include <gnuradio/io_signature.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace rwt {

class RWT_API rwt_base_block : public sync_block
{
public:
    rwt_base_block(void) {}

    rwt_base_block(
        const std::string &name,
        io_signature::sptr input_signature,
        io_signature::sptr output_signature);

    virtual void set_config(std::string key, std::string value) = 0;
    virtual void config_map_handler(
        const std::map<std::string, std::string> &config) = 0;

    virtual void config_msg_handler(pmt::pmt_t msg) = 0;

    virtual int work(
        int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items) = 0;
};

} /* namespace rwt */
} /* namespace gr */

#endif /* RWT_RWT_BASE_BLOCK_H */

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

#ifndef RWT_BASE_BLOCK_IMPL_H
#define RWT_BASE_BLOCK_IMPL_H

#include <rwt/api.h>

#include <stdint.h>
#include <map>
#include <string>
#include <memory>

#include <boost/function.hpp>
#include <gnuradio/io_signature.h>
#include <rwt/rwt_base_block.h>

#include "base_device_common.h"
#include "base_device_source.h"
#include "base_device_sink.h"

namespace gr {
namespace rwt {

typedef boost::function<void(std::string, std::string)> config_handler_t;

class rwt_base_block_impl : virtual public rwt_base_block
{
public:
    rwt_base_block_impl(
        const std::string &name,
        io_signature::sptr input_signature,
        io_signature::sptr output_signature,
        unsigned int source_buffer_size,
        unsigned int sink_buffer_size,
        unsigned int reg_base_addr,
        bool is_source,
        bool is_sink,
        const char *personality,
        bool force_reload,
        const char *phy_name,
        const char *rx_name,
        const char *tx_name);


    ~rwt_base_block_impl();

    void set_config_handler(
        const std::string key,
        config_handler_t handler);

    void set_config(std::string key, std::string value);
    void config_map_handler(const std::map<std::string, std::string> &config);
    void config_msg_handler(pmt::pmt_t msg);
    int buffer_next(uint8_t **data, bool from_source);
    void buffer_done(bool from_source);

    int work(
        int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

protected:
    std::map<std::string,config_handler_t> m_config_handlers;
    bool m_is_source;
    bool m_is_sink;

    std::shared_ptr<base_device_common> m_common;
    std::shared_ptr<base_device_source> m_source;
    std::shared_ptr<base_device_sink> m_sink;

};

} /* namespace rwt */
} /* namespace gr */

#endif /* RWT_BASE_BLOCK_IMPL_H */

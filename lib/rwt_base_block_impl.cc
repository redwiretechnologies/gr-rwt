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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "base_device_common.h"
#include "base_device_sink.h"
#include "base_device_source.h"
#include "rwt_base_block_impl.h"

#define MSG_PORT_STATUS  pmt::mp("status")
#define MSG_PORT_CONTROL pmt::mp("ctrl")

namespace gr {
namespace rwt {

rwt_base_block::rwt_base_block(
    const std::string &name,
    io_signature::sptr input_signature,
    io_signature::sptr output_signature) :
    sync_block(name, input_signature, output_signature)
{
}

rwt_base_block_impl::rwt_base_block_impl(
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
    const char *phy_name = "ad9361-phy",
    const char *rx_name = "cf-ad9361-lpc",
    const char *tx_name = "cf-ad9361-dds-core-lpc") :
    rwt_base_block(name, input_signature, output_signature),
    m_is_source(is_source),
    m_is_sink(is_sink),
    m_common(NULL),
    m_source(NULL),
    m_sink(NULL)
{

    if (!m_is_sink && !m_is_source) {
        throw std::runtime_error(
            "Either 'is_sink' and 'is_source' must be set.");
    }

    m_common = std::make_shared<base_device_common>(
        reg_base_addr,
        personality,
        force_reload,
        phy_name,
        rx_name,
        tx_name);

    if (m_is_source) {
        m_source = std::make_shared<base_device_source>(
            m_common, source_buffer_size);
    }

    if (m_is_sink) {
        m_sink = std::make_shared<base_device_sink>(
            m_common, sink_buffer_size);
    }

    message_port_register_out(MSG_PORT_STATUS);
    message_port_register_in(MSG_PORT_CONTROL);
    set_msg_handler(
        MSG_PORT_CONTROL,
        boost::bind(&rwt_base_block_impl::config_msg_handler, this, _1));
}

/*
 * Our virtual destructor.
 */
rwt_base_block_impl::~rwt_base_block_impl()
{
}

void
rwt_base_block_impl::set_config_handler(
    const std::string key,
    config_handler_t handler)
{
    m_config_handlers[key] = handler;
}

void
rwt_base_block_impl::set_config(std::string key, std::string value)
{
    std::map<std::string,config_handler_t>::iterator it = m_config_handlers.find(key);

    if (it != m_config_handlers.end()) {
        return it->second(key, value);
    }

    m_common->set_attr(key, value);
}

void
rwt_base_block_impl::config_map_handler(const std::map<std::string, std::string> &config)
{
    std::map<std::string, std::string>::const_iterator it;

    for(it = config.begin(); it != config.end(); ++it) {
        set_config(it->first, it->second);
    }
}

void
rwt_base_block_impl::config_msg_handler(pmt::pmt_t msg)
{
    /* If the PMT is a list, assume it's a list of pairs and recurse for each */
    /* Works for dict too */
    try {
        /* Because of PMT is just broken you and can't distinguish between
         * pair and dict, we have to call length() and see if it will throw
         * or not ... */
        if (pmt::length(msg) > 0) {
            for (int i=0; i<pmt::length(msg); i++)
                config_msg_handler(pmt::nth(i, msg));
            return;
        }
    } catch(...) { }

    if (pmt::is_pair(msg)) {
        pmt::pmt_t key(pmt::car(msg));
        pmt::pmt_t val(pmt::cdr(msg));

        if (!pmt::is_symbol(key) || !pmt::is_symbol(val))
            return;

        set_config(
            pmt::symbol_to_string(key),
            pmt::symbol_to_string(val));
    }
}


int
rwt_base_block_impl::work(
    int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    (void)input_items;
    (void)output_items;
    (void)noutput_items;
    return -1;
}

} /* namespace rwt */
} /* namespace gr */

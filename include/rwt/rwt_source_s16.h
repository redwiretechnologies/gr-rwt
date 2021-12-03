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


#ifndef INCLUDED_RWT_RWT_SOURCE_S16_H
#define INCLUDED_RWT_RWT_SOURCE_S16_H

#include <rwt/api.h>
#include <gnuradio/sync_block.h>
#include "rwt_base_block.h"

namespace gr {
namespace rwt {

/*!
 * \brief Source block for rwt2 which works w/ shorts instead of complex.
 * \ingroup rwt
 *
 */
class RWT_API rwt_source_s16 : virtual public rwt_base_block
{
public:
    typedef boost::shared_ptr<rwt_source_s16> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of rwt::rwt_source_s16.
     *
     * To avoid accidental use of raw pointers, rwt::rwt_source_s16's
     * constructor is in a private implementation
     * class. rwt::rwt_source_s16::make is the public interface for
     * creating new instances.
     */
    static sptr make(
        const pmt::pmt_t config,
        bool ch1_en,
        bool ch2_en,
        unsigned int reg_base_addr,
        const char *filter,
        bool use_tags,
        bool auto_filter,
        bool force_reload,
        unsigned int buffer_size);

    virtual pmt::pmt_t get_time() = 0;
    virtual uint64_t get_sample_idx() = 0;

};

} // namespace rwt
} // namespace gr

#endif /* INCLUDED_RWT_RWT_SOURCE_S16_H */

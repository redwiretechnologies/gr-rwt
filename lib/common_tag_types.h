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

#ifndef INCLUDED_COMMON_TAG_TYPES_H
#define INCLUDED_COMMON_TAG_TYPES_H

namespace gr {
namespace rwt {

#define RWT_TAG_IDX_TIME      0x01
#define RWT_TAG_IDX_PPS       0x02
#define RWT_TAG_IDX_OVERFLOW  0x03
#define RWT_TAG_IDX_HOLD      0x04
#define RWT_TAG_IDX_SOB       0x05
#define RWT_TAG_IDX_EOB       0x06

#define RWT_TAG_PMT_TIME      pmt::mp("time")
#define RWT_TAG_PMT_PPS       pmt::mp("pps")
#define RWT_TAG_PMT_OVERFLOW  pmt::mp("overflow")

#define RWT_TAG_PMT_TX_TIME   pmt::mp("tx_time")
#define RWT_TAG_PMT_TX_SAMPLE pmt::mp("tx_sample")
#define RWT_TAG_PMT_TX_SOB    pmt::mp("tx_sob")
#define RWT_TAG_PMT_TX_EOB    pmt::mp("tx_eob")

} // namespace rwt
} // namespace gr

#endif /* INCLUDED_COMMON_TAG_TYPES_H */

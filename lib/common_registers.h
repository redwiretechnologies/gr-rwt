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

#ifndef INCLUDED_COMMON_REGS_H
#define INCLUDED_COMMON_REGS_H

namespace gr {
namespace rwt {

#define REG_BLK_USER 0
#define REG_BLK_ADC  1
#define REG_BLK_DAC  2

#define REG_USER_BLKID         0

#define REG_ADC_CTL            0
#define REG_ADC_ESCAPE_MSB     1
#define REG_ADC_ESCAPE_LSB     2

#define REG_DAC_CTL            0
#define REG_DAC_ESCAPE_MSB     1
#define REG_DAC_ESCAPE_LSB     2

#define REG_DEFAULT_BLKID                   REG_USER_BLKID
#define REG_DEFAULT_CTL                     1
  #define MASK_DEFAULT_CTL_WHICH_PPS           0x0001
  #define MASK_DEFAULT_CTL_OVERFLOW_ENA        0x0002
  #define MASK_DEFAULT_CTL_PPS_ENA             0x0004
  #define MASK_DEFAULT_CTL_HOLD_ENA            0x0008
  #define MASK_DEFAULT_CTL_SAMPLE_IDX          0x0010
#define REG_DEFAULT_STATUS                  2
  #define MASK_DEFAULT_STATUS_UNDERFLOW        0x0001
  #define MASK_DEFAULT_STATUS_OVERFLOW         0x0002
  #define MASK_DEFAULT_STATUS_HOLD_DIFF_VALID  0x0004
#define REG_DEFAULT_SET_SAMPLE_IDX_MSB      3
#define REG_DEFAULT_SET_SAMPLE_IDX_LSB      4
#define REG_DEFAULT_GET_SAMPLE_IDX_ADC_MSB  5
#define REG_DEFAULT_GET_SAMPLE_IDX_ADC_LSB  6
#define REG_DEFAULT_GET_SAMPLE_IDX_DAC_MSB  7
#define REG_DEFAULT_GET_SAMPLE_IDX_DAC_LSB  8
#define REG_DEFAULT_OVERFLOW_WAIT           9
#define REG_DEFAULT_HOLD_DIFF_MSB           10
#define REG_DEFAULT_HOLD_DIFF_LSB           11

#define MIN_SAMPLE_RATE    600000
} // namespace rwt
} // namespace gr

#endif /* INCLUDED_COMMON_REGS_H */

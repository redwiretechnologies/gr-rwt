/*
 * Copyright 2021 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(rwt_source_s16.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(53171e5c0445ce33deef80eeee326c5c)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <rwt/rwt_source_s16.h>
#include <rwt/rwt_base_block.h>
// pydoc.h is automatically generated in the build directory
#include <rwt_source_s16_pydoc.h>

void bind_rwt_source_s16(py::module& m)
{

    using rwt_source_s16    = gr::rwt::rwt_source_s16;
    using rwt_base_block    = gr::rwt::rwt_base_block;


    py::class_<rwt_source_s16,
         rwt_base_block,
	     gr::sync_block,
         gr::block,
         gr::basic_block,
        std::shared_ptr<rwt_source_s16>>(m, "rwt_source_s16", D(rwt_source_s16))

        .def(py::init(&rwt_source_s16::make),
           py::arg("config"),
           py::arg("ch1_en"),
           py::arg("ch2_en"),
           py::arg("reg_base_addr"),
           py::arg("*filter"),
           py::arg("use_tags"),
           py::arg("auto_filter"),
           py::arg("*personality"),
           py::arg("force_reload"),
           py::arg("buffer_size"),
           py::arg("*phy_name"),
           py::arg("*rx_name"),
           py::arg("*tx_name"),
           D(rwt_source_s16,make)
        )




        ;




}









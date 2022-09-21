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
/* BINDTOOL_HEADER_FILE(rwt_source.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(ef227628157524957207568d46a5f76f)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <rwt/rwt_source.h>
#include <rwt/rwt_base_block.h>
// pydoc.h is automatically generated in the build directory
#include <rwt_source_pydoc.h>

void bind_rwt_source(py::module& m)
{

    using rwt_source     = gr::rwt::rwt_source;
    using rwt_base_block = gr::rwt::rwt_base_block;


    py::class_<rwt_source,
         rwt_base_block,
	     gr::sync_block,
         gr::block,
         gr::basic_block,
        std::shared_ptr<rwt_source>>(m, "rwt_source", D(rwt_source))

        .def(py::init(&rwt_source::make),
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
           D(rwt_source,make)
        )




        ;




}








# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT rwt_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT rwt_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND rwt_python_files
    rwt_base_block_python.cc
    rwt_sink_python.cc
    rwt_source_python.cc
    rwt_source_s16_python.cc
    python_bindings.cc)

GR_PYBIND_MAKE_OOT(rwt
   ../..
   gr::rwt
   "${rwt_python_files}")

install(TARGETS rwt_python DESTINATION ${GR_PYTHON_DIR}/rwt COMPONENT pythonapi)

/* -*- c++ -*- */

#define RWT_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "rwt_swig_doc.i"

%{
#include "rwt/rwt_base_block.h"
#include "rwt/rwt_sink.h"
#include "rwt/rwt_source.h"
#include "rwt/rwt_source_s16.h"
%}

%include "rwt/rwt_base_block.h"

%include "rwt/rwt_sink.h"
GR_SWIG_BLOCK_MAGIC2(rwt, rwt_sink);
%include "rwt/rwt_source.h"
GR_SWIG_BLOCK_MAGIC2(rwt, rwt_source);
%include "rwt/rwt_source_s16.h"
GR_SWIG_BLOCK_MAGIC2(rwt, rwt_source_s16);

/* -*- c++ -*- */
/*
 * Copyright 2025 gr-reader author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_READER_TAG_DECODER_H
#define INCLUDED_READER_TAG_DECODER_H

#include <gnuradio/block.h>
#include <gnuradio/reader/api.h>
#include <gnuradio/reader/global_vars.h>
namespace gr {
namespace reader {

/*!
 * \brief <+description of block+>
 * \ingroup reader
 *
 */
class READER_API tag_decoder : virtual public gr::block
{
public:
    typedef std::shared_ptr<tag_decoder> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of reader::tag_decoder.
     *
     * To avoid accidental use of raw pointers, reader::tag_decoder's
     * constructor is in a private implementation
     * class. reader::tag_decoder::make is the public interface for
     * creating new instances.
     */
    static sptr make(float sample_rate);
};

} // namespace reader
} // namespace gr

#endif /* INCLUDED_READER_TAG_DECODER_H */

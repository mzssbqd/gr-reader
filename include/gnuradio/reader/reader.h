/* -*- c++ -*- */
/*
 * Copyright 2025 gr-reader author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_READER_READER_H
#define INCLUDED_READER_READER_H

#include <gnuradio/block.h>
#include <gnuradio/reader/api.h>
#include <vector>

namespace gr {
namespace reader {

/*!
 * \brief <+description of block+>
 * \ingroup reader
 *
 */
class READER_API reader : virtual public gr::block
{
public:
    typedef std::shared_ptr<reader> sptr;
    /*!
     * \brief Return a shared_ptr to a new instance of reader::reader.
     *
     * To avoid accidental use of raw pointers, reader::reader's
     * constructor is in a private implementation
     * class. reader::reader::make is the public interface for
     * creating new instances.
     */
    static sptr make(float sample_rate, float dac_rate, int num_sines, std::vector<float> freqs, std::vector<float> amps);
    virtual void print_results() = 0;
};

} // namespace reader
} // namespace gr

#endif /* INCLUDED_READER_READER_H */

/* -*- c++ -*- */
/*
 * Copyright 2025 gr-reader author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_READER_GATE_H
#define INCLUDED_READER_GATE_H

#include <gnuradio/block.h>
#include <gnuradio/reader/api.h>

namespace gr {
namespace reader {

/*!
 * \brief <+description of block+>
 * \ingroup reader
 *
 */
class READER_API gate : virtual public gr::block
{
public:
    typedef std::shared_ptr<gate> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of reader::gate.
     *
     * To avoid accidental use of raw pointers, reader::gate's
     * constructor is in a private implementation
     * class. reader::gate::make is the public interface for
     * creating new instances.
     */
    static sptr make(float sample_rate);
};

} // namespace reader
} // namespace gr

#endif /* INCLUDED_READER_GATE_H */

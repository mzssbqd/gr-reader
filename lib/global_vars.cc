/* -*- c++ -*- */
/*
 * Copyright 2025 mzssbqd.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/io_signature.h>
#include <gnuradio/reader/global_vars.h>
#include <iostream>

namespace gr {
namespace reader {
    int FIXED_Q = 0;
    int MAX_NUM_QUERIES = 1000;
    bool P_DOWN = false;
    int SESSION[2] = {0, 0};

    void set_reader_config(int fixed_q, int max_num_queries, bool p_down, int session_0, int session_1)
    {
        if (fixed_q < 0) {
            fixed_q = 0;
        } else if (fixed_q > 15) {
            fixed_q = 15;
        }

        FIXED_Q = fixed_q;
        MAX_NUM_QUERIES = max_num_queries;
        P_DOWN = p_down;
        SESSION[0] = session_0;
        SESSION[1] = session_1;

        if (reader_state != nullptr) {
            reader_state->reader_stats.max_slot_number = pow(2, FIXED_Q);
        }
    }

    READER_STATE * reader_state;
    void initialize_reader_state()
    {
        reader_state = new READER_STATE;

        reader_state-> reader_stats.n_queries_sent = 0;
        reader_state-> reader_stats.n_epc_correct = 0;
        reader_state->reader_stats.unique_tags_round.clear();
        reader_state->reader_stats.tag_reads.clear();
        reader_state->reader_stats.tag_epc_hex.clear();
 
        reader_state-> status            = RUNNING;
        reader_state-> gen2_logic_status = START;
        reader_state-> gate_status       = GATE_SEEK_RN16;
        reader_state-> decoder_status    = DECODER_DECODE_RN16;

        reader_state-> reader_stats.max_slot_number = pow(2,FIXED_Q);

        reader_state-> reader_stats.cur_inventory_round = 1;
        reader_state-> reader_stats.cur_slot_number     = 1;

        gettimeofday (&reader_state-> reader_stats.start, NULL);
    }
} /* namespace reader */
} /* namespace gr */

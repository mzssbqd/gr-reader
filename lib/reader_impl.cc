/* -*- c++ -*- */
/*
 * Copyright 2025 gr-reader author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "reader_impl.h"
#include <gnuradio/io_signature.h>
#include <sys/time.h>
#include <gnuradio/reader/global_vars.h>
#include <cmath>

namespace gr {
namespace reader {

using input_type = float;
using output_type = float;
reader::sptr reader::make(float sample_rate, float dac_rate, int num_sines, std::vector<float> freqs, std::vector<float> amps) {
    return gnuradio::make_block_sptr<reader_impl>(sample_rate, dac_rate, num_sines, freqs, amps); 
}


/*
 * The private constructor
 */
reader_impl::reader_impl(float sample_rate, float dac_rate, int num_sines, std::vector<float> freqs, std::vector<float> amps)
    : gr::block("reader",
                gr::io_signature::make(
                    1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                gr::io_signature::make(
                    1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
                    d_num_sines(num_sines), d_freqs(freqs), d_amps(amps)
{
    GR_LOG_INFO(d_logger, "block initialized");

    sample_d = 1.0 / dac_rate * pow(10,6);

    // Number of samples for transmitting

    n_data0_s = 2 * PW_D / sample_d;
    n_data1_s = 4 * PW_D / sample_d;
    n_pw_s    = PW_D    / sample_d;
    n_cw_s    = CW_D    / sample_d;
    n_delim_s = DELIM_D / sample_d;
    n_trcal_s = TRCAL_D / sample_d;

    // CW waveforms of different sizes
    n_cwquery_s   = (T1_D+T2_D+RN16_D)/sample_d;     //RN16
    n_cwack_s     = (3*T1_D+T2_D+EPC_D)/sample_d;    //EPC   if it is longer than nominal it wont cause tags to change inventoried flag
    n_p_down_s    = (P_DOWN_D)/sample_d;
    n_extra_cw    = (T1_D+T2_D+EPC_D)/sample_d;

    p_down.resize(n_p_down_s);        // Power down samples
    cw_query.resize(n_cwquery_s);      // Sent after query/query rep
    cw_ack.resize(n_cwack_s);          // Sent after ack
    extra_cw.resize(n_cwack_s);

    std::fill_n(cw_query.begin(), cw_query.size(), 1);
    std::fill_n(cw_ack.begin(), cw_ack.size(), 1);

    // creat extra cw
    {

    std::vector<double> phase(num_sines, 0.0);        // φ_k：每路初始相位
    std::vector<double> phase_inc(num_sines, 0.0);    // Δφ_k：每路相位增量

    const double two_pi = 2.0 * M_PI;

    double max_cw_amps = 1;
    for (int k = 0; k < num_sines; k++) {
        phase_inc[k] = two_pi * (double)freqs[k] / (double)dac_rate;
        max_cw_amps -= amps[k];
    }
    
    for (size_t n = 0; n < extra_cw.size(); n++) {

        float s = max_cw_amps;  // base CW（你原来的 cw_ack/cw_query 就是全 1）

        for (int k = 0; k < num_sines; k++) {
            // amps[k]*cos(phase[k]) 是第 k 路在该样点的贡献（实数）
            s += (float)amps[k] * (float)std::cos(phase[k]);

            // 相位推进：下一个样点相位增加 Δφ_k
            phase[k] += phase_inc[k];

            // 相位回绕到 [0, 2π)：避免相位无限变大导致 cos 精度变差
            if (phase[k] >= two_pi) phase[k] -= two_pi;
            else if (phase[k] < 0.0) phase[k] += two_pi;
        }
        extra_cw[n] = s;
    }
    }

    // Construct vectors (resize() default initialization is zero)
    data_0.resize(n_data0_s);
    data_1.resize(n_data1_s);
    cw.resize(n_cw_s);
    delim.resize(n_delim_s);
    rtcal.resize(n_data0_s + n_data1_s);
    trcal.resize(n_trcal_s);

    // Fill vectors with data
    std::fill_n(data_0.begin(), data_0.size()/2, 1);
    std::fill_n(data_1.begin(), 3*data_1.size()/4, 1);
    std::fill_n(cw.begin(), cw.size(), 1);
    std::fill_n(rtcal.begin(), rtcal.size() - n_pw_s, 1); // RTcal
    std::fill_n(trcal.begin(), trcal.size() - n_pw_s, 1); // TRcal

    // create preamble
    preamble.insert( preamble.end(), delim.begin(), delim.end() );
    preamble.insert( preamble.end(), data_0.begin(), data_0.end() );
    preamble.insert( preamble.end(), rtcal.begin(), rtcal.end() );
    preamble.insert( preamble.end(), trcal.begin(), trcal.end() );

    // create framesync
    frame_sync.insert( frame_sync.end(), delim.begin() , delim.end() );
    frame_sync.insert( frame_sync.end(), data_0.begin(), data_0.end() );
    frame_sync.insert( frame_sync.end(), rtcal.begin() , rtcal.end() );
    
    // create query rep
    query_rep.insert( query_rep.end(), frame_sync.begin(), frame_sync.end());
    query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
    query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
    query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );
    query_rep.insert( query_rep.end(), data_0.begin(), data_0.end() );

    // create nak
    nak.insert( nak.end(), frame_sync.begin(), frame_sync.end());
    nak.insert( nak.end(), data_1.begin(), data_1.end() );
    nak.insert( nak.end(), data_1.begin(), data_1.end() );
    nak.insert( nak.end(), data_0.begin(), data_0.end() );
    nak.insert( nak.end(), data_0.begin(), data_0.end() );
    nak.insert( nak.end(), data_0.begin(), data_0.end() );
    nak.insert( nak.end(), data_0.begin(), data_0.end() );
    nak.insert( nak.end(), data_0.begin(), data_0.end() );
    nak.insert( nak.end(), data_0.begin(), data_0.end() );

    // init local buffer
    d_tx_buf.resize(0); d_tx_pos = 0;

    gen_query_bits();
    gen_query_adjust_bits();

}

/*
 * Our virtual destructor.
 */
reader_impl::~reader_impl() {
    print_results();
}

void reader_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = 0;
}

int reader_impl::general_work(int noutput_items,
                              gr_vector_int& ninput_items,
                              gr_vector_const_void_star& input_items,
                              gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

    std::vector<float> out_message; 
    // int n_output;
    int consumed = 0;
    int written = 0;

    // consumed = ninput_items[0];

    // 将本地缓冲区的数据先输出
    if (!d_tx_buf.empty()) 
    {
        GR_LOG_INFO(d_debug_logger, "Output Buffer");
        const size_t remain = d_tx_buf.size() - d_tx_pos;
        const size_t n = std::min(remain, (size_t)noutput_items);
        std::memcpy(out + written, d_tx_buf.data() + d_tx_pos, n * sizeof(float));
        d_tx_pos += n;
        written += n;
        if (d_tx_pos == d_tx_buf.size()) {
            d_tx_buf.clear();
            d_tx_pos = 0;
        }

        consume_each(0);
        return written;
    }

    // 清理缓冲区
    d_tx_buf.clear();
    d_tx_pos = 0;

    switch (reader_state->gen2_logic_status)
    {
        case START: {
            GR_LOG_INFO(d_debug_logger, "START");
            
            append_vec(d_tx_buf, cw_ack);
            reader_state->gen2_logic_status = SEND_QUERY;
        }
            break;

        case POWER_DOWN: {
            GR_LOG_INFO(d_debug_logger, "POWER DOWN");
            append_vec(d_tx_buf, p_down);
            reader_state->gen2_logic_status = START;
        }   
            break;

        case SEND_NAK_QR: {
            GR_LOG_INFO(d_debug_logger, "SEND NAK");
            append_vec(d_tx_buf, nak);
            append_vec(d_tx_buf, cw);
            reader_state->gen2_logic_status = SEND_QUERY_REP;
        }
            break;

        case SEND_NAK_Q: {
            GR_LOG_INFO(d_debug_logger, "SEND NAK");
            append_vec(d_tx_buf, nak);
            append_vec(d_tx_buf, cw);
            reader_state->gen2_logic_status = SEND_QUERY;
        }
            break;

        case SEND_QUERY: {

            GR_LOG_INFO(d_debug_logger, "QUERY");
            // GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " << reader_state->reader_stats.cur_inventory_round << " SLOT NUMBER : " << reader_state->reader_stats.cur_slot_number);

            reader_state->reader_stats.n_queries_sent +=1;

            // Controls the other two blocks
            reader_state->decoder_status = DECODER_DECODE_RN16;
            reader_state->gate_status    = GATE_SEEK_RN16;

            append_vec(d_tx_buf, preamble);

            for(size_t i = 0; i < query_bits.size(); i++)
            {
                if(query_bits[i] == 1)
                {
                    append_vec(d_tx_buf, data_1);
                }
                else
                {
                    append_vec(d_tx_buf, data_0);
                }
            }
            
            // Send CW for RN16
            append_vec(d_tx_buf, cw_query);

            // Return to IDLE
            reader_state->gen2_logic_status = IDLE;
        }
            break;

        case SEND_ACK: {
            GR_LOG_INFO(d_debug_logger, "SEND ACK");

            if (ninput_items[0] == RN16_BITS - 1)
            {
                // Controls the other two blocks
                reader_state->decoder_status = DECODER_DECODE_EPC;
                reader_state->gate_status    = GATE_SEEK_EPC;

                gen_ack_bits(in);
                
                // Send FrameSync
                append_vec(d_tx_buf, frame_sync);

                for(size_t i = 0; i < ack_bits.size(); i++)
                {
                    if(ack_bits[i] == 1)
                    {
                        append_vec(d_tx_buf, data_1);
                    }
                    else  
                    {
                        append_vec(d_tx_buf, data_0);
                    }
                }
                
                consumed = ninput_items[0];
                if(d_num_sines == 0) reader_state->gen2_logic_status = SEND_CW;
                else reader_state->gen2_logic_status = SEND_EXTRA_CW;
            }
        }
            break;

        case SEND_CW: {
            GR_LOG_INFO(d_debug_logger, "SEND CW");
            append_vec(d_tx_buf, cw_ack);
            reader_state->gen2_logic_status = IDLE;      // Return to IDLE
        }
            break;

        case SEND_EXTRA_CW: {
            GR_LOG_INFO(d_debug_logger, "SEND EXTRA CW");
            append_vec(d_tx_buf, extra_cw);
            reader_state->gen2_logic_status = IDLE;      // Return to IDLE
        }
            break;
        case SEND_QUERY_REP: {
            GR_LOG_INFO(d_debug_logger, "SEND QUERY_REP");
            // GR_LOG_INFO(d_debug_logger, "INVENTORY ROUND : " << reader_state->reader_stats.cur_inventory_round << " SLOT NUMBER : " << reader_state->reader_stats.cur_slot_number);
            
            // Controls the other two blocks
            reader_state->decoder_status = DECODER_DECODE_RN16;
            reader_state->gate_status    = GATE_SEEK_RN16;
            reader_state->reader_stats.n_queries_sent +=1;

            append_vec(d_tx_buf, query_rep);
            append_vec(d_tx_buf, cw_query);

            reader_state->gen2_logic_status = IDLE;    // Return to IDLE
        }
            break;
        
        case SEND_QUERY_ADJUST: {
            GR_LOG_INFO(d_debug_logger, "SEND QUERY_ADJUST");
            // Controls the other two blocks
            reader_state->decoder_status = DECODER_DECODE_RN16;
            reader_state->gate_status    = GATE_SEEK_RN16;
            reader_state->reader_stats.n_queries_sent +=1;  

            append_vec(d_tx_buf, frame_sync);

            for(size_t i = 0; i < query_adjust_bits.size(); i++)
            {
                if(query_adjust_bits[i] == 1)
                {
                    append_vec(d_tx_buf, data_1);
                }
                else
                {
                    append_vec(d_tx_buf, data_0);
                }
            }
            append_vec(d_tx_buf, cw_query);
            reader_state->gen2_logic_status = IDLE;    // Return to IDLE
        }
            break;

        default:
            // IDLE
            break;
        }
    
    // 将本地缓冲区的数据输出
    size_t to_send = std::min(d_tx_buf.size(), (size_t)noutput_items);
    if (to_send > 0) {
        std::memcpy(out + written, d_tx_buf.data(), to_send * sizeof(float));
        d_tx_pos = to_send;
        written += to_send;
        if (d_tx_pos == d_tx_buf.size()) {
            d_tx_buf.clear();
            d_tx_pos = 0;
        }
    }

    consume_each (consumed);
    return written;
}

/* Function adapted from https://www.cgran.org/wiki/Gen2 */
void reader_impl::crc_append(std::vector<float> & q)
{
    int crc[] = {1,0,0,1,0};

    for(int i = 0; i < 17; i++)
    {
        int tmp[] = {0,0,0,0,0};
        tmp[4] = crc[3];
        if(crc[4] == 1)
        {
            if (q[i] == 1)
            {
                tmp[0] = 0;
                tmp[1] = crc[0];
                tmp[2] = crc[1];
                tmp[3] = crc[2];
            }
            else
            {
                tmp[0] = 1;
                tmp[1] = crc[0];
                tmp[2] = crc[1];
                if(crc[2] == 1)
                {
                    tmp[3] = 0;
                }
                else
                {
                    tmp[3] = 1;
                }
            }
        }
        else
        {
            if (q[i] == 1)
            {
                tmp[0] = 1;
                tmp[1] = crc[0];
                tmp[2] = crc[1];
                if(crc[2] == 1)
                {
                    tmp[3] = 0;
                }
                else
                {
                    tmp[3] = 1;
                }
            }
            else
            {
                tmp[0] = 0;
                tmp[1] = crc[0];
                tmp[2] = crc[1];
                tmp[3] = crc[2];
            }
        }
        memcpy(crc, tmp, 5*sizeof(float));
    }
    for (int i = 4; i >= 0; i--) q.push_back(crc[i]);
}

void reader_impl::gen_query_bits()
{
    // int num_ones = 0, num_zeros = 0;

    query_bits.resize(0);
    query_bits.insert(query_bits.end(), &QUERY_CODE[0], &QUERY_CODE[4]);
    query_bits.push_back(DR);
    query_bits.insert(query_bits.end(), &M[0], &M[2]);
    query_bits.push_back(TREXT);
    query_bits.insert(query_bits.end(), &SEL[0], &SEL[2]);
    query_bits.insert(query_bits.end(), &SESSION[0], &SESSION[2]);
    query_bits.push_back(TARGET);

    query_bits.insert(query_bits.end(), &Q_VALUE[FIXED_Q][0], &Q_VALUE[FIXED_Q][4]);
    crc_append(query_bits);
}

void reader_impl::gen_ack_bits(const float * in)
{
    ack_bits.resize(0);
    ack_bits.insert(ack_bits.end(), &ACK_CODE[0], &ACK_CODE[2]);
    ack_bits.insert(ack_bits.end(), &in[0], &in[16]);
}

void reader_impl::gen_query_adjust_bits()
{
    query_adjust_bits.resize(0);
    query_adjust_bits.insert(query_adjust_bits.end(), &QADJ_CODE[0], &QADJ_CODE[4]);
    query_adjust_bits.insert(query_adjust_bits.end(), &SESSION[0], &SESSION[2]);
    query_adjust_bits.insert(query_adjust_bits.end(), &Q_UPDN[1][0], &Q_UPDN[1][3]);
}

void reader_impl::print_results()
{
    std::cout << "\n --------------------------" << std::endl;
    std::cout << "| Number of queries/queryreps sent : " << reader_state->reader_stats.n_queries_sent - 1 << std::endl;
    std::cout << "| Current Inventory round : "          << reader_state->reader_stats.cur_inventory_round << std::endl;
    std::cout << " --------------------------"            << std::endl;

    std::cout << "| Correctly decoded EPC : "  <<  reader_state->reader_stats.n_epc_correct     << std::endl;
    std::cout << "| Number of unique tags : "  <<  reader_state->reader_stats.tag_reads.size() << std::endl;

    std::map<int,int>::iterator it;

    for(it = reader_state->reader_stats.tag_reads.begin(); it != reader_state->reader_stats.tag_reads.end(); it++) 
    {
        std::cout << std::hex <<  "| Tag ID : " << it->first << "  ";
        std::cout << "Num of reads : " << std::dec << it->second << std::endl;
    }

    std::cout << " --------------------------" << std::endl;
}

} /* namespace reader */
} /* namespace gr */

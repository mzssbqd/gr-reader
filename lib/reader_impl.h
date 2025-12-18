/* -*- c++ -*- */
/*
 * Copyright 2025 gr-reader author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_READER_READER_IMPL_H
#define INCLUDED_READER_READER_IMPL_H

#include <gnuradio/reader/reader.h>
#include <vector>
#include <queue>
#include <fstream>


namespace gr {
namespace reader {

class reader_impl : public reader
{
private:

    /*!
    * \brief Reader 端波形生成相关成员与辅助函数（保持原命名，不做重构）。
    *
    * \details
    * 约定：本文件中带后缀 “_s” 的变量表示“样点数（samples）”，不是秒（seconds）。
    * s_rate/d_rate 用于把协议时序（微秒）换算为样点数；n_*_s 保存各段波形长度（samples）。
    * data_0/data_1/cw/preamble/query_bits/ack_bits 等向量缓存已生成的基带模板，运行时由状态机拼接输出。
    *
    * \note
    * - s_rate: 基带生成/处理采样率（Hz）。
    * - d_rate: 发射 DAC 采样率（Hz）；若 TX 链路存在插值/重采样，可能与 s_rate 不同。
    * - n_cwquery_s: Query 相关 CW 段长度（samples）。
    * - n_cwack_s:   ACK 相关 CW 段长度（samples）。
    * - n_p_down_s:  Power-down 段长度（samples）。
    * - sample_d:    单个样点时间间隔（用于时长换算，单位取决于实现，常见为 us/sample 或 s/sample）。
    * - n_data0_s/n_data1_s: PIE 编码 Data-0/Data-1 符号长度（samples）。
    * - n_cw_s/n_pw_s/n_delim_s/n_trcal_s: CW / PW / Delimiter / TRcal 段长度（samples）。
    *
    * \note
    * - data_0/data_1: Data-0/Data-1 的基带波形模板（通常为幅度开关序列）。
    * - cw/cw_query/cw_ack: 连续载波模板及其在 Query/ACK 前后的专用段。
    * - delim/frame_sync/preamble/rtcal/trcal: Gen2 下行帧结构相关模板片段。
    * - query_bits/ack_bits/query_rep/nak/query_adjust_bits: 各命令的比特序列（或已调制的模板，取决于实现）。
    * - p_down: power-down 模板（关载波一段时间以复位标签）。
    *
    * \note
    * - q_change: QueryAdjust 的 Q 调整方向：0=增，1=不变，2=减。
    *
    * \note
    * - gen_query_bits(): 生成 Query 命令比特序列。
    * - gen_query_adjust_bits(): 生成 QueryAdjust 命令比特序列（由 q_change 决定 UpDn 字段）。
    * - gen_ack_bits(in): 用解码得到的 RN16(handle) 生成 ACK 命令比特序列。
    * - crc_append(q): 对命令比特序列追加 CRC（Query/QueryAdjust 通常为 CRC5，具体以实现为准）。
    */
    int s_rate, d_rate,  n_cwquery_s,  n_cwack_s,n_p_down_s;
    float sample_d, n_data0_s, n_data1_s, n_cw_s, n_pw_s, n_delim_s, n_trcal_s, n_extra_cw;
    std::vector<float> data_0, data_1, cw, cw_ack, cw_query, delim, frame_sync, preamble, rtcal, trcal, query_bits, ack_bits, query_rep,nak, query_adjust_bits,p_down, extra_cw;
    int q_change; // 0-> increment, 1-> unchanged, 2-> decrement
    
    std::vector<float> d_tx_buf; size_t d_tx_pos; // 本地缓冲区以及缓冲区指针

    int d_num_sines;
    std::vector<float> d_freqs, d_amps;
    void gen_query_adjust_bits();
    void crc_append(std::vector<float> & q);
    void gen_query_bits();
    void gen_ack_bits(const float * in);

    static inline void append_vec(std::vector<float>& dst, const std::vector<float>& src) {
        dst.insert(dst.end(), src.begin(), src.end());
    }
public:
    reader_impl(float sample_rate, float dac_rate, int nums_sine, std::vector<float> freq, std::vector<float> amp);
    ~reader_impl();

    void print_results();
    
    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

};

} // namespace reader
} // namespace gr

#endif /* INCLUDED_READER_READER_IMPL_H */

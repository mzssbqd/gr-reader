/* -*- c++ -*- */
/*
 * Copyright 2025 gr-reader author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_READER_GATE_IMPL_H
#define INCLUDED_READER_GATE_IMPL_H

#include <gnuradio/reader/gate.h>
#include <gnuradio/reader/global_vars.h>
#include <vector>
namespace gr {
namespace reader {

class gate_impl : public gate
{
private:
    // 边沿检测状态（门限穿越计数）
    enum SIGNAL_STATE { NEG_EDGE, POS_EDGE };

    // 关键样点数（由 us * sample_rate / 1e6 换算）
    int n_samples, n_samples_T1, n_samples_PW, n_samples_TAG_BIT;

    // 窗口索引/长度与采样率
    int win_index, dc_index, win_length, dc_length, s_rate;

    // 自适应门限与脉冲计数
    float avg_ampl, num_pulses, sample_thresh;

    // 缓冲：滑窗幅度/能量、CW记录、DC估计
    std::vector<float> win_samples, cw_samples; // win 滑动窗口求平均值
    std::vector<gr_complex> dc_samples;
    gr_complex dc_est;     // DC偏置估计（输出常用 x - dc_est）

    SIGNAL_STATE signal_state; // 当前等待的边沿类型


public:
    gate_impl(int sample_rate);
    ~gate_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace reader
} // namespace gr

#endif /* INCLUDED_READER_GATE_IMPL_H */

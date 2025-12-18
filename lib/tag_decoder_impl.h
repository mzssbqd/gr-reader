/* -*- c++ -*- */
/*
 * Copyright 2025 gr-reader author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_READER_TAG_DECODER_IMPL_H
#define INCLUDED_READER_TAG_DECODER_IMPL_H

#include <gnuradio/reader/tag_decoder.h>
#include <vector>

namespace gr {
namespace reader {

class tag_decoder_impl : public tag_decoder
{
private:
    float n_samples_TAG_BIT;                 // 每个Tag比特对应的采样点数（samples/bit）
    int s_rate;                              // 采样率 Hz
    std::vector<float> pulse_bit;            // 比特模板/相关模板（用于检测或匹配滤波）
    float T_global;                          // 全局时间/周期参数（用于定时/同步）
    gr_complex h_est;                        // 信道估计复系数（幅度+相位）
    char* char_bits;                         // 解码后的硬判决比特缓存（0/1 或 '0'/'1'）

    std::vector<float> tag_detection_EPC(std::vector<gr_complex>& EPC_samples_complex, int index); // 从EPC窗口解码并输出软指标/幅度序列
    std::vector<float> tag_detection_RN16(std::vector<gr_complex>& RN16_samples_complex);          // 从RN16窗口解码并输出软指标/幅度序列
    int tag_sync(const gr_complex* in, int size);                                                  // 在输入采样中找到Tag回复起点并返回索引
    int check_crc(char* bits, int num_bits);                                                       // 对bit流做CRC校验并返回是否通过


public:
    tag_decoder_impl(float sample_rate, std::vector<int> output_sizes);
    ~tag_decoder_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace reader
} // namespace gr

#endif /* INCLUDED_READER_TAG_DECODER_IMPL_H */

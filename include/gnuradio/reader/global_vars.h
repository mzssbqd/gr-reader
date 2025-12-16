/* -*- c++ -*- */
/*
 * Copyright 2025 mzssbqd.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_READER_GLOBAL_VARS_H
#define INCLUDED_READER_GLOBAL_VARS_H

#include <gnuradio/reader/api.h>
#include <vector>
#include <map>
#include <sys/time.h>
#include <math.h>
#include <gnuradio/logger.h>

namespace gr {
namespace reader {
    enum STATUS {RUNNING, TERMINATED}; // reader状态
    enum GEN2_LOGIC_STATUS {SEND_QUERY, SEND_ACK, SEND_QUERY_REP, IDLE, SEND_CW, START, SEND_QUERY_ADJUST, SEND_NAK_QR, SEND_NAK_Q, POWER_DOWN};
    enum GATE_STATUS {GATE_OPEN, GATE_CLOSED, GATE_SEEK_RN16, GATE_SEEK_EPC, GATE_Handle};
    enum DECODER_STATUS {DECODER_DECODE_RN16, DECODER_DECODE_EPC};

    // 运行统计信息（run-time statistics）：不参与信号处理，只用于记录盘存过程与结果
    struct READER_STATS 
    {    
        int n_queries_sent;          // 已发送的 Query 类命令次数（Query / QueryRep / QueryAdjust 等，具体看实现里累加位置）
        int cur_inventory_round;     // 当前盘存轮次（inventory round）编号
        int cur_slot_number;         // 当前轮次内 slot 编号（0,1,2,...）
        int max_slot_number;         // 一轮的最大 slot 数，通常为 2^Q（代码里常由 FIXED_Q 决定）
        int max_inventory_round;     // 最大盘存轮次（达到后可终止）
        int n_epc_correct;           // CRC 校验通过的 EPC 次数（成功解码次数）
        std::vector<int> unique_tags_round; // 每轮盘存读到的“唯一标签数”（每轮结束 push_back 一次计数）
        std::map<int,int> tag_reads;         // 标签读数统计：tag_id -> 成功读到次数（tag_id 在实现里从 EPC 某些 bit 抽取）
        struct timeval start, end;   // 运行起止时间（用于耗时/吞吐统计）
    };

    // 全局共享状态（global state）：三块 reader/gate/tag_decoder 通过它协同
    struct READER_STATE
    {
        STATUS            status;            // 系统运行状态：RUNNING / TERMINATED（用于停止条件）
        GEN2_LOGIC_STATUS gen2_logic_status; // Reader 的 Gen2 逻辑状态机：下一步发什么（SEND_QUERY/SEND_ACK/...）
        GATE_STATUS       gate_status;       // Gate 门控状态：开门/关门/搜 RN16/搜 EPC（GATE_OPEN/...）
        DECODER_STATUS    decoder_status;    // Decoder 模式：解 RN16 还是解 EPC（DECODE_RN16/DECODE_EPC）

        READER_STATS      reader_stats;      // 统计信息（由 reader/decoder 更新）

        std::vector<float> magn_squared_samples; // Gate 在开门期间记录的 |x[n]|^2 序列（Decoder 用于同步/符号周期微调）
        int n_samples_to_ungate;                 // 本次需要放行的样点数
        
    };

    // 配置

    // Fixed number of slots (2^(FIXED_Q))  
    const int FIXED_Q       = 0;

    // const int MAX_INVENTORY_ROUND = 50;
    const int MAX_NUM_QUERIES     = 1000;     // Stop after MAX_NUM_QUERIES have been sent 

    // valid values for Q
    const int Q_VALUE [16][4] =  
    {
        {0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1}, 
        {0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1}, 
        {1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
        {1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1}
    };

    const bool P_DOWN = false;

    // Duration in us（单位：微秒 us）
    // reader ---> tag
    const int CW_D         = 250;    // Carrier wave
    const int P_DOWN_D     = 2000;    // power down
    const int T1_D         = 240;    // Time from Interrogator transmission to Tag response (250 us)
    const int T2_D         = 480;    // Time from Tag response to Interrogator transmission. Max value = 20.0 * T_tag = 500us 
    const int PW_D         = 12;      // Half Tari 
    const int DELIM_D       = 12;      // A preamble shall comprise a fixed-length start delimiter 12.5us +/-5%
    const int TRCAL_D     = 200;    // BLF = DR/TRCAL => 40e3 = 8/TRCAL => TRCAL = 200us
    const int RTCAL_D     = 72;      // 6*PW = 72us

    /*
     * GATE 参数
     */
    const int NUM_PULSES_COMMAND = 5;       // Number of pulses to detect a reader command
    const int NUMBER_UNIQUE_TAGS = 100;     // Stop after NUMBER_UNIQUE_TAGS have been read
    const float THRESH_FRACTION = 0.75;     
    const int WIN_SIZE_D         = 250;

    // 命令比特数
    const int PILOT_TONE          = 12;  // Optional
    const int TAG_PREAMBLE_BITS   = 6;   // Number of preamble bits
    const int RN16_BITS           = 17;  // Dummy bit at the end
    const int EPC_BITS            = 129;  // PC + EPC + CRC16 + Dummy = 6 + 16 + 96 + 16 + 1 = 135
    const int QUERY_LENGTH        = 22;  // Query length in bits
    
    // 下行链路长度估计
    // tag ---> reader
    const int T_READER_FREQ       = 40e3;     // BLF = 40kHz
    const float TAG_BIT_D         = 1.0/T_READER_FREQ * pow(10,6); // Duration in us
    const int RN16_D              = (RN16_BITS + TAG_PREAMBLE_BITS) * TAG_BIT_D;
    const int EPC_D               = (EPC_BITS  + TAG_PREAMBLE_BITS) * TAG_BIT_D;

    // 命令内容

    // Query command 
    const int QUERY_CODE[4] = {1,0,0,0};
    const int M[2]          = {0,0};
    const int SEL[2]         = {0,0};
    const int SESSION[2]     = {0,0};
    const int TARGET         = 0;
    const int TREXT         = 0;
    const int DR            = 0;

    // NAK command
    const int NAK_CODE[8]   = {1,1,0,0,0,0,0,0};

    // ACK command
    const int ACK_CODE[2]   = {0,1};

    // QueryAdjust command
    const int QADJ_CODE[4]   = {1,0,0,1};

    const int Q_UPDN[3][3]  = { {1,1,0}, {0,0,0}, {0,1,1} };

    // FM0 encoding preamble sequences
    const int TAG_PREAMBLE[] = {1,1,0,1,0,0,1,0,0,0,1,1};

    
    // Duration in which dc offset is estimated (T1_D is 250)
    const int DC_SIZE_D         = 120;

    // Global variable
    extern READER_STATE * reader_state;
    extern READER_API void initialize_reader_state();
} // namespace reader
} // namespace gr

#endif /* INCLUDED_READER_GLOBAL_VARS_H */

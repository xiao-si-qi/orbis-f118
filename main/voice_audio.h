/**
 * @file voice_audio.h
 * @brief 语音硬件通路：PDM 麦克风录音 + I2S(PCM5100A) 播放（小智语音的音频底层）。
 *
 * 板子（ESP32-S31 / JC3636F118）音频引脚：
 *   喇叭（PCM5100A，I2S 标准模式 TX only）：BCK=53, DIN=52, LRCK/WS=51
 *   麦克风（PDM 数字麦）：DATA=37, SCK=39
 *
 * 注意：本板 PCM5100A 没有 I2C 控制、也没有静音脚，
 *       voice_audio_mute() 保留为空实现（只为兼容上层调用）。
 */
#ifndef VOICE_AUDIO_H
#define VOICE_AUDIO_H

#include <stddef.h>
#include <stdint.h>

/** 初始化麦克风（PDM RX）+ 喇叭（I2S TX）（重复调用安全）。 */
int voice_audio_init(void);

/** 是否已初始化。 */
int voice_audio_ready(void);

/** 麦克风录音到缓冲，返回实际采样点数（16bit mono，约 ms 毫秒）。 */
int voice_audio_record(int16_t *buf, int max_samples, int sample_rate, int ms);

/** 播放一段 16bit mono PCM。 */
int voice_audio_play(const int16_t *buf, int samples, int sample_rate);

/** 静音/解除静音：本板没有静音脚，空实现（保留符号）。 */
void voice_audio_mute(int on);

/**
 * @brief 自检：录 1.5 秒 → 统计峰值/RMS → 回放。
 *
 * 用于验证麦克风和喇叭接线是否正常（日志/界面上看 peak/rms）。
 * @param peak 输出峰值（0-32767）
 * @param rms  输出均方根
 */
int voice_audio_selftest(int *peak, int *rms);

#endif /* VOICE_AUDIO_H */

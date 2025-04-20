#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// Board Specific Configuration
//---------------------------------------------------------------------
#define BOARD_TUD_RHPORT      0
#define BOARD_TUD_MAX_SPEED   TUSB_SPEED_HIGH

//--------------------------------------------------------------------+
// Common Configuration
//---------------------------------------------------------------------
#define CFG_TUSB_MCU          OPT_MCU_RP2040
#define CFG_TUSB_OS           OPT_OS_PICO
#define CFG_TUSB_DEBUG        0

//--------------------------------------------------------------------+
// Device Configuration
//---------------------------------------------------------------------
#define CFG_TUD_ENDPOINT0_SIZE    64

//--------------------------------------------------------------------+
// AUDIO Class Configuration
//---------------------------------------------------------------------
#define CFG_TUD_AUDIO             1
#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE          48000
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX        4
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX 3  // 24-bit
#define CFG_TUD_AUDIO_EP_SZ_IN                    TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN             TUD_AUDIO_MIC_FOUR_CH_DESC_LEN

#ifdef __cplusplus
}
#endif

#endif
/*
 * MEMS Microphone Array Beamforming System for RP2040
 * Copyright (c) Patrick Christian Bryant 2025
 * Version 0.01a10
 * 
 * Features:
 * - Quad-channel I2S microphone input
 * - Real-time beamforming processing
 * - USB Audio Class 2.0 interface
 * - Dynamic buffer management
 * - Hardware watchdog monitoring
 * - Triple buffering for zero-overlap processing
 */

/****************************************************
 *  todo: debugging and profiling over usb or uart  *
 *  method to change beamforming over serial        *
 *  make a build and deploy system                  *
 *  redo everything for the rp2350,misc chips etc   *
 *  and an analog front end version                 *
 ****************************************************/

/**********************************************
 *                  Includes                  *
 **********************************************/
 #include <math.h>
 #include <string.h>
 #include "pico/stdlib.h"
 #include "pico/multicore.h"
 #include "pico/float.h"
 #include "hardware/pio.h"
 #include "hardware/dma.h"
 #include "hardware/clocks.h"
 #include "hardware/watchdog.h"
 #include "hardware/structs/bus_ctrl.h"
 #include "hardware/irq.h"

// 1) Define TinyUSB audio config before including tusb.h:
#ifndef CFG_TUD_AUDIO
#define CFG_TUD_AUDIO                 1
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX  3   // 24‑bit
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX          1   // mono
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ           64
#endif

#include "tusb.h"
#include "class/audio/audio_device.h"  // use the device‑side audio macros

// 2) Compute total descriptor length at compile time:
#define UAC2_AC_DESC_LEN  ( \
    TUD_AUDIO_DESC_IAD_LEN        + \
    TUD_AUDIO_DESC_STD_AC_LEN     + \
    TUD_AUDIO_DESC_CS_AC_LEN      + \
    TUD_AUDIO_DESC_CLK_SRC_LEN    + \
    TUD_AUDIO_DESC_INPUT_TERM_LEN + \
    /* feature unit: master + 1 channel = 2 controls */ \
    TUD_AUDIO_DESC_FEATURE_UNIT_LEN(2) + \
    TUD_AUDIO_DESC_OUTPUT_TERM_LEN \
)

#define UAC2_AS_DESC_LEN  ( \
    /* two alt‑settings: zero BW + active */  \
    TUD_AUDIO_DESC_STD_AS_INT_LEN*2 + \
    TUD_AUDIO_DESC_CS_AS_INT_LEN    + \
    TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN + \
    TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN + \
    TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN \
)

#define TOTAL_DESC_LEN (TUD_CONFIG_DESC_LEN + UAC2_AC_DESC_LEN + UAC2_AS_DESC_LEN)

static const uint8_t desc_fs_configuration[] = {
    // — Standard config descriptor
    TUD_CONFIG_DESCRIPTOR(
        /*interfaces=*/2,            // audio control + audio streaming
        /*config*/      1,
        /*string_idx*/  0,
        /*total_len*/   TOTAL_DESC_LEN,
        /*attr*/        0x00,
        /*mA*/          100
    ),

    // — Audio Function IAD (use the UAC2 protocol code)
    TUD_AUDIO_DESC_IAD(
        /*first_if*/    0,
        /*count*/       1,
        /*proto*/       AUDIO_FUNC_PROTOCOL_CODE_V2
    ),

    // — Standard Audio Control interface
    TUD_AUDIO_DESC_STD_AC(
        /*if*/           0,
        /*string_idx*/   0,
        /*stream_if_count*/ 1
    ),

    // — Class‑specific AC header for UAC2 (bcdADC = 2.00)
    TUD_AUDIO_DESC_CS_AC(
        /*bcdADC*/       0x0200,
        /*category*/     AUDIO_FUNC_MICROPHONE,
        /*total_len*/    UAC2_AC_DESC_LEN - TUD_AUDIO_DESC_IAD_LEN - TUD_AUDIO_DESC_STD_AC_LEN,
        /*ctrl*/         0x01         // clock freq control
    ),

    // — Clock Source
    TUD_AUDIO_DESC_CLK_SRC(
        /*clkid*/        0x01,
        /*attr*/         AC_ATTR_INTERNAL_FIXED,
        /*ctrl*/         AC_CTRL_SAM_FREQ,
        /*assocTerm*/    0x00,
        /*stridx*/       0x00
    ),

    // — Input Term (USB streaming)
    TUD_AUDIO_DESC_INPUT_TERM(
        /*termid*/       0x02,
        /*termType*/     TERM_TYPE_USB_STREAMING,
        /*assocTerm*/    0x00,
        /*clkSrc*/       0x01,
        /*nChannels*/    1,
        /*chConfig*/     0x0001,
        /*stridx*/       0x00,
        /*ctrl*/         0x00,
        /*stridxTerm*/   0x00
    ),

    // — Feature Unit (master + 1 channel = 2 total channels)
    TUD_AUDIO_DESC_FEATURE_UNIT(
        /*unitid*/       0x03,
        /*src*/          0x02,
        /*ctrlSize*/     2,              // two control bytes: master & channel‑1
        /*controls*/     0x0001,         // mute control on master only
        /*stridx*/       0x00
    ),

    // — Output Term (to speaker/headset)
    TUD_AUDIO_DESC_OUTPUT_TERM(
        /*termid*/       0x04,
        /*termType*/     TERM_TYPE_OUT_HEADPHONES,
        /*assocTerm*/    0x00,
        /*src*/          0x03,
        /*clkSrc*/       0x01,
        /*ctrl*/         0x00,
        /*stridx*/       0x00
    ),

    // — Standard AS interface, alt‑0 (zero bandwidth)
    TUD_AUDIO_DESC_STD_AS_INT(
        /*if*/           1,
        /*alt*/          0,
        /*nEP*/          0,
        /*stridx*/       0
    ),

    // — Standard AS interface, alt‑1 (active streaming)
    TUD_AUDIO_DESC_STD_AS_INT(
        /*if*/           1,
        /*alt*/          1,
        /*nEP*/          1,
        /*stridx*/       0
    ),

    // — Class‑specific AS interface header
    TUD_AUDIO_DESC_CS_AS_INT(
        /*termLink*/     0x02,
        /*ctrl*/         0x00,
        /*format*/       AUDIO_FORMAT_TYPE_I,
        /*subslot*/      AUDIO_DATA_FORMAT_TYPE_I_PCM,
        /*nChannels*/    1,
        /*chConfig*/     0x0001,
        /*stridx*/       0x00
    ),

    // — Type I PCM format
    TUD_AUDIO_DESC_TYPE_I_FORMAT(
        /*subslotSize*/  3,           // bytes per sample
        /*bitResolution*/24
    ),

    // — Sampling frequency descriptor
    TUD_AUDIO_DESC_SAMPLE_RATE(
        SAMPLE_RATE, SAMPLE_RATE
    ),

    // — Standard isochronous endpoint, IN endpoint 0x81
    TUD_AUDIO_DESC_STD_AS_ISO_EP(
        /*ep_addr*/      0x81,
        /*attr*/         TUSB_XFER_ISOCHRONOUS,
        /*maxsize*/      (CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * SAMPLE_RATE / 1),
        /*interval*/     1
    ),

    // — Class‑specific endpoint descriptor
    TUD_AUDIO_DESC_CS_AS_ISO_EP(
        /*attr*/         AUDIO_CS_AS_ISO_DATA_EP_ATT_ADAPTIVE_SYNC,
        /*ctrl*/         AUDIO_CTRL_NONE,
        /*lockDelayUnit*/0,
        /*lockDelay*/    0
    ),
};
#include <stdlib.h>  // For abs()
#include <float.h>   // For FLT_MAX
 
 /**********************************************
  *            Hardware Configuration          *
  **********************************************/
 // I2S Physical Interface Pins (RP2040 GPIO numbers)
 #define I2S0_DATA_PIN    28  // GPIO28: Left mic array data (PIO0 SM0)
 #define I2S1_DATA_PIN    29  // GPIO29: Right mic array data (PIO0 SM1)
 #define I2S_SCK_PIN      26  // GPIO26: Bit clock (BCLK)
 #define I2S_WS_PIN       27  // GPIO27: Word select (LRCLK)
 
 // System Configuration Constants
 #define SYS_CLOCK_MHZ       250     // RP2040 overclock frequency
 #define SAMPLE_RATE         48000   // Audio sample rate (Hz)
 #define BIT_DEPTH           24      // Bits per sample
 #define NUM_CHANNELS        4       // Number of microphone channels
 #define MAX_BUFFER_FRAMES   256     // Maximum audio buffer size
 #define MIN_BUFFER_FRAMES   128     // Minimum audio buffer size
 #define WATCHDOG_TIMEOUT_MS 200     // Hardware watchdog timeout
 #define AUDIO_PROTOCOL_IP_VERSION_02_00 0x20 // UsbAudio 2.0
 
 /***********************************************
  *              Global State                  *
  **********************************************/
 // Dynamic buffer configuration
 volatile uint32_t g_buffer_frames = 192;  // Initial buffer size (~4ms @48kHz)
 
 // Triple Buffer System States
 typedef enum {
     BUF_STATE_EMPTY = 0,
     BUF_STATE_FILLING,
     BUF_STATE_READY,
     BUF_STATE_PROCESSING
 } buffer_state_t;
 
 typedef struct {
     int32_t samples[NUM_CHANNELS][MAX_BUFFER_FRAMES];
     uint32_t timestamp;          // Capture time (µs since boot)
     volatile buffer_state_t state;
     uint16_t sequence;           // Incremental buffer counter
     uint32_t actual_frames;      // Valid frames in buffer
 } audio_buffer_t;
 
 audio_buffer_t buffers[3];       // Triple buffer instance
 volatile uint8_t active_dma_buf = 0;    // Currently filling buffer
 volatile uint8_t active_proc_buf = 1;   // Being processed buffer
 volatile uint8_t active_usb_buf = 2;    // Ready for USB transfer buffer
 
 // DMA Control
 int dma_chans[2] = {-1, -1};     // DMA channel handles
 volatile bool dma_complete[2] = {false, false};
 uint8_t current_alt_setting = 0;  // Current audio interface alternate setting
 
 /**********************************************
  *         Audio Processing State             *
  **********************************************/
 typedef struct {
     int32_t dc_offset;    // Q16.16 fixed-point DC offset
     int32_t gain;         // Q8.24 fixed-point gain
     uint32_t clip_count;  // Cumulative clip events
     uint32_t error_count; // Processing error counter
 } channel_state_t;
 
 channel_state_t channel_states[NUM_CHANNELS];
 
 // Beamforming Configuration
 typedef struct {
     float mic_positions[NUM_CHANNELS][3]; // Mic positions in meters (XYZ)
     float beam_direction[3];              // Normalized direction vector
     uint8_t delay_samples[NUM_CHANNELS];  // Computed delays in samples
     bool beamforming_enabled;
     int32_t beamformed_buffer[MAX_BUFFER_FRAMES]; // Output buffer
 } beamforming_config_t;
 
 beamforming_config_t beam_config = {
     .mic_positions = {
         {-0.02, 0.02, 0},  // Top-left mic (X,Y,Z)
         {0.02, 0.02, 0},   // Top-right mic
         {-0.02, -0.02, 0}, // Bottom-left
         {0.02, -0.02, 0}   // Bottom-right
     },
     .beam_direction = {0, 1, 0}, // Default forward direction
     .beamforming_enabled = false
 };
 
 /**********************************************
  *              USB Configuration             *
  **********************************************/
 #define CFG_TUD_AUDIO 1
 #define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX 3  // 24-bit audio = 3 bytes

 // String descriptors
static const char* const string_desc_arr[] = {
    "PC Labs",              // 0: Manufacturer
    "Beamforming Mic Array", // 1: Product
    "123456",               // 2: Serial number
    "Audio Interface"       // 3: Interface
};

 volatile bool usb_was_connected = false; // Connection state tracker
 static int32_t clip_lut[256];            // Soft clipping lookup table
 
 /**********************************************
  *           Forward Declarations             *
  **********************************************/
 void init_clip_lut(void);
 void process_audio_buffer(void);
 void handle_usb_status(void);
 void core1_entry(void);
 void reconfigure_dma(uint8_t buf_idx);
 void init_dma(void);
 void handle_buffer_rotation(uint32_t* last_usb, uint32_t* last_clock_check, uint32_t* last_usb_activity);
 static inline int32_t constrain(int32_t value, int32_t min, int32_t max);
 void i2s_program_init(PIO pio, uint sm, uint offset, uint data_pin, uint clock_pin_base, float clock_div, bool is_swapped);
 static inline int32_t saturate(int64_t value, int bits) {
     int32_t max = (1 << (bits - 1)) - 1;
     int32_t min = -(1 << (bits - 1));
     return (value > max) ? max : (value < min) ? min : (int32_t)value;
 }
 
 // Forward declarations for TinyUSB Audio callbacks
 bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting);
 bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting);
 
 /**********************************************
  *              PIO Assembly Code             *
  **********************************************/
 /* I2S PIO Program Explanation:
  * - Manages I2S timing and data shifting
  * - Two state machines handle left/right channels
  * - 24-bit data packed into 32-bit words
  * - Clock edges synchronized with data sampling
  */
 const uint16_t i2s_pio_program_instructions[] = {
     // Standard I2S program (non-swapped channels)
     0x6021, // [0] out pins, 1      side 0b10
     0x1043, // [1] jmp x--, 1       side 0b11
     0x6001, // [2] out pins, 1      side 0b00
     0xe82e, // [3] set x, 14        side 0b01
     0x6001, // [4] out pins, 1      side 0b00
     0x1083, // [5] jmp x--, 4       side 0b01
     0x6021, // [6] out pins, 1      side 0b10
     0xe82f, // [7] set x, 14        side 0b11
 
     // Swapped channel program (for right channel)
     0x6011, // [8] out pins, 1      side 0b01
     0x1043, // [9] jmp x--, 9       side 0b11 
     0x6001, // [10] out pins, 1     side 0b00 
     0xe82a, // [11] set x, 14       side 0b10 
     0x6001, // [12] out pins, 1     side 0b00 
     0x1085, // [13] jmp x--, 12     side 0b10 
     0x6011, // [14] out pins, 1     side 0b01 
     0xe82f, // [15] set x, 14       side 0b11 
 };
 
 /**********************************************
  *              Core Functions                *
  **********************************************/
 
 // System Initialization
 void system_init() {
     stdio_init_all();  // Initialize standard I/O for debugging
     
     // Hardware Configuration
     watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
     set_sys_clock_khz(SYS_CLOCK_MHZ * 1000, true);
     
     // Bus Priority Configuration
     bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | 
                           BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
     
     // Initialize Processing State
     init_clip_lut();
     for(int i=0; i<NUM_CHANNELS; i++) {
         channel_states[i] = (channel_state_t){
             .gain = 0x01000000,  // Q8.24 unity gain
             .dc_offset = 0,
             .clip_count = 0,
             .error_count = 0
         };
     }
     
     // Start Secondary Core
     multicore_launch_core1(core1_entry);
     
     // Initialize Hardware Peripherals
     init_dma();
     tusb_init();
 }
 
 // Main Processing Loop (Core 0)
 int main() {
     system_init();
     uint32_t last_usb = 0, last_clock_check = 0, last_usb_activity = 0;
     
     while(1) {
         watchdog_update();
         tud_task();
         handle_usb_status();
         
         // Buffer Management
         if(buffers[active_usb_buf].state == BUF_STATE_READY) {
             handle_buffer_rotation(&last_usb, &last_clock_check, &last_usb_activity);
         }
         
         sleep_us(100); // Reduce CPU utilization
     }
 }
 
 /**********************************************
  *          Secondary Core (Core 1)           *
  **********************************************/
 void core1_entry() {
     bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;
     
     while(1) {
         watchdog_update();
         
         if(buffers[active_proc_buf].state == BUF_STATE_READY) {
             process_audio_buffer();
         }
         sleep_us(100);
     }
 }
 
 /**************************************************************************
  * TinyUSB Device Descriptors - modeled after the example
  **************************************************************************/
 #define _PID_MAP(itf, n) ((CFG_TUD_##itf) << (n))
 
 // USB PID
 #define USB_PID (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                  _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) | _PID_MAP(AUDIO, 5))
 
// USB device descriptor
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Audio class for Audio v2.0
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0xCAFE,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};
 
 #define TUD_AUDIO_DESC_LEN_TOTAL (TUD_AUDIO_DESC_IAD_LEN + TUD_AUDIO_DESC_STD_AC_LEN(1) + \
         TUD_AUDIO_DESC_CS_AC_LEN + TUD_AUDIO_DESC_INPUT_TERM_LEN + TUD_AUDIO_DESC_FEATURE_UNIT_LEN(4) + \
         TUD_AUDIO_DESC_OUTPUT_TERM_LEN + TUD_AUDIO_DESC_STD_AS_INT_LEN + TUD_AUDIO_DESC_CS_AS_INT_LEN + \
         TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN)


// USB Audio Class 2.0 Descriptors
static const uint8_t desc_fs_configuration[] = {
    // Configuration Descriptor (9 bytes)
    TUD_CONFIG_DESCRIPTOR(
        2,    // bNumInterfaces (1 Control + 1 Streaming)
        1,    // Configuration number
        0,    // String index
        TUD_CONFIG_DESC_LEN + CFG_TUD_AUDIO_FUNC_1_DESC_LEN,
        0x00, // bmAttributes
        100   // mA current
    ),

    // IAD for Audio Function
    TUD_AUDIO_DESC_IAD(0, 2, AUDIO_FUNCTION_PROTOCOL_CODE_UNDEF),

    // Audio Control Interface
    TUD_AUDIO_DESC_STD_AC(
        0,    // Interface number
        0,    // String index
        1     // Number of streaming interfaces
    ),

    // Class-Specific AC Interface Header (9 bytes)
    TUD_AUDIO_DESC_CS_AC(
        0x0200,                // bcdADC (Audio 2.0)
        AUDIO_FUNC_MICROPHONE, // bCategory
        16,                    // wTotalLength (Approximate)
        0x01                   // bmControls (Clock Frequency Control)
    ),

    // Clock Source Descriptor (required for UAC 2.0)
    TUD_AUDIO_DESC_CLK_SRC(
        0x01,           // bClockID
        0x03,           // bmAttributes (internal fixed clock)
        0x01,           // bmControls (Clock Frequency Control)
        0x00,           // bAssocTerminal
        0x00            // iClockSource
    ),

    // Input Terminal (12 bytes)
    TUD_AUDIO_DESC_INPUT_TERM(
        0x02,           // bTerminalID
        AUDIO_TERM_TYPE_USB_STREAMING,
        0x00,           // bAssocTerm
        0x01,           // bCSourceID (clock source ID)
        0x04,           // bNrChannels
        0x0000003F,     // wChannelConfig
        0x00,           // iChannelNames
        0x0000,         // bmControls
        0x00            // iTerminal (string index)
    ),

    // Feature Unit (13 bytes per channel)
    TUD_AUDIO_DESC_FEATURE_UNIT(
        0x03,           // bUnitID
        0x02,           // bSourceID
        4,              // Number of channels (including master)
        0x01, 0x00, 0x00, 0x00, 0x00, // Master + 4 channel controls
        0x00            // iFeature (string index)
    ),

    // Output Terminal (9 bytes)
    TUD_AUDIO_DESC_OUTPUT_TERM(
        0x04,           // bTerminalID
        AUDIO_TERM_TYPE_OUT_HEADPHONES,
        0x00,           // bAssocTerm
        0x03,           // bSourceID
        0x01,           // bCSourceID (clock source)
        0x0000,         // bmControls
        0x00            // iTerminal (string index)
    ),

    // Audio Streaming Interface (Alt 0 - inactive)
    TUD_AUDIO_DESC_STD_AS_INT(
        1,              // bInterfaceNumber
        1,              // bAlternateSetting
        1,              // bNumEndpoints
        0               // iInterface (string index)
    ),

    // Audio Streaming Interface (Alt 1 - active)
    TUD_AUDIO_DESC_STD_AS_INT(
        1,              // bInterfaceNumber
        1,              // bAlternateSetting
        1,              // bNumEndpoints
        0               // iInterface (string index)
    ),

    // CS Interface AS General
    TUD_AUDIO_DESC_CS_AS_INT(
        0x02,           // bTerminalLink
        0x00,           // bmControls
        AUDIO_FORMAT_TYPE_I,
        AUDIO_DATA_FORMAT_TYPE_I_PCM,
        1,              // bNrChannels
        0x0000,         // wChannelConfig
        0               // iChannelNames (string index)
    ),
    

    // Type I Format Type Descriptor
    TUD_AUDIO_DESC_TYPE_I_FORMAT(
        CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX,  // bSubslotSize
        BIT_DEPTH      // bBitResolution
    ),
    
    // Add separate sampling frequency descriptor
    TUD_AUDIO_DESC_SAMPLE_RATE(SAMPLE_RATE, SAMPLE_RATE),

    // Standard AS Isochronous Audio Data Endpoint Descriptor
    TUD_AUDIO_DESC_STD_AS_ISO_EP(
        0x81,           // bEndpointAddress
        TUSB_XFER_ISOCHRONOUS,
        192,            // wMaxPacketSize
        1               // bInterval
    ),

    // Class-Specific AS Isochronous Audio Data Endpoint Descriptor
    TUD_AUDIO_DESC_CS_AS_ISO_EP(
        AUDIO_CS_AS_ISO_DATA_EP_ATT_ADAPTIVE_SYNC,  // bmAttributes
        AUDIO_CTRL_NONE,       // bmControls
        0,                     // bLockDelayUnit
        0                      // wLockDelay
    ),
};

 uint8_t const * tud_descriptor_device_cb(void)
 {
     return (uint8_t const *) &desc_device;
 }
 
 uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
 {
     (void) index;
     return desc_fs_configuration;
 }
 
 uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
 {
     static uint16_t desc_str[64];
     uint8_t len;
 
     // Convert ASCII string into UTF-16
     if (index >= sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) return NULL;
     
     const char* str = string_desc_arr[index];
     
     // First byte is length (including header), second byte is string type
     len = 1;
     for(uint8_t i = 0; i < strlen(str); i++) {
         desc_str[1+i] = str[i];
         len++;
     }
 
     desc_str[0] = (TUSB_DESC_STRING << 8) | (2*len);
 
     return desc_str;
 }
 
 /***********************************************
  *          System Utilities                   *
  ***********************************************/
 __attribute__((noreturn)) void system_reboot() {
     watchdog_reboot(0, 0, WATCHDOG_TIMEOUT_MS);
     while(1); // Safety loop until reset
 }
 
 /**
  * @brief Optimized DC offset removal filter
  */
 inline int32_t fast_dc_block(int32_t sample, int32_t *dc_offset) {
     *dc_offset = (*dc_offset * 63) >> 6;  // 0.984375 coefficient
     *dc_offset += (sample >> 6);          // 64-sample moving average
     return sample - *dc_offset;
 }
 
 /**********************************************
  *          Beamforming Mathematics           *
  **********************************************/
 
 /**
  * @brief Calculate microphone delay coefficients
  */
 void update_beamforming_delays() {
     if(!beam_config.beamforming_enabled) return;
     
     const float c = 343.0f; // Speed of sound (m/s)
     int ref_mic = 0;
     float max_projection = -FLT_MAX;
 
     // Find reference microphone
     for(int i=0; i<NUM_CHANNELS; i++) {
         float projection = beam_config.mic_positions[i][0] * beam_config.beam_direction[0] +
                          beam_config.mic_positions[i][1] * beam_config.beam_direction[1] +
                          beam_config.mic_positions[i][2] * beam_config.beam_direction[2];
         
         if(projection > max_projection) {
             max_projection = projection;
             ref_mic = i;
         }
     }
 
     // Calculate relative delays
     for(int i=0; i<NUM_CHANNELS; i++) {
         float dx = beam_config.mic_positions[i][0] - beam_config.mic_positions[ref_mic][0];
         float dy = beam_config.mic_positions[i][1] - beam_config.mic_positions[ref_mic][1];
         float dz = beam_config.mic_positions[i][2] - beam_config.mic_positions[ref_mic][2];
 
         float projection = dx * beam_config.beam_direction[0] +
                          dy * beam_config.beam_direction[1] +
                          dz * beam_config.beam_direction[2];
 
         float delay_time = projection / c;
         int samples = (int)(delay_time * SAMPLE_RATE);
         beam_config.delay_samples[i] = (uint8_t)constrain(samples, 0, 255);
     }
 }
 
 /**********************************************
  *          Audio Processing Utilities        *
  **********************************************/
 
 /**
  * @brief Initialize soft clipping lookup table
  */
 void init_clip_lut() {
     for(int i=0; i<256; i++) {
         float x = (i - 128) / 128.0f;  // Normalized to [-1, +1]
         float y = fabs(x) > 1.0f ? copysignf(1.0f, x) : (1.5f*x) - (0.5f*x*x*x);
         clip_lut[i] = (int32_t)(y * 8388608.0f); // Q23.0 conversion
     }
 }
 
 /**
  * @brief Process audio buffer with DC removal, gain, and clipping
  */
 void process_audio_buffer() {
     uint32_t start = time_us_32();
     uint32_t frames = buffers[active_proc_buf].actual_frames;
     
     buffers[active_proc_buf].state = BUF_STATE_PROCESSING;
     
     for(int ch=0; ch<NUM_CHANNELS; ch++) {
         for(int i=0; i<frames; i++) {
             int32_t sample = buffers[active_proc_buf].samples[ch][i];
             
             // DC offset removal
             sample = fast_dc_block(sample, &channel_states[ch].dc_offset);
             
             // Apply gain with saturation
             int64_t amplified = (int64_t)sample * channel_states[ch].gain;
             sample = saturate(amplified >> 24, 24);
             
             // Soft clipping
             uint8_t lut_index = ((sample >> 16) + 128) & 0xFF;
             buffers[active_proc_buf].samples[ch][i] = clip_lut[lut_index];
         }
     }
 
     // Beamforming if enabled
     if(beam_config.beamforming_enabled) {
         memset(beam_config.beamformed_buffer, 0, frames * sizeof(int32_t));
         
         for(int ch=0; ch<NUM_CHANNELS; ch++) {
             uint8_t delay = beam_config.delay_samples[ch];
             for(int i=0; i<frames - delay; i++) {
                 beam_config.beamformed_buffer[i + delay] += 
                     buffers[active_proc_buf].samples[ch][i] / NUM_CHANNELS;
             }
         }
     }
 
     // Timing validation
     uint32_t duration = time_us_32() - start;
     uint32_t max_allowed = (frames * 1000000 / SAMPLE_RATE) * 90 / 100;
     
     if(duration > max_allowed) {
         channel_states[0].error_count++;
         if(channel_states[0].error_count > 10) system_reboot();
     } else if(channel_states[0].error_count > 0) {
         channel_states[0].error_count--;
     }
 
     buffers[active_proc_buf].state = BUF_STATE_READY;
 }
 
 /**********************************************
  *          DMA & Interrupt Handlers          *
  **********************************************/
 
 static void dma_handler() {
     for(int i=0; i<2; i++) {
         if(dma_channel_get_irq0_status(dma_chans[i])) {
             dma_channel_acknowledge_irq0(dma_chans[i]);
             dma_complete[i] = true;
 
             if(dma_complete[0] && dma_complete[1]) {
                 dma_complete[0] = dma_complete[1] = false;
                 buffers[active_dma_buf].timestamp = time_us_32();
                 buffers[active_dma_buf].actual_frames = g_buffer_frames;
                 buffers[active_dma_buf].state = BUF_STATE_READY;
 
                 uint8_t next_buf = (active_dma_buf + 1) % 3;
                 if(buffers[next_buf].state == BUF_STATE_EMPTY) {
                     buffers[next_buf].state = BUF_STATE_FILLING;
                     buffers[next_buf].sequence++;
                     active_dma_buf = next_buf;
                     reconfigure_dma(active_dma_buf);
                 } else {
                     channel_states[0].error_count++;
                     reconfigure_dma(active_dma_buf);
                 }
             }
         }
     }
 }
 
 void reconfigure_dma(uint8_t buf_idx) {
     for(int i=0; i<2; i++) {
         dma_channel_config c = dma_channel_get_default_config(dma_chans[i]);
         channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
         channel_config_set_read_increment(&c, false);
         channel_config_set_write_increment(&c, true);
         channel_config_set_dreq(&c, pio_get_dreq(pio0, i, false));
         channel_config_set_irq_quiet(&c, false);
         channel_config_set_chain_to(&c, dma_chans[(i+1)%2]);
         channel_config_set_high_priority(&c, true);
 
         dma_channel_configure(dma_chans[i], &c,
             &buffers[buf_idx].samples[i],
             &pio0->rxf[i],
             g_buffer_frames,
             true
         );
     }
 }
 
 /**********************************************
  *          PIO Initialization                *
  **********************************************/
 
 void i2s_program_init(PIO pio, uint sm, uint offset, uint data_pin, 
                      uint clock_pin_base, float clock_div, bool is_swapped) {
     pio_sm_config sm_config = pio_get_default_sm_config();
     
     // GPIO configuration
     pio_gpio_init(pio, data_pin);
     pio_gpio_init(pio, clock_pin_base);
     pio_gpio_init(pio, clock_pin_base+1);
     pio_sm_set_consecutive_pindirs(pio, sm, data_pin, 1, true);
     pio_sm_set_consecutive_pindirs(pio, sm, clock_pin_base, 2, true);
 
     // PIO program configuration
     sm_config_set_out_pins(&sm_config, data_pin, 1);
     sm_config_set_sideset_pins(&sm_config, clock_pin_base);
     sm_config_set_out_shift(&sm_config, false, true, 32);
     sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
     sm_config_set_clkdiv(&sm_config, clock_div);
 
     // Initialize state machine
     uint entry_point = is_swapped ? offset + 8 : offset;
     pio_sm_init(pio, sm, entry_point, &sm_config);
     pio_sm_set_pins(pio, sm, 0);
     pio_sm_exec(pio, sm, pio_encode_jmp(offset + 
               (is_swapped ? 15 : 7))); // Entry points
     pio_sm_set_enabled(pio, sm, true);
 }
 
 /**********************************************
  *          USB Management                    *
  **********************************************/
 
 void handle_usb_status() {
     bool is_connected = tud_mounted();
     
     if(is_connected && !usb_was_connected) {
         // Reset all buffers on new connection
         for(int i=0; i<3; i++) {
             buffers[i].state = BUF_STATE_EMPTY;
             buffers[i].sequence = 0;
             buffers[i].timestamp = 0;
         }
         active_dma_buf = 0;
         active_proc_buf = 1;
         active_usb_buf = 2;
         
         g_buffer_frames = 192;
         buffers[active_dma_buf].state = BUF_STATE_FILLING;
         reconfigure_dma(active_dma_buf);
     }
     usb_was_connected = is_connected;
 }
 
 /*****************************************************
  *          TinyUSB Callbacks for Audio Class        *
  *****************************************************/
 bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * request)
 {
     (void) rhport;
     uint8_t const itf = tu_u16_low(request->wIndex);
     uint8_t const alt = tu_u16_low(request->wValue);
 
     // Only support alt 1 (active streaming) or 0 (no streaming)
     if (alt > 1) return false;
     
     current_alt_setting = alt;
     return true;
 }
 
 bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting) 
 {
     (void) rhport;
     (void) itf;
     (void) ep_in;
     (void) cur_alt_setting;
     
     if (cur_alt_setting == 0) return false;  // Streaming is disabled
     if (buffers[active_usb_buf].state != BUF_STATE_READY) return false;
     
     // Use the active buffer as our source
     tud_audio_write((uint8_t *)buffers[active_usb_buf].samples, 
                     buffers[active_usb_buf].actual_frames * NUM_CHANNELS * (BIT_DEPTH/8));
     
     return true;
 }
 
 bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
 {
     (void) rhport;
     (void) itf;
     (void) ep_in;
     (void) cur_alt_setting;
     
     if (n_bytes_copied > 0) {
         buffers[active_usb_buf].state = BUF_STATE_EMPTY;
     }
     
     return true;
 }
 
 /**********************************************
  *          Buffer Management                 *
  **********************************************/
 
 static inline int32_t constrain(int32_t value, int32_t min, int32_t max) {
     return (value < min) ? min : (value > max) ? max : value;
 }
 
 void handle_buffer_rotation(uint32_t* last_usb, 
                                   uint32_t* last_clock_check,
                                   uint32_t* last_usb_activity) {
     uint32_t buffer_time_us = (g_buffer_frames * 1000000) / SAMPLE_RATE;
     
     if(time_us_32() - *last_usb > (buffer_time_us - 500)) {
         // Clock drift adjustment
         if(time_us_32() - *last_clock_check > 1000000) {
             uint32_t expected = buffers[active_usb_buf].sequence * g_buffer_frames;
             uint32_t actual = buffers[active_usb_buf].timestamp * SAMPLE_RATE / 1000000;
             int32_t drift = actual - expected;
             
             if(abs(drift) > 10) {
                 g_buffer_frames = constrain(g_buffer_frames + ((drift > 0) ? 1 : -1), 
                                           MIN_BUFFER_FRAMES, MAX_BUFFER_FRAMES);
             }
             *last_clock_check = time_us_32();
         }
 
         // Check if buffer is ready for transfer
         if(tud_mounted() && current_alt_setting > 0) {
             // The actual transfer happens in the tud_audio_tx_done_pre_load_cb callback
             // Let the USB stack know we have data ready
             tud_audio_write_flush();
             
             
             *last_usb = *last_usb_activity = time_us_32();
             buffers[active_usb_buf].state = BUF_STATE_EMPTY;
             
             // Rotate buffers
             uint8_t prev = active_usb_buf;
             active_usb_buf = active_proc_buf;
             active_proc_buf = active_dma_buf;
             active_dma_buf = prev;
         }
     }
 }
 
 /**********************************************
  *          DMA Initialization                *
  **********************************************/
 
 void init_dma() {
     // Initialize buffer states
     for(int i=0; i<3; i++) {
         buffers[i] = (audio_buffer_t){
             .state = BUF_STATE_EMPTY,
             .sequence = 0,
             .timestamp = 0,
             .actual_frames = g_buffer_frames
         };
     }
 
     // Calculate PIO clock divider
     float pio_divider = (float)clock_get_hz(clk_sys) / (2 * SAMPLE_RATE * BIT_DEPTH * 2);
     
     // Load PIO program
     PIO pio = pio0;
     pio_program_t i2s_program = {
                 .instructions = i2s_pio_program_instructions,
                 .length = sizeof(i2s_pio_program_instructions) / sizeof(uint16_t),
                 .origin = -1  // Allocate automatically
             };
         
             uint offset = pio_add_program(pio, &i2s_program);
         
     
     // Initialize both I2S channels
     i2s_program_init(pio, 0, offset, I2S0_DATA_PIN, I2S_SCK_PIN, pio_divider, false);
     i2s_program_init(pio, 1, offset, I2S1_DATA_PIN, I2S_SCK_PIN, pio_divider, true);
 
     // Claim DMA channels
     dma_chans[0] = dma_claim_unused_channel(true);
     dma_chans[1] = dma_claim_unused_channel(true);
 
     // Configure DMA interrupts
     irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
     irq_set_enabled(DMA_IRQ_0, true);
     for(int i=0; i<2; i++) {
         dma_channel_set_irq0_enabled(dma_chans[i], true);
     }
 
     // Start first transfer
     buffers[active_dma_buf].state = BUF_STATE_FILLING;
     reconfigure_dma(active_dma_buf);
 }
 

#include "tusb.h"
#include "bsp/board_api.h"

//--------------------------------------------------------------------+
// Device Descriptors
//---------------------------------------------------------------------
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0xCafe,
    .idProduct          = 0x4004,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

//--------------------------------------------------------------------+
// Configuration Descriptor
//---------------------------------------------------------------------
uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUD_CONFIG_DESC_LEN + TUD_AUDIO_MIC_FOUR_CH_DESC_LEN, 0x00, 100),
    TUD_AUDIO_MIC_FOUR_CH_DESCRIPTOR(
        /* Control Interface */ 0, 
        /* String Index */ 0,
        /* Bytes per Sample */ CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX,
        /* Bits per Sample */ 24,
        /* EP In Address */ 0x81,
        /* EP Size */ CFG_TUD_AUDIO_EP_SZ_IN
    )
};

//--------------------------------------------------------------------+
// String Descriptors
//---------------------------------------------------------------------
char const* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // 0: English
    "Your Company",                // 1: Manufacturer
    "Beamforming Mic Array",       // 2: Product
    NULL,                          // 3: Serial
};

static uint16_t _desc_str[32];

uint8_t const* tud_descriptor_device_cb(void) { return (uint8_t const*)&desc_device; }
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) { return index == 0 ? desc_configuration : NULL; }

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    if (index == 0) {
        _desc_str[0] = 0x0409;
        return _desc_str;
    }
    
    const char* str = index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0]) ? string_desc_arr[index] : NULL;
    if (!str) return NULL;

    uint8_t len = strlen(str);
    if (len > 31) len = 31;
    
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2*len + 2);
    for(uint8_t i=0; i<len; i++) _desc_str[i+1] = str[i];
    
    return _desc_str;
}
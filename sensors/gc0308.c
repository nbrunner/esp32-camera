
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sccb.h"
#include "gc0308.h"
#include "gc0308_regs.h"
#include "gc0308_settings.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "gc0308";
#endif

typedef enum {
    VGA =   0,      // 640 x 480
    CIF =   1,      // 400 x 296
    QVGA =  2,      // 320 x 240
} resolution_t;

typedef enum {
    YUV = 0,
    RGB565,
    ONLY_Y,
} output_format_t;

typedef enum {
    YUYV = 0,
    UYVY,
    VYUY,
    YVYU
} yuv_order_t;


//#define REG_DEBUG_ON

static int read_reg(uint8_t slv_addr, const uint16_t reg){
    int ret = SCCB_Read(slv_addr, reg);
#ifdef REG_DEBUG_ON
    if (ret < 0) {
        ESP_LOGE(TAG, "READ REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int write_reg(uint8_t slv_addr, const uint16_t reg, uint8_t value){
    int ret = 0;
#ifndef REG_DEBUG_ON
    ret = SCCB_Write(slv_addr, reg, value);
#else
    int old_value = read_reg(slv_addr, reg);
    if (old_value < 0) {
        return old_value;
    }
    if ((uint8_t)old_value != value) {
        ESP_LOGI(TAG, "NEW REG 0x%04x: 0x%02x to 0x%02x", reg, (uint8_t)old_value, value);
        ret = SCCB_Write(slv_addr, reg, value);
    } else {
        ESP_LOGD(TAG, "OLD REG 0x%04x: 0x%02x", reg, (uint8_t)old_value);
        ret = SCCB_Write(slv_addr, reg, value);//maybe not?
    }
    if (ret < 0) {
        ESP_LOGE(TAG, "WRITE REG 0x%04x FAILED: %d", reg, ret);
    }
#endif
    return ret;
}

static int check_reg_mask(uint8_t slv_addr, uint16_t reg, uint8_t mask){
    return (read_reg(slv_addr, reg) & mask) == mask;
}

static int set_reg_bits(uint8_t slv_addr, uint16_t reg, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;
    ret = read_reg(slv_addr, reg);
    if(ret < 0) {
        return ret;
    }
    c_value = ret;
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = write_reg(slv_addr, reg, new_value);
    return ret;
}

static int write_regs(uint8_t slv_addr, const uint16_t (*regs)[2])
{
    int i = 0, ret = 0;
    while (!ret && regs[i][0] != REGLIST_TAIL) {
        if (regs[i][0] == REG_DLY) {
            vTaskDelay(regs[i][1] / portTICK_PERIOD_MS);
        } else {
            ret = write_reg(slv_addr, regs[i][0], regs[i][1]);
        }
        i++;
    }
    return ret;
}


static void set_YUV_order(sensor_t *sensor, yuv_order_t yuv_order)
{
    SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
    SCCB_Write(sensor->slv_addr, GC0308_YUV_ORDER[yuv_order][0], GC0308_YUV_ORDER[yuv_order][1]);
}

static void set_pixformat(sensor_t *sensor, output_format_t output_format)
{
    SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
    switch (output_format)
    {
        case YUV:
        {
            ESP_LOGI(TAG, "Perform YUV Init");
            /*Adjust YUV output order*/
            SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
            break;
        }
        case RGB565:
        {
            ESP_LOGI(TAG, "Perform RGB565 Init");
            SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
            SCCB_Write(sensor->slv_addr, 0x24, 0xa6);
        } break;
        case ONLY_Y:
        {
            ESP_LOGI(TAG, "Perform GRAY Init");
            SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
            SCCB_Write(sensor->slv_addr, 0x24, 0xb1); // 0xb1
        }
    }
}

static void set_vertical_flip(sensor_t *sensor)
{
    SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
    uint8_t value = SCCB_Read(sensor->slv_addr, 0x14);
    if (value == 0x12) SCCB_Write(sensor->slv_addr, 0x14, 0x10);
    else SCCB_Write(sensor->slv_addr, 0x14, 0x12);
}

static void set_CbCr_saturation_enhance(sensor_t *sensor, uint8_t saturation)
{
    if (saturation != 0)
    {
        SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
        SCCB_Write(sensor->slv_addr, 0xb1, saturation);
        SCCB_Write(sensor->slv_addr, 0xb2, saturation);
    }
}

static void set_edge_saturation_enhance(sensor_t *sensor, uint8_t edge_saturation)
{
    if (edge_saturation != 0)
    {
        SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
        SCCB_Write(sensor->slv_addr, 0xbd, edge_saturation);  // default is 0x38/56
    }
}

static void set_contrast(sensor_t *sensor, uint8_t contrast)
{
    if (contrast != 0)
    {
        SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
        SCCB_Write(sensor->slv_addr, 0xb3, contrast);
    }
}

static void set_global_gain(sensor_t *sensor, uint8_t gain_level)
{
    if (gain_level != 0)
    {
        SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
        SCCB_Write(sensor->slv_addr, 0x50, gain_level);
    }
}

static void set_AEC(sensor_t *sensor, bool aec, uint8_t *aec_value)
{
    for (int i = 0; i < sizeof(GC0308_HB_VB_STEPS) / 2; i++)
    {
        SCCB_Write(sensor->slv_addr, GC0308_HB_VB_STEPS[i][0], GC0308_HB_VB_STEPS[i][1]);
    }
    if (aec == true)  // Auto exposure control
    {
        for (int i = 0; i < sizeof(aec_value); i++)
        {
            uint8_t index = aec_value[i] + 1;
            SCCB_Write(sensor->slv_addr, GC0308_AEC_EXP_LEVEL[i][0], GC0308_AEC_EXP_LEVEL[index][0]);
            SCCB_Write(sensor->slv_addr, GC0308_AEC_EXP_LEVEL[i][1], GC0308_AEC_EXP_LEVEL[index][1]);
            SCCB_Write(sensor->slv_addr, 0xec, 0x00 + i * 16);
        };
    }
    else  // Fixed exposure
    {
        uint8_t index = *aec_value + 1;
        SCCB_Write(sensor->slv_addr, GC0308_AEC_EXP_LEVEL[0][0], GC0308_AEC_EXP_LEVEL[index][0]);
        SCCB_Write(sensor->slv_addr, GC0308_AEC_EXP_LEVEL[0][1], GC0308_AEC_EXP_LEVEL[index][1]);
        SCCB_Write(sensor->slv_addr, 0xec, 0x00);
    }
}

static void set_AEC_target_Y(sensor_t *sensor, uint8_t AEC_target_Y)
{
    SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
    SCCB_Write(sensor->slv_addr, 0xd3, AEC_target_Y);
}

static void set_framesize(sensor_t *sensor, resolution_t resolution)
{
    SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
    switch (resolution)
    {
        case VGA:
            ESP_LOGI(TAG, "Perform VGA Init");
            break;
        case CIF:
        {
            ESP_LOGI(TAG, "Perform CIF Init");
            for (int i = 0; i < sizeof(GC0308_CIF_CROP) / 2; i++)
            {
                SCCB_Write(sensor->slv_addr, GC0308_CIF_CROP[i][0], GC0308_CIF_CROP[i][1]);
            }
        } break;
        case QVGA:
        {
            ESP_LOGI(TAG, "Perform QVGA Init");
            if (0/*sensor->use_windowing_model == false*/)
            {
                for (int i = 0; i < sizeof(GC0308_QVGA_CROP) / 2; i++)
                {
                    SCCB_Write(SCCB_ID, GC0308_QVGA_CROP[i][0], GC0308_QVGA_CROP[i][1]);
                }
            }
            else
            {
                for (int i = 0; i < sizeof(GC0308_QVGA_WINDOWING) / 2; i++)
                {
                    SCCB_Write(sensor->slv_addr, GC0308_QVGA_WINDOWING[i][0], GC0308_QVGA_WINDOWING[i][1]);
                }
            }
        } break;
    };
}

static void get_default_config(sensor_t *sensor)
{
    SCCB_Write(sensor->slv_addr, 0xfe, 0x00);
    uint8_t saturation = SCCB_Read(sensor->slv_addr, 0xb1);
    uint8_t edge_saturation = SCCB_Read(sensor->slv_addr, 0xbd);
    uint8_t contrast = SCCB_Read(sensor->slv_addr, 0xb3);
    uint8_t global_gin = SCCB_Read(sensor->slv_addr, 0x50);
    uint8_t aec_target_y = SCCB_Read(sensor->slv_addr, 0xd3);
    printf("Default configuration:\n");
    printf("Saturation: \t\t0x%0x\n", saturation);
    printf("Edge saturation: \t0x%0x\n", edge_saturation);
    printf("Contrast: \t\t0x%0x\n", contrast);
    printf("Global gain: \t\t0x%0x\n", global_gin);
    printf("AEC target Y: \t\t0x%0x\n", aec_target_y);
}

static int reset(sensor_t *sensor)
{
    int ret = 0;
    // Software Reset: clear all registers and reset them to their default values
    ret = write_reg(sensor->slv_addr, RESET_RELATED, 0x80);
    if(ret){
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret = write_regs(sensor->slv_addr, GC0308_DEFAULT_CONFIG);
    if (ret == 0) {
        ESP_LOGD(TAG, "Camera defaults loaded");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return ret;
}

static int init_status(sensor_t *sensor){
    sensor->status.brightness = 0;
    sensor->status.contrast = 0;
    sensor->status.saturation = 0;
    sensor->status.ae_level = 0;
    sensor->status.special_effect = 0;
    sensor->status.wb_mode = 0;

    // sensor->status.agc_gain = 30;
    // int agc_gain = read_reg(sensor, BANK_SENSOR, GAIN);
    // for (int i=0; i<30; i++){
    //     if(agc_gain >= agc_gain_tbl[i] && agc_gain < agc_gain_tbl[i+1]){
    //         sensor->status.agc_gain = i;
    //         break;
    //     }
    // }

    // sensor->status.aec_value = ((uint16_t)get_reg_bits(sensor, BANK_SENSOR, REG45, 0, 0x3F) << 10)
    //                          | ((uint16_t)read_reg(sensor, BANK_SENSOR, AEC) << 2)
    //                          | get_reg_bits(sensor, BANK_SENSOR, REG04, 0, 3);//0 - 1200
    // sensor->status.quality = read_reg(sensor, BANK_DSP, QS);
    // sensor->status.gainceiling = get_reg_bits(sensor, BANK_SENSOR, COM9, 5, 7);

    // sensor->status.awb = get_reg_bits(sensor, BANK_DSP, CTRL1, 3, 1);
    // sensor->status.awb_gain = get_reg_bits(sensor, BANK_DSP, CTRL1, 2, 1);
    // sensor->status.aec = get_reg_bits(sensor, BANK_SENSOR, COM8, 0, 1);
    // sensor->status.aec2 = get_reg_bits(sensor, BANK_DSP, CTRL0, 6, 1);
    // sensor->status.agc = get_reg_bits(sensor, BANK_SENSOR, COM8, 2, 1);
    // sensor->status.bpc = get_reg_bits(sensor, BANK_DSP, CTRL3, 7, 1);
    // sensor->status.wpc = get_reg_bits(sensor, BANK_DSP, CTRL3, 6, 1);
    // sensor->status.raw_gma = get_reg_bits(sensor, BANK_DSP, CTRL1, 5, 1);
    // sensor->status.lenc = get_reg_bits(sensor, BANK_DSP, CTRL1, 1, 1);
    // sensor->status.hmirror = get_reg_bits(sensor, BANK_SENSOR, REG04, 7, 1);
    // sensor->status.vflip = get_reg_bits(sensor, BANK_SENSOR, REG04, 6, 1);
    // sensor->status.dcw = get_reg_bits(sensor, BANK_DSP, CTRL2, 5, 1);
    // sensor->status.colorbar = get_reg_bits(sensor, BANK_SENSOR, COM7, 1, 1);

    // sensor->status.sharpness = 0;//not supported
    // sensor->status.denoise = 0;
    return 0;
}

int gc0308_init(sensor_t *sensor)
{
    sensor->reset = reset;
    sensor->init_status = init_status;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_contrast  = set_contrast;
    // sensor->set_brightness= set_brightness;
    // sensor->set_saturation= set_saturation;

    // sensor->set_quality = set_quality;
    // sensor->set_colorbar = set_colorbar;

    // sensor->set_gainceiling = set_gainceiling_sensor;
    // sensor->set_gain_ctrl = set_agc_sensor;
    // sensor->set_exposure_ctrl = set_aec_sensor;
    // sensor->set_hmirror = set_hmirror_sensor;
    // sensor->set_vflip = set_vflip_sensor;

    // sensor->set_whitebal = set_awb_dsp;
    // sensor->set_aec2 = set_aec2;
    // sensor->set_aec_value = set_aec_value;
    // sensor->set_special_effect = set_special_effect;
    // sensor->set_wb_mode = set_wb_mode;
    // sensor->set_ae_level = set_ae_level;

    // sensor->set_dcw = set_dcw_dsp;
    // sensor->set_bpc = set_bpc_dsp;
    // sensor->set_wpc = set_wpc_dsp;
    // sensor->set_awb_gain = set_awb_gain_dsp;
    // sensor->set_agc_gain = set_agc_gain;

    // sensor->set_raw_gma = set_raw_gma_dsp;
    // sensor->set_lenc = set_lenc_dsp;

    // //not supported
    // sensor->set_sharpness = set_sharpness;
    // sensor->set_denoise = set_denoise;

    // sensor->get_reg = get_reg;
    // sensor->set_reg = set_reg;
    // sensor->set_res_raw = set_res_raw;
    // sensor->set_pll = _set_pll;
    // sensor->set_xclk = set_xclk;
    ESP_LOGD(TAG, "GC0308 Attached");
    return 0;
}

#include "epd_board.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

#define NUMBER_OF_SAMPLES 100

static adc_oneshot_unit_handle_t adc1_handle;
const static adc_channel_t adc1_channel = ADC_CHANNEL_7;
static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static bool do_calibration1_chan0 = false;

const static char *TAG = "epd_temperature";

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void epd_board_temperature_init_v2() {
  // do not use deprecated API of adc1_get_raw
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = SOC_ADC_RTC_MAX_BITWIDTH,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(adc1_handle, adc1_channel, &config);
    do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, adc1_channel, ADC_ATTEN_DB_6, &adc1_cali_chan0_handle);
}

float epd_board_ambient_temperature_v2() {
  int adc_raw;
  int voltage;
  float voltage_average = 0;
  for (int i = 0; i < NUMBER_OF_SAMPLES; i++) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc1_channel, &adc_raw));
    if (do_calibration1_chan0) {
      ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &voltage));
    } else {
      voltage = adc_raw * 1100 / (1LLU << SOC_ADC_RTC_MAX_BITWIDTH);
    }
    ESP_LOGD(TAG, "ADC1 channel %d raw: %d\tvoltage: %dmV", adc1_channel, adc_raw, voltage);
    voltage_average += voltage;
  }
  voltage_average /= NUMBER_OF_SAMPLES;
  // FIXME: this may not be accurate
  return (voltage_average - 300.0) / 10.0;
}

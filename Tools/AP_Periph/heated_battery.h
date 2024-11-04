#pragma once

#ifdef HAL_PERIPH_ENABLE_HEATED_BATTERY

class HeatedBattery {
public:
    friend class AP_Periph_FW;
    HeatedBattery(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 enable;
    AP_Int8 id;
    AP_Float rate;
    uint32_t last_send_ms;
    uint8_t heater_power;

    AP_HAL::AnalogSource **adc;
    uint8_t adc_allocated;

    void heater_on(AP_HAL::GPIO* gpio);
    void heater_off(AP_HAL::GPIO* gpio);

};

#endif // HAL_PERIPH_ENABLE_HEATED_BATTERY


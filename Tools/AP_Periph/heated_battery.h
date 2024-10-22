#pragma once

#ifdef HAL_PERIPH_ENABLE_HEATED_BATTERY

class HeatedBattery {
public:
    friend class AP_Periph_FW;
    HeatedBattery(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 num_cells;
    AP_Int8 id;
    AP_Int8 cell1_pin;
    AP_Float rate;
    uint32_t last_send_ms;
    uint8_t heater_power;

    AP_HAL::AnalogSource **cells;
    uint8_t cells_allocated;
};

#endif // HAL_PERIPH_ENABLE_HEATED_BATTERY


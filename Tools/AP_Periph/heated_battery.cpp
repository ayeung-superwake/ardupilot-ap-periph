#include "AP_Periph.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#endif

#ifdef HAL_PERIPH_ENABLE_HEATED_BATTERY

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

#ifndef AP_PERIPH_BATTERY_BALANCE_NUMCELLS_DEFAULT
#define AP_PERIPH_BATTERY_BALANCE_NUMCELLS_DEFAULT 0
#endif

#ifndef AP_PERIPH_BATTERY_BALANCE_RATE_DEFAULT
#define AP_PERIPH_BATTERY_BALANCE_RATE_DEFAULT 1
#endif

#ifndef AP_PERIPH_BATTERY_BALANCE_CELL1_PIN_DEFAULT
#define AP_PERIPH_BATTERY_BALANCE_CELL1_PIN_DEFAULT 1
#endif

#ifndef AP_PERIPH_BATTERY_BALANCE_ID_DEFAULT
#define AP_PERIPH_BATTERY_BALANCE_ID_DEFAULT 0
#endif


const AP_Param::GroupInfo HeatedBattery::var_info[] {
    // @Param: _ID
    // @DisplayName: Battery ID
    // @Description: Battery ID to match against other batteries
    // @Range: 0 127
    AP_GROUPINFO("_ENABLE", 1, HeatedBattery, enable, 0),

    // @Param: _ID
    // @DisplayName: Battery ID
    // @Description: Battery ID to match against other batteries
    // @Range: 0 127
    AP_GROUPINFO("_ID", 2, HeatedBattery, id, AP_PERIPH_BATTERY_BALANCE_ID_DEFAULT),
        
    // @Param: _RATE
    // @DisplayName: Send Rate
    // @Description: Rate to send cell information
    // @Range: 0 20
    AP_GROUPINFO("_RATE", 3, HeatedBattery, rate, AP_PERIPH_BATTERY_BALANCE_RATE_DEFAULT),

    AP_GROUPEND
};

HeatedBattery::HeatedBattery(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    heater_power = 0;
}

void HeatedBattery::heater_on(AP_HAL::GPIO* gpio)
{
    gpio->pinMode(80, HAL_GPIO_OUTPUT);
    gpio->write(80, 1);
}

void HeatedBattery::heater_off(AP_HAL::GPIO* gpio)
{
    gpio->pinMode(80, HAL_GPIO_OUTPUT);
    gpio->write(80, 0);
}

void AP_Periph_FW::heated_battery_update()
{
    // TODO: Read GPIO to determine ID
    // TODO: Read Temperature
    // TODO: Read Cell Voltage
    // TODO: Turn on and off heater pin
    // TODO: Control heater output, min/max temperature, min voltage, max current
    // TODO: Status bits 
    // TODO: EEPROM access

    const int8_t ncell = 16;
    if (ncell <= 0) {
        return;
    }

    // allocate adc channels
    if (heated_battery.adc == nullptr) {
        heated_battery.adc = NEW_NOTHROW AP_HAL::AnalogSource*[ncell];
        if (heated_battery.adc == nullptr) {
            return;
        }
        heated_battery.adc_allocated = ncell;
        for (uint8_t i=0; i<heated_battery.adc_allocated; i++) {
            heated_battery.adc[i] = hal.analogin->channel(i);
        }
    }

    // return if it's not time to send.
    const uint32_t now = AP_HAL::millis();
    if (now - heated_battery.last_send_ms < 1000.0/heated_battery.rate.get()) {
        return;
    }
    heated_battery.last_send_ms = now;

    can_send_battery(uint8_t(heated_battery.id), heated_battery.adc, ncell);

    if (heated_battery.heater_power > 9) {
        // Turn on heater
        if (heated_battery.enable)
        {
            heated_battery.heater_on(hal.gpio);
            can_printf("Heated battery: On");
        }
        else
        {
            heated_battery.heater_off(hal.gpio);
            can_printf("Heated battery: Off");
        }
        if (heated_battery.heater_power > 12) {
            heated_battery.heater_power = 0;
        }
    } else {
        // Turn off heater
        heated_battery.heater_off(hal.gpio);
        can_printf("Heated battery: Off");
    }
    heated_battery.heater_power ++;

    for (uint8_t i = 3; i < 6; i++) {
        if (heated_battery.adc[i] != nullptr) {
            float voltage = heated_battery.adc[i]->voltage_average();
            float temperature = -96.1f + 66.5f * voltage + 4.38f * voltage * voltage;
            can_printf("Channel %d Voltage: %f Temperature: %f", i, voltage, temperature);
        }
    }

}

void AP_Periph_FW::can_send_battery(uint8_t id, AP_HAL::AnalogSource **values, uint8_t nvalues)
{
    // allocate space for the packet. This is a large
    // packet that won't fit on the stack, so dynamically allocate
    auto *pkt = NEW_NOTHROW ardupilot_equipment_power_BatteryInfoAux;
    uint8_t *buffer = NEW_NOTHROW uint8_t[ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_MAX_SIZE];
    if (pkt == nullptr || buffer == nullptr) {
        delete pkt;
        delete [] buffer;
        return;
    }

    pkt->voltage_cell.len = nvalues;
    for (uint8_t i=0; i<nvalues; i++) {
        auto *chan = values[i];
        if (chan == nullptr) {
            continue;
        }
        const float v = chan->voltage_average();
        pkt->voltage_cell.data[i] = v;
    }
    pkt->max_current = nanf("");
    pkt->nominal_voltage = nanf("");
    pkt->battery_id = id;

    // encode and send message:
    const uint16_t total_size = ardupilot_equipment_power_BatteryInfoAux_encode(pkt, buffer, !periph.canfdout());

    canard_broadcast(ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_SIGNATURE,
                     ARDUPILOT_EQUIPMENT_POWER_BATTERYINFOAUX_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     buffer,
                     total_size);

    delete pkt;
    delete [] buffer;
}

#endif  // HAL_PERIPH_ENABLE_HEATED_BATTERY


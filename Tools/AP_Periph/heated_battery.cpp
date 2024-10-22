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
    // @Param: _NUM_CELLS
    // @DisplayName: Number of battery cells
    // @Description: Number of battery cells to monitor
    // @Range: 0 64
    AP_GROUPINFO("_NUM_CELLS", 1, HeatedBattery, num_cells, AP_PERIPH_BATTERY_BALANCE_NUMCELLS_DEFAULT),

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

    // @Param: _CELL1_PIN
    // @DisplayName: First analog pin
    // @Description: Analog pin of the first cell. Later cells must be sequential
    // @Range: 0 127
    AP_GROUPINFO("_CELL1_PIN", 4, HeatedBattery, cell1_pin, AP_PERIPH_BATTERY_BALANCE_CELL1_PIN_DEFAULT),
        
    AP_GROUPEND
};

HeatedBattery::HeatedBattery(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    heater_power = 0;
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

    const int8_t ncell = heated_battery.num_cells;
    if (ncell <= 0) {
        return;
    }

    // allocate cell sources if needed
    if (heated_battery.cells == nullptr) {
        heated_battery.cells = NEW_NOTHROW AP_HAL::AnalogSource*[ncell];
        if (heated_battery.cells == nullptr) {
            return;
        }
        heated_battery.cells_allocated = ncell;
        for (uint8_t i=0; i<heated_battery.cells_allocated; i++) {
            heated_battery.cells[i] = hal.analogin->channel(heated_battery.cell1_pin + i);
        }
    }

    // return if it's not time to send.
    const uint32_t now = AP_HAL::millis();
    if (now - heated_battery.last_send_ms < 1000.0/heated_battery.rate.get()) {
        return;
    }
    heated_battery.last_send_ms = now;

    can_send_battery(uint8_t(heated_battery.id), heated_battery.cells, ncell);


    if (heated_battery.heater_power > 9) {
        // Turn on heater
        hal.gpio->pinMode(80, HAL_GPIO_OUTPUT);
        hal.gpio->write(80, 1);
        can_printf("Heated battery: On");
        if (heated_battery.heater_power > 12) {
            heated_battery.heater_power = 0;
        }
    } else {
        // Turn off heater
        hal.gpio->pinMode(80, HAL_GPIO_OUTPUT);
        hal.gpio->write(80, 0);
        can_printf("Heated battery: Off");
    }
    heated_battery.heater_power ++;

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


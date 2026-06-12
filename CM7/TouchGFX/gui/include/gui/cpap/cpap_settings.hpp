/**
 * cpap_settings.hpp - CPAP option store (singleton).
 *
 * Pure data + helpers. No TouchGFX widget or HAL dependency, so the same
 * struct can be persisted to flash or shared with the CM4 / back-end later.
 *
 * The menu controller (cpap_menu) edits these values; the screen view
 * formats them for display. Back-end logic that consumes them (blower,
 * humidifier, etc.) will be wired in later - for now these are pure GUI
 * state.
 */
#ifndef CPAP_SETTINGS_HPP
#define CPAP_SETTINGS_HPP

#include <stdint.h>

namespace cpap
{

/* Ramp time: 0 = OFF, 1 = AUTO, 2..10 = 5..45 minutes (in 5-min steps). */
enum class RampMode : uint8_t
{
    OFF = 0,
    AUTO = 1,
    MIN_5 = 2,
    MIN_10 = 3,
    MIN_15 = 4,
    MIN_20 = 5,
    MIN_25 = 6,
    MIN_30 = 7,
    MIN_35 = 8,
    MIN_40 = 9,
    MIN_45 = 10,
    NUM_VALUES
};

enum class OnOff : uint8_t
{
    OFF = 0,
    ON  = 1,
    NUM_VALUES
};

enum class TubeTempMode : uint8_t
{
    OFF = 0,
    AUTO = 1,
    MANUAL = 2,
    NUM_VALUES
};

enum class ClimateMode : uint8_t
{
    AUTO = 0,
    MANUAL = 1,
    NUM_VALUES
};

/* Humidity level: 0 = OFF, 1..8 are integer steps. */
enum class HumidityLevel : uint8_t
{
    OFF = 0,
    L1 = 1,
    L2 = 2,
    L3 = 3,
    L4 = 4,
    L5 = 5,
    L6 = 6,
    L7 = 7,
    L8 = 8,
    NUM_VALUES
};

struct Settings
{
    RampMode       ramp;
    OnOff          epr;
    OnOff          smart_start;
    OnOff          smart_stop;
    TubeTempMode   tube_temp_mode;
    uint8_t        tube_temp_c;          /* used when mode = MANUAL, range 16..30 */
    ClimateMode    climate_mode;
    HumidityLevel  humidity;
    OnOff          airplane_mode;

    /* "Available only if enabled by care provider" gates. Front-end honours
     * these by hiding/locking the menu item. */
    bool           epr_allowed;
    bool           smart_start_allowed;
    bool           smart_stop_allowed;
};

/* Singleton accessor. Default values are populated on first call. */
Settings& getSettings();

/* -------------------------------------------------------------------------
 *  Live system status (drives the status-bar icons).
 *
 *  Settings above are user-editable and persistent (in concept). The fields
 *  here are runtime telemetry: which humidifier state the heater is in,
 *  whether Bluetooth is paired, current cellular signal strength. They are
 *  wired into the status-bar widgets in screenView::updateStatusBar().
 *
 *  Back-end code can write to these freely from the appropriate task; the
 *  view samples them once per second.
 * ------------------------------------------------------------------------- */
enum class HumidifierStatus : uint8_t
{
    NORMAL = 0,    /* heater idle / nominal - no icon shown          */
    FAULT,         /* over-temp, sensor lost, etc.                   */
    WARMING,       /* ramping up to setpoint                         */
    COOLING        /* setpoint reduced, plate cooling                */
};

/* Cellular: 0 = no signal (shows no-cellular icon), 1..4 = bar count.
 * 4 is full strength. The view picks the matching bar mask. */
enum class CellularSignal : uint8_t
{
    NONE = 0,
    BARS_1 = 1,
    BARS_2 = 2,
    BARS_3 = 3,
    BARS_4 = 4
};

struct SystemStatus
{
    HumidifierStatus humidifier;
    bool             bluetooth_connected;
    CellularSignal   cellular;
};

SystemStatus& getSystemStatus();

/* Convert enum values to short display strings (NUL-terminated, ASCII). */
const char* toString(RampMode v);
const char* toString(OnOff v);
const char* toString(TubeTempMode v);
const char* toString(ClimateMode v);
const char* toString(HumidityLevel v);

/* Convenience: walk an enum up/down by `delta`, clamped to valid range. */
RampMode      step(RampMode v, int delta);
OnOff         step(OnOff v, int delta);
TubeTempMode  step(TubeTempMode v, int delta);
ClimateMode   step(ClimateMode v, int delta);
HumidityLevel step(HumidityLevel v, int delta);
uint8_t       stepTubeTempC(uint8_t v, int delta);  /* clamps 16..30 */

} // namespace cpap

#endif // CPAP_SETTINGS_HPP

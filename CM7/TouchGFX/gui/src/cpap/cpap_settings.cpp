/**
 * cpap_settings.cpp - default values + enum-to-string helpers.
 */
#include <gui/cpap/cpap_settings.hpp>

namespace cpap {

namespace {

Settings g_settings = {
    /* ramp                */ RampMode::AUTO,
    /* epr                 */ OnOff::OFF,
    /* smart_start         */ OnOff::OFF,
    /* smart_stop          */ OnOff::OFF,
    /* tube_temp_mode      */ TubeTempMode::AUTO,
    /* tube_temp_c         */ 27U,
    /* climate_mode        */ ClimateMode::AUTO,
    /* humidity            */ HumidityLevel::L4,
    /* airplane_mode       */ OnOff::OFF,
    /* epr_allowed         */ true,
    /* smart_start_allowed */ true,
    /* smart_stop_allowed  */ true
};

/* Defaults: nothing connected, no humidifier activity. Back-end code
 * (humidifier task, BT stack, modem driver) writes into this freely. */
SystemStatus g_system_status = {
    /* humidifier          */ HumidifierStatus::NORMAL,
    /* bluetooth_connected */ false,
    /* cellular            */ CellularSignal::NONE
};

template <typename E>
E clampEnum(int v)
{
    int max_v = static_cast<int>(E::NUM_VALUES) - 1;
    if (v < 0) v = 0;
    if (v > max_v) v = max_v;
    return static_cast<E>(v);
}

} // namespace

Settings& getSettings()
{
    return g_settings;
}

SystemStatus& getSystemStatus()
{
    return g_system_status;
}

const char* toString(RampMode v)
{
    switch (v)
    {
    case RampMode::OFF:    return "Off";
    case RampMode::AUTO:   return "Auto";
    case RampMode::MIN_5:  return "5 min";
    case RampMode::MIN_10: return "10 min";
    case RampMode::MIN_15: return "15 min";
    case RampMode::MIN_20: return "20 min";
    case RampMode::MIN_25: return "25 min";
    case RampMode::MIN_30: return "30 min";
    case RampMode::MIN_35: return "35 min";
    case RampMode::MIN_40: return "40 min";
    case RampMode::MIN_45: return "45 min";
    default:               return "?";
    }
}

const char* toString(OnOff v)
{
    return (v == OnOff::ON) ? "On" : "Off";
}

const char* toString(TubeTempMode v)
{
    switch (v)
    {
    case TubeTempMode::OFF:    return "Off";
    case TubeTempMode::AUTO:   return "Auto (27 C)";
    case TubeTempMode::MANUAL: return "Manual";
    default:                   return "?";
    }
}

const char* toString(ClimateMode v)
{
    return (v == ClimateMode::AUTO) ? "Auto" : "Manual";
}

const char* toString(HumidityLevel v)
{
    switch (v)
    {
    case HumidityLevel::OFF: return "Off";
    case HumidityLevel::L1:  return "1";
    case HumidityLevel::L2:  return "2";
    case HumidityLevel::L3:  return "3";
    case HumidityLevel::L4:  return "4";
    case HumidityLevel::L5:  return "5";
    case HumidityLevel::L6:  return "6";
    case HumidityLevel::L7:  return "7";
    case HumidityLevel::L8:  return "8";
    default:                 return "?";
    }
}

RampMode      step(RampMode v,      int d) { return clampEnum<RampMode>     (static_cast<int>(v) + d); }
OnOff         step(OnOff v,         int d) { return clampEnum<OnOff>        (static_cast<int>(v) + d); }
TubeTempMode  step(TubeTempMode v,  int d) { return clampEnum<TubeTempMode> (static_cast<int>(v) + d); }
ClimateMode   step(ClimateMode v,   int d) { return clampEnum<ClimateMode>  (static_cast<int>(v) + d); }
HumidityLevel step(HumidityLevel v, int d) { return clampEnum<HumidityLevel>(static_cast<int>(v) + d); }

uint8_t stepTubeTempC(uint8_t v, int delta)
{
    int x = static_cast<int>(v) + delta;
    if (x < 16) x = 16;
    if (x > 30) x = 30;
    return static_cast<uint8_t>(x);
}

} // namespace cpap

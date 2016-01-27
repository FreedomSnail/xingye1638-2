/**
 * @file math/isa.h
 * @brief 气压高度转换工具atmospheric pressure conversion utilities
 *
 * Conversion functions are use to approximate altitude
 * from atmospheric pressure based on the standard model
 * and the International Standard Atmosphere (ISA)
 *
 * http://en.wikipedia.org/wiki/Atmospheric_pressure
 * http://en.wikipedia.org/wiki/International_Standard_Atmosphere
 *
 */

#ifndef ISA_H
#define ISA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include <math.h>

// Standard Atmosphere constants
/** ISA sea level standard atmospheric pressure in Pascal */
#define ISA_SEA_LEVEL_PRESSURE 101325.0f
/** ISA sea level standard temperature in Kelvin */
#define ISA_SEA_LEVEL_TEMP 288.15f
/** temperature laps rate in K/m */
#define ISA_TEMP_LAPS_RATE 0.0065f
/** earth-surface gravitational acceleration in m/s^2 */
#define ISA_GRAVITY 9.80665f
/** universal gas constant in J/(mol*K) */
#define ISA_GAS_CONSTANT 8.31447f
/** molar mass of dry air in kg/mol */
#define ISA_MOLAR_MASS 0.0289644f
/** universal gas constant / molar mass of dry air in J*kg/K */
#define ISA_AIR_GAS_CONSTANT (ISA_GAS_CONSTANT/ISA_MOLAR_MASS)
/** standard air density in kg/m^3 */
#define ISA_AIR_DENSITY 1.225f

/**
 * Get absolute altitude from pressure (using simplified equation).
 * Referrence pressure is standard pressure at sea level
 *
 * @param pressure current pressure in Pascal (Pa)
 * @return altitude in m in ISA conditions
 */
static inline float isa_altitude_of_pressure(float pressure)
{
    float ISA_M_OF_P_CONST = (ISA_AIR_GAS_CONSTANT *ISA_SEA_LEVEL_TEMP / ISA_GRAVITY);
    if (pressure > 0.f)
    {
        return (ISA_M_OF_P_CONST * logf(ISA_SEA_LEVEL_PRESSURE / pressure));
    }
    else
    {
        return 0.f;
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_ISA_H */

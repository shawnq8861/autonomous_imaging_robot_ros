#ifndef BQ4050_H
#define BQ4050_H

#define FUEL_GAUGE_ADDR                               (0x16 >> 1)
#define ERROR                                         0
#define MANUFACTURER_ADDR                             0x20
#define MANUFACTURER_ID                               21521
#define REALATIVE_STATE_ADDR                          0x0D
#define VOLTAGE_ADDR                                  0x09
#define CURRENT_ADDR                                  0x0A
#define BATTERY_STATUS_ADDR                           0x16
#define ABSOLUTE_STATE_ADDR                           0x0E
#define CURRENT_SHIFT                                 65536
#define CELL_VOLTAGE_ADDR_1                           0x3F
#define CELL_VOLTAGE_ADDR_2                           0x3E
#define CELL_VOLTAGE_ADDR_3                           0x3D
#define CELL_VOLTAGE_ADDR_4                           0x3C
#define MANUFACTURER_ADDRESS                          0x44
#define GAS_GAUGE_TOGGLE_BYTE_1                       0x2
#define GAS_GAUGE_TOGGLE_BYTE_2                       0x41
#define GAS_GAUGE_TOGGLE_BYTE_3                       0x00
#define GAS_GAUGE_TOGGLE_BYTE_4                       0x22
#define GAS_GAUGE_RESET_TRIES                         5

#endif

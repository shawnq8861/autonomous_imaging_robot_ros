#ifndef BQ24770_H
#define BQ24470_H

#define I2C_BUS_0                                  0
#define I2C_BUS_1                                  1
#define SUCCESS                                    0
#define ERR_I2C_REG_RW                             5
#define CHARGER_ADDR_I2C                           (0xD4 >> 1)
#define CHARGER_ADDR_SMB                           0x09
#define CHARGER_MAX_CHARGE_VOLTAGE_ADDR_I2C        0x0C
#define CHARGER_MAX_CHARGE_VOLTAGE_ADDR_SMB        0x15
#define CHARGER_MIN_SYS_VOLTAGE_ADDR_I2C           0x0E
#define CHARGER_MIN_SYS_VOLTAGE_ADDR_SMB           0x3E
#define CHARGER_CHARGE_CURRENT_ADDR_I2C            0x0A
#define CHARGER_CHARGE_CURRENT_ADDR_SMB            0x14
#define CHARGER_INPUT_CURRENT_ADDR_I2C             0x0F
#define CHARGER_INPUT_CURRENT_ADDR_SMB             0x3F
#define CHARGER_OPTION_0_ADDR_I2C                  0x00
#define CHARGER_OPTION_0_ADDR_SMB                  0x12
#define CHARGER_DEVICE_ID_ADDR_I2C                 0x09
#define CHARGER_DEVICE_ID_ADDR_SMB                 0xFF
#define CHARGER_DEVICE_ID_I2C                      0x41
#define CHARGER_DEVICE_ID_SMB                      0x0114
#define CHARGER_OPTION_0_ADDR_I2C_2                0x01
#define BOARD_TYPE_NULL                            0
#define BOARD_TYPE_I2C                             1
#define BOARD_TYPE_SMB                             2
#define MANUFACTURER_ID_REG                        0xFE
#define FUEL_GAUGE_ADDR                           (0x16 >> 1)
#define BOARD_ID                                   65
#define MIN_SYS_VOLTAGE                            0x003F
#define NULL_VAL                                   0
#define SECOND_CHARGER_MAX_CHARGE_VOLTAGE_ADDR_I2C 0x0D
#define SECOND_CHARGER_CHARGE_CURRENT_ADDR_I2C     0x0B
#define CHARGE_CURRENT_CONV                        0x1F
#define INPUT_CURRENT_CONV                         0x1FC0
#define DISABLE_WATCHDOG_TIMER                     0x9D
#define ENABLE_CHARGING_VAL_1                      0xFC
#define ENABLE_CHARGING_VAL_2                      0x9D
#define DEFAULT_CURRENT                            450
#define DEFAULT_MAX_VOLTAGE                        16800
#define DEAFULT_MIN_VOLTAGE                        12000
#define STOP_CHARGE_VOLTAGE                        16600
#define MAX_CHARGING_FLAG                          1050
#define STABLE_CURRENT_DROP                        500
#define CURRENT_PADDING                            200
#define CURRENT_ADJUSTMENT                         50
#define MAX_CHARGE_CURRENT_HZ                      10
#define SYNC_LOST_MAX                              5
#define MAX_MOD_DIF                                5
#define MAX_NEGATIVE_CURRENT_COUNT                 10
#define SIGNAL_TRIM_CHARGE                         5
#define CHARGE_MAINTENANCE_HZ                      .2
#define FIND_CHARGER_CURRENT_THRESH                -300
#define CHARGE_CURRENT_SAMPLE_SIZE                 7
#define CHARGE_CURRENT_MAX_RETRIES                 3
#define VOLTAGE_MAX_RETRIES                        3
#define ABS_MAX_RETRIES                            3
#define GPIO_MAX_RETRIES                           3
#define MAX_ABS_STATE                              110
#define MAX_VOLTAGE_THRESH                         16.5
#define BAT_RESET_WAIT                             3
#define CHARGER_CHIP_DEBUG                         0x12

#endif

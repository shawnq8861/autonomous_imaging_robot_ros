#ifndef ROS_ROBO_BOB_MRFC522_H
#define ROS_ROBO_BOB_MRFC522_H

/* MRFC522 commands */
#define PCD_IDLE              0x00
#define PCD_AUTHENT           0x0E
#define PCD_TRANSCEIVE        0x0C
#define PCD_RESET             0x0F
#define PCD_CALCCRC           0x03

/* Mirafare_One Commands (rfid tag itself) */
#define PICC_REQIDL           0x26
#define PICC_REQALL           0x52
#define PICC_ANTICOLL         0x93
#define PICC_SELECTTAG        0x93
#define PICC_HALT             0x05

/* MRFC522 register definitions */
#define     CommandReg            0x01
#define     CommIEnReg            0x02
#define     DivlEnReg             0x03
#define     CommIrqReg            0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFCfgReg              0x26
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D

#endif

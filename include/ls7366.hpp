#ifndef ROS_ROBO_BOB_LS7366_H
#define ROS_ROBO_BOB_LS7366_H

#define CLR             0x00
#define RD              0x40
#define WR              0x80
#define LOAD_COUNTER    0xC0

#define MDR0            0x8
#define MDR1            0x10
#define DTR             0x18
#define CNTR            0x20
#define OTR             0x28
#define STR             0x30

/*
    filter factor 1
    async index
    no index
    free-running
    4x quadrature 
*/
#define MDR0_CONF 0x3

/*
    no flag
    enabled
    32 bits
*/
#define MDR1_CONF 0x00

#endif
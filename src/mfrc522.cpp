/* General C++ includes */
#include <cstdlib>
#include <fstream>
#include <sys/file.h>
#include <std_msgs/Int8.h>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

/* ROS includes */
#include <ros/ros.h>

/* Package specific includes */
#include "../include/spi.hpp"
#include "../include/gpio.hpp"
#include "../include/utils.hpp"
#include "../include/mfrc522.hpp"
#include "../include/bob_status.hpp"
#include "../include/ftd4222_helper.hpp"

/* FTD4222 */
#include "ftd2xx.h"
#include "libft4222.h"

/* MRAA Includes */
#include "mraa/common.hpp"
#include "mraa/spi.hpp"

/* Service Includes */
#include <ros_robo_bob/ReadRFID.h>
#include <ros_robo_bob/GetLastRFID.h>

/* Constant definitions */
#define MFRC522_NODE_NAME "mfrc522"
#define MFRC522_MAX_RESPONSE_LEN 8
#define MFRC522_RESET_PIN 3
#define MFRC522_SAVE_FILE "last_rfid.txt"
#define MFRC522_SPI_PORT 0
#define MFRC522_RETRIES 6

/* Namespace definitions */
using namespace boost::interprocess;

/* Named mutex definitions */
named_mutex spiLock(open_or_create, SPI_LOCK_NAME);

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @addr                 Address to write to                               #
#           @val                  Value to write                                    #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Writes a value to a specific register on the mfrc522.              #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_write(mraa::Spi * spi, uint8_t addr, uint8_t val)
{
    FT_STATUS ft4222Status;
    uint8_t txData[2];
    uint16 sizeTransferred;

    txData[0] = (addr<<1)&0x7E;
    txData[1] = val;

    // Write the data to the mfrc522
    spi->write(txData, 2);

    return BOB_SUCCESS;

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @addr                 Address to write to                               #
#           @data                 RX data                                           #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Reads a specific register on the mfrc522.                          #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_read(mraa::Spi * spi, uint8_t addr, uint8_t & data)
{
    FT_STATUS ft4222Status;
    uint16 sizeTransferred;
    uint8_t rxBuffer[2];
    uint8_t txData[2];

    txData[0] = ((addr<<1)&0x7E)|0x80;
    txData[1] = 0x00;

    // Read data from the chip
    spi->transfer(txData, rxBuffer, 2);

    data = rxBuffer[1];

    return BOB_SUCCESS;

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @reg                  Register to modify                                #
#           @mask                 Mask to apply to the register                     #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Sets a bitmask for a specific register.                            #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_set_bitmask(mraa::Spi * spi, uint8_t reg, uint8_t mask)
{
    uint8_t data;
    BobStatus status = BOB_SUCCESS;

    status |= mfrc522_read(spi, reg, data);
    status |= mfrc522_write(spi, reg, data | mask);

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @reg                  Register to modify                                #
#           @mask                 Mask to apply to the register                     #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Removes a mask from a register.                                    #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_clear_bitmask(mraa::Spi * spi, uint8_t reg, uint8_t mask)
{
    uint8_t data;
    BobStatus status = BOB_SUCCESS;

    status |= mfrc522_read(spi, reg, data);
    status |= mfrc522_write(spi, reg, data & (~mask));

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @sendData             Data to send to the card                          #
#           @sendLen              Length of data to send to the card                #
#           @backData             Data received from the card                       #
#           @backLen              Length of data received from the card             #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Sends/receives data to/from the RFID tag                           #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_card_transceive(mraa::Spi * spi, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen)
{
    BobStatus status = BOB_SUCCESS;
    int loopCount = 0;
    uint8_t lastBits, result;

    // Allow interrupts
    status |= mfrc522_write(spi, CommIEnReg, 0xF7);

    // Clear IRQ bit
    status |= mfrc522_clear_bitmask(spi, CommIrqReg, 0x80);

    // Flush FIFO Buffer
    status |= mfrc522_set_bitmask(spi, FIFOLevelReg, 0x80);

    // Cancel any pending commands
    status |= mfrc522_write(spi, CommandReg, PCD_IDLE);

    // Write data to the FIFO
    for (int i = 0; i < sendLen; i++)
        status |= mfrc522_write(spi, FIFODataReg, sendData[i]);

    // Execute Command
    status |= mfrc522_write(spi, CommandReg, PCD_TRANSCEIVE);

    // StartSend=1,transmission of data starts
    status |= mfrc522_set_bitmask(spi, BitFramingReg, 0x80);

    do
    {
        status |= mfrc522_read(spi, CommIrqReg, result);
        loopCount++;
    }
    while (!(result & 0x01) &&     //timeout interrupt
           !(result & 0x30) &&     //desired end of data stream interrupt or reader idle interrupt
           !(loopCount > 10000));    //prevent infinite loops when rfid reader isn't connected

    if(loopCount > 10000)
    {
        return BOB_CANNOT_FIND_RFID;
    }
    else if (result & 0x01)
    {
        // Timeout occured, so no tag was read
        status = BOB_CANNOT_FIND_RFID;
    }

    mfrc522_clear_bitmask(spi, BitFramingReg, 0x80);
    mfrc522_read(spi, ErrorReg, result);

    // BufferOvfl | CollErr | CRCErr | ProtocolErr
    if(!(result & 0x1B))
    {
        mfrc522_read(spi, FIFOLevelReg, result);

        *backLen = (result-1) * 8;

        // These bits of this register tell us how many bits of the last byte in the FIFO are valid
        mfrc522_read(spi, ControlReg, lastBits);
        lastBits &= 0x07;

        if (lastBits > 0)
            *backLen += lastBits;
        else
            *backLen += 8;

        if (result > MFRC522_MAX_RESPONSE_LEN)
            result = MFRC522_MAX_RESPONSE_LEN;

        //Read recieved data from FIFO
        for (int i = 0; i < result; i++)
            mfrc522_read(spi, FIFODataReg, backData[i]);
    }

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @buf                  Contains the UID of the tag                       #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Attempts to get the RFID tags UID w/ anticollision enabled.        #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_anticollision(mraa::Spi * spi, uint8_t *buf)
{
    BobStatus status = BOB_SUCCESS;
    uint8_t check=0;
    uint16_t unLen;

    // TxLastBists = BitFramingReg[2..0]
    mfrc522_write(spi, BitFramingReg, 0x00);

    // Select code for anticollision level 1
    buf[0] = PICC_ANTICOLL;

    //The value of ’0x20′ for NVB (number of valid bits or buf[1]) means
    //that the terminal is not sending any portion of the cascade level 1 UID
    buf[1] = 0x20;

    status = mfrc522_card_transceive(spi, buf, 2, buf, &unLen);

    if (status == BOB_SUCCESS)
    {
        //Check for matching card UID
        check ^= buf[0];
        check ^= buf[1];
        check ^= buf[2];
        check ^= buf[3];
        if (check != buf[4])
            status = BOB_CANNOT_READ_RFID;

    }

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @reqMode              Request mode                                      #
#           @TagType              Buffer for tag type info                          #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Attempts to perform a request to an RFID tag.                      #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_request(mraa::Spi * spi, uint8_t reqMode, uint8_t *TagType)
{
    BobStatus status = BOB_SUCCESS;

    // Number of bits received
    uint16_t backBits;

    // TxLastBists = BitFramingReg[2..0] because request commands are 7-bit commands
    mfrc522_write(spi, BitFramingReg, 0x07);

    // Either PICC_REQIDL (request idle) or PICC_REQALL (request all)
    TagType[0] = reqMode;
    status = mfrc522_card_transceive(spi, TagType, 1, TagType, &backBits);

    // We expect 16 bits back, otherwise something went wrong
    if (backBits != 0x10)
        status = BOB_CANNOT_FIND_RFID;

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @pIndata              Data to compute CRC with                          #
#           @len                  Length of the data buffer                         #
#           @pOutData             The computed 16 bit CRC                           #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Sets a bitmask for a specific register.                            #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_calculate_crc(mraa::Spi * spi, uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
{
    BobStatus status = BOB_SUCCESS;
    uint8_t c, n;

    // CRCIrq = 0
    status |= mfrc522_clear_bitmask(spi, DivIrqReg, 0x04);

    // Clear FIFO
    status |= mfrc522_set_bitmask(spi, FIFOLevelReg, 0x80);
    status |= mfrc522_write(spi, CommandReg, PCD_IDLE);

    // Write data to the FIFO
    for (int i = 0; i < len; i++)
        status |= mfrc522_write(spi, FIFODataReg, *(pIndata+i));

    status |= mfrc522_write(spi, CommandReg, PCD_CALCCRC);

    // Wait for CRC calculation to complete
    c = 0xFF;
    do
    {
        status |= mfrc522_read(spi, DivIrqReg, n);
        c--;
    }
    while ((c!=0) && !(n&0x04));

    // Read crc calculation result
    status |= mfrc522_read(spi, CRCResultRegL, pOutData[0]);
    status |= mfrc522_read(spi, CRCResultRegM, pOutData[1]);

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Halts comms with a tag.                                            #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_halt(mraa::Spi * spi)
{
    uint16_t unLen;
    uint8_t buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    if(mfrc522_calculate_crc(spi, buff, 2, &buff[2]) != BOB_SUCCESS)
        return BOB_MFRC522_SPI_RW_ERROR;

    return mfrc522_card_transceive(spi, buff, 4, buff, &unLen);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @tag                  Tag to select                                     #
#           @size                 Size of data from the transceive                  #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Attempts to delect a tag based on its UID.                         #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_select_tag(mraa::Spi * spi, uint8_t *tag, uint8_t * size)
{
    BobStatus status = BOB_SUCCESS;
    uint16_t recv_bits;
    uint8_t buffer[9];

    buffer[0] = PICC_SELECTTAG;
    buffer[1] = 0x70;

    for (int i = 0; i < 5; i++)
        buffer[i+2] = *(tag+i);

    if(mfrc522_calculate_crc(spi, buffer, 7, &buffer[7]) != BOB_SUCCESS)
        return BOB_MFRC522_SPI_RW_ERROR;

    status = mfrc522_card_transceive(spi, buffer, 9, buffer, &recv_bits);

    if ((status = BOB_SUCCESS) && (recv_bits == 0x18))
        *size = buffer[0];

    else
        *size = 0;

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @handle               Handle to the FTDI device                         #
#           @tag                  32 bit UID of the tag                             #
#           @found                Set to true if a tag is found                     #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #
#                                                                                   #
#   Description: Scans for an RFID tag, and updates the tag UID and found           #
#                variables accordingly.                                             #
#                                                                                   #
************************************************************************************/

BobStatus mfrc522_scan_for_rfid(mraa::Spi * spi, uint8_t tag[4], bool & found)
{
    BobStatus status;
    uint8_t size;
    uint8_t str[MFRC522_MAX_RESPONSE_LEN];

    status = mfrc522_request(spi, PICC_REQIDL, str);

    if(status != BOB_SUCCESS)
        return status;

    status = mfrc522_anticollision(spi, str);

    if(status == BOB_SUCCESS)
    {
        memcpy(tag, str, 4); //4 byte tag, 5th byte is crc
        mfrc522_select_tag(spi, str, &size);
        mfrc522_halt(spi);
        found = true;

        return BOB_SUCCESS;
    }

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @device               Reference to the node variable for the ft device  #
#                                                                                   #
#   Description: Initializes the mfrc522 device.                                    #
#                                                                                   #
************************************************************************************/

void mfrc522_init(mraa::Spi * spi)
{
    mfrc522_write(spi, CommandReg, PCD_RESET);

    // Timer Period = TPrescaler*TreloadVal/6.78MHz = 24ms (timeout length for transceive)
    // Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    mfrc522_write(spi, TModeReg, 0x8D);
    mfrc522_write(spi, TPrescalerReg, 0x3E);

    //TModeReg[3..0] + TPrescalerReg
    mfrc522_write(spi, TReloadRegL, 30);
    mfrc522_write(spi, TReloadRegH, 0);

     //Force 100% ASK Modulation
    mfrc522_write(spi, TxAutoReg, 0x40);

    // RxGain = 48dB, max gain for greatest read distance
    mfrc522_write(spi, RFCfgReg, 0x7F);

    // Turn the antenna of
    mfrc522_set_bitmask(spi, TxControlReg, 0x03);

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @tagID                The tag id to save                                #
#                                                                                   #
#   Description: Saves a tag ID to a data file, which serves as the storage place   #
#                for the last known RFID tag.                                       #
#                                                                                   #
************************************************************************************/

void mfrc522_save_rfid(uint32_t tagID)
{
    std::ofstream stream;

    // Create the data dir if it doesn't already exist yet
    createDataDir();

    // Create the RFID code to the file
    stream.open ("data/"+std::string(MFRC522_SAVE_FILE), std::ofstream::out);
    stream << tagID << "\n";
    stream.close();
}


/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @req                  The request from the client                       #
#           @res                  The response object for the client                #
#           @device               Pointer to the FTD4222 node handle                #
#                                                                                   #
#   Description: Services a request from a client for a RFID scan.                  #
#                                                                                   #
************************************************************************************/

bool RFIDServiceCallback(ros_robo_bob::ReadRFID::Request &req, ros_robo_bob::ReadRFID::Response &res,
                         mraa::Spi * spi)
{
    uint8_t tag[4];
    BobStatus status;
    bool found = false;

    for(int i = 0; i < MFRC522_RETRIES; i++)
    {
        // Enable the chip
        GPIOWrite(MFRC522_RST_PIN, VALUE_HIGH);

        // Initialize the device
        mfrc522_init(spi);

        // Scan for the tag and populate the response
        res.status = mfrc522_scan_for_rfid(spi, tag, found);
        res.tagID = (tag[0] << 24) | (tag[1] << 16) | (tag[2] << 8) | tag[3];
        res.found = found;

        // Save the tag if we found it
        if(found)
            mfrc522_save_rfid(res.tagID);

        // Disable the chip
        GPIOWrite(MFRC522_RST_PIN, VALUE_LOW);

        if(res.status == BOB_SUCCESS)
            break;
    }

    return true;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @req                  The request from the client                       #
#           @res                  The response object for the client                #
#                                                                                   #
#   Description: Returns the last known RFID that the robot read as a service.      #
#                                                                                   #
************************************************************************************/

bool getLastRFIDServiceCallback(ros_robo_bob::GetLastRFID::Request &req, ros_robo_bob::GetLastRFID::Response &res)
{
    bool found;
    std::string tag;
    BobStatus status;

    // Attempt to open the rfid file
    std::ifstream file("data/"+std::string(MFRC522_SAVE_FILE));

    if(file.good())
    {
        // Read the rfid tag
        getline(file, tag);
        res.status = BOB_SUCCESS;
        res.tagID = (uint32_t)std::stol(tag);
    }
    else
    {
        res.status = BOB_NO_RFID_SAVED;
        res.tagID = 0;
    }

    file.close();

    return true;
}

int main(int argc, char **argv)
{
    // Initialize the rfid node
    ros::init(argc, argv, MFRC522_NODE_NAME);

    // The ros handle for this node
    ros::NodeHandle handle;

    // Configure the RFID reset pin as out
    GPIOExport(MFRC522_RST_PIN);
    GPIODirection(MFRC522_RST_PIN, "out");

    // Initialize the SPI instance
    mraa::Spi spi(MFRC522_SPI_PORT);

    // Create a service to handle tag scan requests.
    ros::ServiceServer RFIDService = handle.advertiseService<ros_robo_bob::ReadRFID::Request,
                                        ros_robo_bob::ReadRFID::Response>(addBase2Topic(MFRC522_NODE_NAME, "scan_for_tag"),
                                        boost::bind(RFIDServiceCallback, _1, _2, &spi));

    // Create a service to get the robots last known tag
    ros::ServiceServer getLastRFIDService = handle.advertiseService<ros_robo_bob::GetLastRFID::Request,
                                        ros_robo_bob::GetLastRFID::Response>(addBase2Topic(MFRC522_NODE_NAME, "get_last_tag"),
                                        getLastRFIDServiceCallback);

    ros::spin();
}
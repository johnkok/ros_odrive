#ifndef ODRIVE_ENDPOINT_HPP_
#define ODRIVE_ENDPOINT_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <endian.h>
#include <mutex>
#include <libusb-1.0/libusb.h>

// ODrive Device Info
#define ODRIVE_USB_VENDORID     0x1209
#define ODRIVE_USB_PRODUCTID    0x0D32

// ODrive USB Protool
#define ODRIVE_TIMEOUT 2000
#define ODRIVE_MAX_BYTES_TO_RECEIVE 64
#define ODRIVE_MAX_RESULT_LENGTH 100
//#define ODRIVE_DEFAULT_CRC_VALUE 0x7411
#define ODRIVE_DEFAULT_CRC_VALUE 0x9b40
#define ODRIVE_PROTOCOL_VERSION 1

// ODrive Comm
#define ODRIVE_COMM_SUCCESS 0
#define ODRIVE_COMM_ERROR   1

// Endpoints (from target)
#define CDC_IN_EP                                   0x81  /* EP1 for data IN (target) */
#define CDC_OUT_EP                                  0x01  /* EP1 for data OUT (target) */
#define CDC_CMD_EP                                  0x82  /* EP2 for CDC commands */
#define ODRIVE_IN_EP                                0x83  /* EP3 IN: ODrive device TX endpoint */
#define ODRIVE_OUT_EP                               0x03  /* EP3 OUT: ODrive device RX endpoint */

// CDC Endpoints parameters
#define CDC_DATA_HS_MAX_PACKET_SIZE                 0x40  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 0x40  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         0x08  /* Control Endpoint Packet size */

#define USB_CDC_CONFIG_DESC_SIZ                     (67 + 39)
#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE

#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23


typedef std::vector<uint8_t> commBuffer;

class odrive_endpoint {

    public:
        odrive_endpoint();
        ~odrive_endpoint();
        int init(uint64_t serialNumber);
        void remove(void);

        template<typename T>
            int getData(int id, T& value);
        template<typename TT>
            int setData(int id, const TT& value);

        int execFunc(int id);

        int endpointRequest(int endpoint_id, commBuffer& received_payload,
            int& received_length, commBuffer payload, bool ack = false,
            int length = 0, bool read = false, int address = 0);

    private:
        libusb_context* libusb_context_;
        short outbound_seq_no_ = 0;
        libusb_device_handle *odrive_handle_ = NULL;
        std::mutex ep_lock;

        void appendShortToCommBuffer(commBuffer& buf, const short value);
        void appendIntToCommBuffer(commBuffer& buf, const int value);
        commBuffer decodeODrivePacket(commBuffer& buf, short& seq_no, commBuffer& received_packet);
        commBuffer createODrivePacket(short seq_no, int endpoint_id, short response_size,
            bool read, int address, const commBuffer& input);
};

#endif // ODRIVE_ENDPOINT_HPP_

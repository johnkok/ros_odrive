#include "ros_odrive/odrive.hpp"
#include "ros_odrive/odrive_endpoint.hpp"

using namespace std;

/**
 *
 * Odrive endpoint constructor
 * initialize USB library and local variables  
 *
 */
odrive_endpoint::odrive_endpoint()
{

    if (libusb_init(&libusb_context_) != LIBUSB_SUCCESS) {
	ROS_ERROR("* Error initializing USB!");
    }
}

/**
 *
 * Odrive endpoint destructor
 * release USB library
 *
 */
odrive_endpoint::~odrive_endpoint()
{

    if (libusb_context_ != NULL) {
        libusb_exit(libusb_context_);
	libusb_context_ = NULL;
    }
}

/**
 *
 * Append short data to data buffer
 * @param buf data buffer
 * @param value data to append
 *
 */
void odrive_endpoint::appendShortToCommBuffer(commBuffer& buf, const short value) 
{
    buf.push_back((value >> 0) & 0xFF);
    buf.push_back((value >> 8) & 0xFF);
}

/**
 *
 * Append int data to data buffer
 * @param buf data buffer
 * @param value data to append
 *
 */
void odrive_endpoint::appendIntToCommBuffer(commBuffer& buf, const int value) 
{
    buf.push_back((value >> 0) & 0xFF);
    buf.push_back((value >> 8) & 0xFF);
    buf.push_back((value >> 16) & 0xFF);
    buf.push_back((value >> 24) & 0xFF);
}

/**
 *
 *  Decode odrive packet
 *  @param buf data buffer
 *  @param seq_no packet sequence number
 *  @param received_packet received buffer
 *  @return data buffer
 *
 */
commBuffer odrive_endpoint::decodeODrivePacket(commBuffer& buf, 
		short& seq_no, commBuffer& received_packet) 
{
    commBuffer payload;

    memcpy(&seq_no, &buf[0], sizeof(short));
    seq_no &= 0x7fff;
    for (commBuffer::size_type i = 2; i < buf.size(); ++i) {
        payload.push_back(buf[i]);
    }
    return payload;
}

/**
 *
 * Read data buffer from Odrive harware
 * @param seq_no next sequence number
 * @param endpoint_id USB endpoint ID
 * @param response_size maximum data length to be read
 * @param read append request address
 * @param address desctination address
 * @param input data buffer to send
 * @return data buffer read
 *
 */
commBuffer odrive_endpoint::createODrivePacket(short seq_no, int endpoint_id,
                short response_size, bool read, int address, const commBuffer& input) 
{
    commBuffer packet;
    short crc = 0;

    if ((endpoint_id & 0x7fff) == 0) {
        crc = ODRIVE_PROTOCOL_VERION;
    }
    else {
        crc = ODRIVE_DEFAULT_CRC_VALUE;
    }

    appendShortToCommBuffer(packet, seq_no);
    appendShortToCommBuffer(packet, endpoint_id);
    appendShortToCommBuffer(packet, response_size);
    if (read) {
        appendIntToCommBuffer(packet, address);
    }

    for (uint8_t b : input) {
        packet.push_back(b);
    }

    appendShortToCommBuffer(packet, crc);

    return packet;
}

/**
 *
 *  Read value from ODrive
 *  @param id odrive ID
 *  @param value Data read
 *  @return ODRIVE_OK on success
 *
 */
template<typename T>
int odrive_endpoint::getData(int id, T& value) 
{
    commBuffer tx;
    commBuffer rx;
    int rx_size;

    int result = endpointRequest(id, rx,
                    rx_size, tx, 1 /* ACK */, sizeof(value));
    if (result != ODRIVE_OK) {
        return result;
    }

    memcpy(&value, &rx[0], sizeof(value));

    return ODRIVE_OK;
}


/**
 *
 *  Request function to ODrive
 *  @param id odrive ID
 *  @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::execFunc(int endpoint_id) 
{
    commBuffer tx;
    commBuffer rx;
    int rx_length;

    return endpointRequest(endpoint_id, rx, rx_length, tx, 1, 0);
}

/**
 *
 *  Write value to Odrive
 *  @param id odrive ID
 *  @param value Data to be written
 *  @return ODRIVE_OK on success
 *
 */
template<typename TT>
int odrive_endpoint::setData(int endpoint_id, const TT& value) 
{
    commBuffer tx;
    commBuffer rx;
    int rx_length;

    for(int i = 0; i < sizeof(value); i++){
       tx.push_back(((unsigned char*)&value)[i]);
    }

    return endpointRequest(endpoint_id, rx, rx_length, tx, 1, 0);
}

/**
 *
 * Request endpoint
 * @param handle USB device handler
 * @param endpoint_id odrive ID
 * @param received_payload receive buffer
 * @param received_length receive length
 * @param payload data read
 * @param ack request acknowledge
 * @param length data length
 * @param read send read address
 * @param address read address
 * @return LIBUSB_SUCCESS on success
 *
 */
int odrive_endpoint::endpointRequest(int endpoint_id, commBuffer& received_payload, 
		int& received_length, commBuffer payload, 
		bool ack, int length, bool read, int address) 
{
    commBuffer send_buffer;
    commBuffer receive_buffer;
    unsigned char receive_bytes[ODRIVE_MAX_RESULT_LENGTH] = { 0 };
    int sent_bytes = 0;
    int received_bytes = 0;
    short received_seq_no = 0;

    ep_lock.lock();

    // Prepare sequence number
    if (ack) {
        endpoint_id |= 0x8000;
    }
    outbound_seq_no_ = (outbound_seq_no_ + 1) & 0x7fff;
    outbound_seq_no_ |= LIBUSB_ENDPOINT_IN; 
    short seq_no = outbound_seq_no_;

    // Create request packet
    commBuffer packet = createODrivePacket(seq_no, endpoint_id, length, read, address, payload);

    // Transfer paket to target
    int result = libusb_bulk_transfer(odrive_handle_, ODRIVE_OUT_EP, 
		    packet.data(), packet.size(), &sent_bytes, 0);
    if (result != LIBUSB_SUCCESS) {
	ROS_ERROR("* Error in transfering data to USB!");
        ep_lock.unlock();
        return result;
    } else if (packet.size() != sent_bytes) {
        ROS_ERROR("* Error in transfering data to USB, not all data transferred!");
    }

    // Get responce
    if (ack) {
        result = libusb_bulk_transfer(odrive_handle_, ODRIVE_IN_EP, 
			receive_bytes, ODRIVE_MAX_BYTES_TO_RECEIVE, 
			&received_bytes, ODRIVE_TIMEOUT);
        if (result != LIBUSB_SUCCESS) {
            ROS_ERROR("* Error in reading data from USB!");
            ep_lock.unlock();
    	    return result;
        }

	// Push recevived data to buffer
        for (int i = 0; i < received_bytes; i++) {
            receive_buffer.push_back(receive_bytes[i]);
        }

        received_payload = decodeODrivePacket(receive_buffer, received_seq_no, receive_buffer);
        if (received_seq_no != seq_no) {
	    ROS_ERROR("* Error Received data out of order");
        }
        received_length = received_payload.size();
    }

    ep_lock.unlock();

    return LIBUSB_SUCCESS;
}

/**
 *
 * Odrive endpoint init
 * enumerate ODrive hardware 
 * @param serialNumber odrive serial number
 * @return ODRIVE_OK on success 
 *
 */
int odrive_endpoint::init(uint64_t serialNumber)		
{
    libusb_device ** usb_device_list;
    int ret = 1;

    ssize_t device_count = libusb_get_device_list(libusb_context_, &usb_device_list);
    if (device_count <= 0) {
        return device_count;
    }

    for (size_t i = 0; i < device_count; ++i) {
        libusb_device *device = usb_device_list[i];
        libusb_device_descriptor desc = {0};

        int result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            ROS_ERROR("* Error getting device descriptor");
            continue;
	}
	/* Check USB devicei ID */
        if (desc.idVendor == ODRIVE_USB_VENDORID && desc.idProduct == ODRIVE_USB_PRODUCTID) {

	    libusb_device_handle *device_handle;
            if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS) {
                ROS_ERROR("* Error opeening USB device");
                continue;
	    }

	    struct libusb_config_descriptor *config;
	    result = libusb_get_config_descriptor(device, 0, &config);
            int ifNumber = 2;//config->bNumInterfaces;

	    if ((libusb_kernel_driver_active(device_handle, ifNumber) != LIBUSB_SUCCESS) && 
		(libusb_detach_kernel_driver(device_handle, ifNumber) != LIBUSB_SUCCESS)) {
                ROS_ERROR("* Driver error");
		libusb_close(device_handle);
		continue;
	    }
	    
	    if ((result = libusb_claim_interface(device_handle, ifNumber)) !=  LIBUSB_SUCCESS) {
                ROS_ERROR("* Error claiming device");
                libusb_close(device_handle);
		continue;
            } else {
		bool attached_to_handle = false;
                unsigned char buf[128];

                result = libusb_get_string_descriptor_ascii(device_handle, desc.iSerialNumber, 
				buf, 127);
		if (result <= 0) {
                    ROS_ERROR("* Error getting data");
                    result = libusb_release_interface(device_handle, ifNumber);
		    libusb_close(device_handle);
                    continue;
		} else {
                    std::stringstream stream;
                    stream << uppercase << std::hex << serialNumber;
                    std::string sn(stream.str());

		    if (sn.compare(0, strlen((const char*)buf), (const char*)buf) == 0) {
                        ROS_INFO("Device 0x%8.8lX Found", serialNumber); 
                        odrive_handle_ = device_handle;
                        attached_to_handle = true;
			ret = ODRIVE_OK;
			break;
                    }		    
                }
                if (!attached_to_handle) {
                    result = libusb_release_interface(device_handle, ifNumber);
                    libusb_close(device_handle);
                }
            }
        }
    }

    libusb_free_device_list(usb_device_list, 1);

    return ret;
}

/**
 *
 * Odrive endpoint remove
 * close ODrive dvice
 *
 */
void odrive_endpoint::remove(void)
{
    if (odrive_handle_ != NULL) {
        libusb_release_interface(odrive_handle_, 2);
        libusb_close(odrive_handle_);
        odrive_handle_ = NULL;
    }
}

template int odrive_endpoint::getData(int, bool&);
template int odrive_endpoint::getData(int, short&);
template int odrive_endpoint::getData(int, int&);
template int odrive_endpoint::getData(int, float&);
template int odrive_endpoint::getData(int, uint8_t&);
template int odrive_endpoint::getData(int, uint16_t&);
template int odrive_endpoint::getData(int, uint32_t&);
template int odrive_endpoint::getData(int, uint64_t&);

template int odrive_endpoint::setData(int, const bool&);
template int odrive_endpoint::setData(int, const short&);
template int odrive_endpoint::setData(int, const int&);
template int odrive_endpoint::setData(int, const float&);
template int odrive_endpoint::setData(int, const uint8_t&);
template int odrive_endpoint::setData(int, const uint16_t&);
template int odrive_endpoint::setData(int, const uint32_t&);
template int odrive_endpoint::setData(int, const uint64_t&);


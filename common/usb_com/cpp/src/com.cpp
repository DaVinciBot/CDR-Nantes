#include <Arduino.h>
#include <com.h>
#include <crc.h>
#include <cstring>  // To use memcpy() / strlen()

/**
 * @brief Constructor for USB serial communication.
 *
 * @param stream Pointer to the USB serial class.
 * @param baudrate Baud rate for serial communication.
 */
Com::Com(usb_serial_class* stream, uint32_t baudrate) {
    this->stream = stream;
    stream->begin(baudrate);
}

/**
 * @brief Constructor for hardware serial communication.
 *
 * @param stream Pointer to the hardware serial class.
 * @param baudrate Baud rate for serial communication.
 */
Com::Com(HardwareSerial* stream, uint32_t baudrate) {
    this->stream = stream;
    stream->begin(baudrate);
}

void Com::reject_frame() {
    ++this->rx_rejected;
    byte invalid_msg = NACK;
    // is_nack = true: a NACK must never overwrite the stored message,
    // otherwise a NACK from the peer makes us resend a NACK instead of the
    // real payload.
    send_msg(&invalid_msg, 1, true);
}

/**
 * @brief Handles incoming data from the communication stream.
 *
 * Bytes are accumulated until a FRAME_DELIMITER closes the frame. The frame is
 * then COBS-decoded and its CRC8 checked. Since COBS keeps 0x00 out of the
 * encoded bytes, the delimiter always marks a frame boundary: after line noise
 * or a cut frame, the stream is back in sync at the next delimiter.
 *
 * @return The size of the valid message (ID + data), or 0 if no complete
 *     valid message arrived.
 */
byte Com::handle() {
    while (this->stream->available()) {
        const byte data = this->stream->read();

        if (data != FRAME_DELIMITER) {
            if (this->rx_length < sizeof(this->rx_encoded))
                this->rx_encoded[this->rx_length++] = data;
            else
                this->rx_overflow = true;  // dropped at its delimiter
            continue;
        }

        // End of a frame
        const uint16_t length = this->rx_length;
        const bool overflow = this->rx_overflow;
        this->rx_length = 0;
        this->rx_overflow = false;

        if (length == 0)
            continue;  // two delimiters in a row: nothing in between

        const int decoded =
            overflow ? -1
                     : cobs_decode(this->rx_encoded, length, this->buffer,
                                   sizeof(this->buffer));

        // At least the ID and the CRC
        if (decoded < 2) {
            reject_frame();
            continue;
        }

        const byte msg_size = static_cast<byte>(decoded - 1);
        CRC crc;
        if (crc.digest(this->buffer, msg_size) != this->buffer[msg_size]) {
            reject_frame();
            continue;
        }

        return msg_size;
    }
    return 0;
}

/**
 * @brief Handles callbacks for received messages.
 *
 * Processes received messages, retrieves the message ID, and calls the
 * corresponding callback function from the provided function array. Handles
 * unknown messages and retransmits the last message in case of NACK.
 *
 * @param functions Array of callback functions indexed by message ID.
 */
void Com::handle_callback(void (*functions[256])(byte* msg, byte size)) {
    // Retrieve the size of the received message
    byte size = this->handle();
    if (size > 0) {
        // Directly access the buffer pointer
        const byte* msg = this->read_buffer();
        if (msg == nullptr) {
            // Exit if the buffer is null (protection)
            return;
        }

        // Retrieve the message ID
        byte msg_id = msg[0];

        // Check if the function corresponding to the ID exists
        if (functions[msg_id] != nullptr) {
            functions[msg_id](const_cast<byte*>(msg),
                              size);  // Call the function
        } else if (msg_id == NACK) {
            // Resend the last message in case of NACK
            if (this->last_msg != nullptr && this->last_msg->size > 0) {
                this->send_msg((byte*)&this->last_msg->msg,
                               this->last_msg->size, true);
            }
        } else {
            // Handle unknown message types
            msg_unknown_msg_type error_message;
            error_message.type_id = msg_id;

            // Send a response indicating an unknown message type.
            // is_nack = true: this is an error report, not a payload worth
            // storing for retransmission.
            this->send_msg((byte*)&error_message, sizeof(msg_unknown_msg_type),
                           true);
        }
    }
}

/**
 * @brief Provides access to the internal buffer.
 *
 * @return Pointer to the last valid message: ID first, then the data.
 */
byte* Com::read_buffer() {
    return this->buffer;
}

/**
 * @brief Sends a message over the communication stream.
 *
 * Appends the CRC8, COBS-encodes, stores the message as the last message (if
 * not a NACK), and writes the frame followed by its delimiter. No allocation:
 * this runs at the rate of the telemetry.
 *
 * @param msg Pointer to the message data to send (ID first).
 * @param size The size of the message data (at most COM_MAX_PAYLOAD).
 * @param is_nack Indicates if the message is a retransmission due to a NACK.
 */
void Com::send_msg(byte* msg, byte size, bool is_nack) {
    if (size == 0)
        return;

    // Store the message for retransmission, unless this send is itself a
    // retransmission or an error report. Both `size` and `msg` must be kept
    // in sync: writing `size` alone would leave a stored message whose
    // content is stale or empty.
    if (!is_nack) {
        this->last_msg->size = size;
        memcpy(this->last_msg->msg, msg, size);
    }

    // [ID][data][CRC8]
    CRC crc;
    memcpy(this->_tx_raw, msg, size);
    this->_tx_raw[size] = crc.digest(msg, size);

    const size_t encoded = cobs_encode(this->_tx_raw, size + 1, this->_tx_encoded);

    this->stream->write(this->_tx_encoded, encoded);
    this->stream->write(FRAME_DELIMITER);
    this->stream->flush();
}

/**
 * @brief Sends a debug text message.
 *
 * This function sends a text message for debugging purposes. The message must
 * be in ASCII format; it is cut at 254 characters.
 *
 * @param text Pointer to the null-terminated ASCII text string to send.
 */
void Com::print(const char* text) {
    byte msg[COM_MAX_PAYLOAD];
    size_t text_len = strlen(text);
    if (text_len > COM_MAX_PAYLOAD - 1)
        text_len = COM_MAX_PAYLOAD - 1;

    msg[0] = PRINT;
    memcpy(msg + 1, text, text_len);
    this->send_msg(msg, static_cast<byte>(text_len + 1));
}

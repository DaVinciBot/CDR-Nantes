#include <Arduino.h>
#include <com.h>
#include <crc.h>
#include <cstring>  // To use memcpy()

/**
 * @brief Constructor for USB serial communication.
 *
 * Initializes the communication stream for USB serial and sets up the internal
 * buffer.
 *
 * @param stream Pointer to the USB serial class.
 * @param baudrate Baud rate for serial communication.
 */
Com::Com(usb_serial_class* stream, uint32_t baudrate) {
    memcpy(this->signature, END_BYTES_SIGNATURE, sizeof(this->signature));

    this->stream = stream;
    stream->begin(baudrate);

    // Initialize the buffer
    for (uint16_t k = 0; k < 256; k++)
        this->buffer[k] = 0;
}

/**
 * @brief Constructor for hardware serial communication.
 *
 * Initializes the communication stream for hardware serial and sets up the
 * internal buffer.
 *
 * @param stream Pointer to the hardware serial class.
 * @param baudrate Baud rate for serial communication.
 */
Com::Com(HardwareSerial* stream, uint32_t baudrate) {
    // Without this memcpy, `signature` stays uninitialized: outgoing frames
    // carry a garbage signature and are never recognized by the peer.
    memcpy(this->signature, END_BYTES_SIGNATURE, sizeof(this->signature));

    this->stream = stream;
    stream->begin(baudrate);

    // Initialize the buffer
    for (uint16_t k = 0; k < 256; k++)
        this->buffer[k] = 0;
}

/**
 * @brief Handles incoming data from the communication stream.
 *
 * Processes incoming data byte by byte, checks for valid messages based on the
 * signature and CRC, and extracts the message size if a valid message is
 * detected.
 *
 * @return The size of the valid message, or 0 if no valid message is found.
 */
byte Com::handle() {
    while (this->stream->available()) {
        byte data = this->stream->read();
        this->buffer[this->pointer++] = data;

        // Wait until at least 6 bytes are received
        if (this->pointer < 6)
            continue;

        // Check for signature validity
        bool is_signature = true;
        for (int i = 0; i < 4 && is_signature; i++)
            is_signature =
                this->buffer[pointer - 1 - i] == this->signature[3 - i];

        if (!is_signature)
            continue;

        // Extract message size
        byte msg_size = this->buffer[pointer - 6];

        // The size byte is located relative to the END of the frame, but the
        // CRC below is checked relative to the START of the buffer. Those two
        // conventions only agree when the frame begins exactly at index 0, so
        // realign it first: any leading noise (line garbage, tail of a frame
        // that was rejected earlier) would otherwise make the CRC read the
        // wrong bytes and NACK a perfectly valid frame.
        int frame_start = (int)this->pointer - 6 - (int)msg_size;

        if (frame_start >= 0) {
            if (frame_start > 0) {
                // Move [msg | size | crc] to the start of the buffer
                memmove(this->buffer, this->buffer + frame_start, msg_size + 2);
            }

            CRC crc;
            byte crc_b = crc.digest(this->buffer, msg_size + 1);

            // Validate CRC
            if (crc_b != this->buffer[msg_size + 1]) {
                byte invalid_crc_msg = NACK;
                // is_nack = true: a NACK must never overwrite the stored
                // message, otherwise a NACK from the peer makes us resend
                // a NACK instead of the real payload.
                send_msg(&invalid_crc_msg, 1, true);
                this->pointer = 0;
                continue;
            }

            // Reset the pointer and return the message size
            this->pointer = 0;
            return msg_size;
        } else {
            this->pointer = 0;
        }
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
            if (this->last_msg != nullptr) {
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
 * @return Pointer to the internal buffer.
 */
byte* Com::read_buffer() {
    return this->buffer;
}

/**
 * @brief Sends a message over the communication stream.
 *
 * Prepares the message with size and CRC, stores it as the last message (if not
 * a NACK), and sends it over the stream.
 *
 * @param msg Pointer to the message data to send.
 * @param size The size of the message data.
 * @param is_nack Indicates if the message is a retransmission due to a NACK.
 */
void Com::send_msg(byte* msg, byte size, bool is_nack) {
    CRC crc;

    // Prepare the full message with size, used only to compute the CRC.
    // `_crc_buffer` is a fixed member: no heap allocation on the send path,
    // which runs at ~100 Hz.
    for (byte i = 0; i < size; i++)
        this->_crc_buffer[i] = msg[i];
    this->_crc_buffer[size] = size;

    // Store the message for retransmission, unless this send is itself a
    // retransmission or an error report. Both `size` and `msg` must be kept
    // in sync: writing `size` alone would leave a stored message whose
    // content is stale or empty.
    if (!is_nack) {
        this->last_msg->size = size;
        for (byte i = 0; i < size; i++)
            this->last_msg->msg[i] = msg[i];
    }

    // Compute CRC
    byte crc_b = crc.digest(this->_crc_buffer, size + 1);

    // Send the message
    this->stream->write(msg, size);
    this->stream->write(size);
    this->stream->write(crc_b);
    this->stream->write(this->signature, 4);
    this->stream->flush();
}

/**
 * @brief Sends a debug text message.
 *
 * This function sends a text message for debugging purposes. The message must
 * be in ASCII format and cannot exceed 253 characters.
 *
 * @param text Pointer to the null-terminated ASCII text string to send.
 */
void Com::print(char* text) {
    // Use send_msg to send the text input
    size_t text_len = strlen(text);
    byte* msg = new byte[text_len + 2];
    msg[0] = PRINT;
    for (size_t i = 0; i <= text_len; i++) {
        msg[i + 1] = (byte)text[i];
    }
    this->send_msg(msg, text_len + 1);
    delete[] msg;
}

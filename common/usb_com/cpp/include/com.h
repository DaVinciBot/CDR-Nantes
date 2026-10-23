#ifndef COM_H
#define COM_H

#include <Arduino.h>
#include <cobs.h>
#include <messages.h>

// Decoded frame: ID + data + CRC8.
const uint16_t COM_MAX_DECODED = COM_MAX_PAYLOAD + 1;
// Encoded frame, delimiter excluded.
const uint16_t COM_MAX_ENCODED = cobs_max_encoded_size(COM_MAX_DECODED);

/**
 * @brief Structure to store the last sent message.
 *
 * This structure keeps track of the size and content of the last message
 * sent by the communication system. It is used to handle retransmissions
 * in case of a NACK (Negative Acknowledgment).
 */
struct last_message {
    byte size = 0;        ///< Size of the message in bytes
    byte msg[256] = {0};  ///< Content of the message
};

/**
 * @brief Communication (Com) class for serial communication.
 *
 * Frames are COBS([ID][data][CRC8]) followed by a 0x00 delimiter (see
 * messages.h). Incoming bytes are accumulated across calls until a delimiter
 * closes the frame, which is then decoded and checked; a bad frame is answered
 * with a NACK and counted. The class also handles callbacks and retransmission
 * of the last message on a NACK.
 */
class Com {
   private:
    Stream* stream;  ///< Pointer to the serial stream (USB or Hardware)

    /// Encoded bytes of the frame being received, kept between calls of
    /// handle(): a frame may arrive over several of them.
    byte rx_encoded[COM_MAX_ENCODED];
    uint16_t rx_length = 0;
    /// The frame being received did not fit: it is dropped at its delimiter.
    bool rx_overflow = false;
    /// Frames rejected since boot (bad COBS, bad CRC, too long, too short).
    uint32_t rx_rejected = 0;

    /// Last decoded frame: [ID][data][CRC8]; read_buffer() points here.
    byte buffer[COM_MAX_DECODED];

    /// Static storage for the last sent message. Previously `last_msg` was
    /// reallocated with `new` on every `send_msg()` and released with `free()`
    /// (undefined behaviour), i.e. dynamic allocation at ~100 Hz on the Teensy.
    last_message _last_msg_storage;

    /// Scratch buffers of the send path, so that it never allocates.
    byte _tx_raw[COM_MAX_DECODED];
    byte _tx_encoded[COM_MAX_ENCODED];

    /// Counts a bad frame and answers it with a NACK.
    void reject_frame();

   public:
    /**
     * @brief Constructor for USB serial communication.
     *
     * @param stream Pointer to the USB serial class.
     * @param baudrate Baud rate for serial communication.
     */
    Com(usb_serial_class* stream, uint32_t baudrate);

    /**
     * @brief Constructor for hardware serial communication.
     *
     * @param stream Pointer to the hardware serial class.
     * @param baudrate Baud rate for serial communication.
     */
    Com(HardwareSerial* stream, uint32_t baudrate);

    /**
     * @brief Handles incoming data from the communication stream.
     *
     * Reads what is available, up to the end of the first complete frame;
     * an incomplete frame is kept for the next call.
     *
     * @return The size of the valid message (ID + data), or 0 if no complete
     *     valid message arrived.
     */
    byte handle();

    /**
     * @brief Handles callbacks for received messages.
     *
     * Retrieves the message ID from the received message and calls the
     * corresponding callback function from the provided array of function
     * pointers.
     *
     * @param functions Array of callback functions indexed by message ID.
     */
    void handle_callback(void (*functions[256])(byte* msg, byte size));

    /**
     * @brief Provides access to the internal buffer.
     *
     * @return Pointer to the last valid message: ID first, then the data.
     */
    byte* read_buffer();

    /**
     * @brief Sends a message over the communication stream.
     *
     * Appends the CRC8, COBS-encodes, and writes the frame and its delimiter.
     *
     * @param msg Pointer to the message data to send (ID first).
     * @param size Size of the message data (at most COM_MAX_PAYLOAD).
     * @param is_nack Indicates if the message is a retransmission due to a NACK
     * (default: false).
     */
    void send_msg(byte* msg, byte size, bool is_nack = false);

    /**
     * @brief Sends a debug text message.
     *
     * This function sends a text message for debugging purposes. The message
     * must be in ASCII format; it is cut at 254 characters.
     *
     * @param text Pointer to the null-terminated ASCII text string to send.
     */
    void print(const char* text);

    /** @return Frames rejected since boot. */
    uint32_t rejected_frames() const { return rx_rejected; }

    last_message* last_msg =
        &_last_msg_storage;  ///< Pointer to the last sent message for
                             ///< retransmission (static storage, never
                             ///< reallocated)
};

#endif // COM_H

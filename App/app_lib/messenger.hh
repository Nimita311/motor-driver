/**
 * @file     messenger.hh
 * @brief    Messaging helper class. Wrapper for nanopb library.
 * @author   Haoze Zhang
 * @version  20200211
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#ifndef INC_MESSENGER_HH_
#define INC_MESSENGER_HH_

#include "pb.h"
#include "pb_encode.h"

namespace brown {

class Messenger {
private:
    // Preamble and delimiter of the message. Simple packeting scheme.
    const char* PREAMBLE = "UUUAT";
    size_t PREAMBLE_SIZE = 5;
    const char* DELIMITER = "\0\0\0AT";
    size_t DELIMITER_SIZE = 5;

    // Buffer for encoded message.
    uint8_t* buffer = NULL;
    size_t bufferSize = 0;

    // Bytes to write to physical interface.
    // Writes should be blocked if not zero.
    volatile size_t isSending = false;

    // Callbacks that initiate hardware transmission.
    // Character-based transmission, assume blocking.
    void (*putchar) (char) = NULL;
    // Block-based (DMA) transmission transmission, assume non-blocking.
    // Need to call `sendCompleteCallback` to indicate completion.
    void (*putblock) (char*, size_t) = NULL; //

    bool _writePacketToStream(pb_ostream_t* pStream,
            const pb_msgdesc_t* pFields, const void* pMessage) {
        return (pb_write(pStream, (pb_byte_t*)PREAMBLE, PREAMBLE_SIZE)
             && pb_encode(pStream, pFields, pMessage)
             && pb_write(pStream, (pb_byte_t*)DELIMITER, DELIMITER_SIZE));
    }

    static bool _putcharStreamCallback(
        pb_ostream_t *stream, const pb_byte_t *buf, size_t count);

public:
    /**
     * @brief Constructor for block-based transmission.
     * @param buffer Buffer for encoded message.
     * @param bufferSize Size of the buffer.
     * @param putblock Callback function to transmit a block of characters
     * via the hardware interface.
     */
    Messenger(uint8_t* buffer, uint16_t bufferSize,
            void (*putblock) (char*, size_t)):
        buffer(buffer), bufferSize(bufferSize), putblock(putblock) {}

    /**
     * @brief Constructor for character-based transmission.
     * @param putchar Callback function to transmit a charater via the
     * hardware interface.
     *
     * This form should be used only for debugging or memory critical cases.
     * It does not require a buffer.
     */
    Messenger(void (*putchar) (char)): putchar(putchar) {}

    /**
     * @brief Encode the message to the buffer and send it as a whole block.
     * @return bool Success flag.
     */
    bool sendPerBlock(const pb_msgdesc_t* pFields, const void* pMessage) {
        if (isSending || putblock == NULL) {return false;}

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);
        bool success = _writePacketToStream(&stream, pFields, pMessage);
        isSending = true;
        putblock(reinterpret_cast<char*>(buffer), stream.bytes_written);

        return success;
    }

    /**
     * @brief Send a message while encoding it.
     * @return bool Success flag.
     */
    bool sendPerChar(const pb_msgdesc_t* pFields, const void* pMessage) {
        if (isSending || putchar == NULL) {return false;}

        pb_ostream_t stream = {0};
        stream.max_size = SIZE_MAX;
        stream.state = reinterpret_cast<void*>(putchar);
        stream.callback = Messenger::_putcharStreamCallback;
        isSending = true;
        bool success = _writePacketToStream(&stream, pFields, pMessage);

        sendCompleteCallback();
        return success;
    }

    /**
     * @brief Signify the transmission is complete.
     *
     * This method should be called in the hardware transmission complete ISR.
     */
    void sendCompleteCallback() {
        isSending = 0;
    }
}; // class Messenger

} // namespace brown

#endif /* INC_MESSENGER_HH_ */

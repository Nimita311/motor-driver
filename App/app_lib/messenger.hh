/**
 * @file     messenger.hh
 * @brief    Messaging helper classes `Sender` and `Receiver`.
 *           Wrapper for nanopb library.
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
#include "pb_decode.h"

namespace brown {

class Sender {
private:
    uint8_t* buffer = nullptr;
    size_t bufferSize = 0;

    // Bytes to write to physical interface.
    // Writes should be blocked if not zero.
    volatile size_t isSending = false;

    // Callbacks that initiate hardware transmission.
    // Character-based transmission, assume blocking.
    void (*putchar) (char) = nullptr;
    // Block-based (DMA) transmission transmission, assume non-blocking.
    // Need to call `sendCompleteCallback` to indicate completion.
    void (*putblock) (char*, size_t) = nullptr; //

    bool _writePacketToStream(pb_ostream_t* pStream,
        const pb_msgdesc_t* pFields, const void* pMessage);

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
    Sender(uint8_t* buffer, uint16_t bufferSize,
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
    Sender(void (*putchar) (char)): putchar(putchar) {}

    /**
     * @brief Encode and send the message.
     * @return bool Success flag.
     */
    bool send(const pb_msgdesc_t* pFields, const void* pMessage);

    /**
     * @brief Signify the transmission is complete.
     *
     * This method should be called in the hardware transmission complete ISR.
     */
    void sendCompleteCallback() {
        isSending = 0;
    }
}; // class Sender

class Receiver {
private:
    uint8_t* pRawBuffer = nullptr;
    size_t rawBufferSize = 0;

    uint8_t* pPktBuffer = nullptr;
    size_t pktBufferSize = 0;

    size_t headIdx = 0;
    volatile size_t tailIdx = 0;

    uint32_t count = 0;
    uint32_t errCount = 0;

public:
    /**
     * @param pRawBuffer Buffer for DMA circular write. Need to be large.
     * @param pPktBuffer Buffer to hold a valid packet.
     */
    Receiver(uint8_t* pRawBuffer, size_t rawBufferSize,
             uint8_t* pPktBuffer, size_t pktBufferSize):
        pRawBuffer(pRawBuffer), rawBufferSize(rawBufferSize),
        pPktBuffer(pPktBuffer), pktBufferSize(pktBufferSize) {}

    /**
     * @brief Decode an arrived packet.
     * @param pFields Pointer to Nanopb message fields.
     * @param pMessage Pointer to Nanopb message struct.
     */
    bool receive(const pb_msgdesc_t* pFields, void* pMessage);

    /**
     * @brief Update message tail location.
     * @param dataLength Distance to the end of the raw buffer. With STM32
     * DMA this is the NDT value in NDTR.
     *
     * This method should be called in UART Rx timeout ISR.
     */
    void receiveCompleteCallback(size_t dataLength);

}; // class Receiver

} // namespace brown

#endif /* INC_MESSENGER_HH_ */

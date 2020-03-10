/**
 * @file     messenger.cc
 * @brief    `Sender` and `Receiver` class implementation.
 * @author   Haoze Zhang
 * @version  20200301
 *
 * Distributed under MIT License
 * Copyright (c) 2020 Haoze Zhang | Brown Engineering
 */

#include "messenger.hh"

namespace brown {

// Preamble and delimiter of the message. Simple packeting scheme.
static constexpr const char* PREAMBLE = "UUUAT";
static constexpr const size_t PREAMBLE_SIZE = 5;
static constexpr const char* DELIMITER = "\0\0\0AT";
static constexpr const size_t DELIMITER_SIZE = 5;

bool Sender::_writePacketToStream(pb_ostream_t* pStream,
        const pb_msgdesc_t* pFields, const void* pMessage) {
    return (pb_write(pStream, (pb_byte_t*)PREAMBLE, PREAMBLE_SIZE)
            && pb_encode(pStream, pFields, pMessage)
            && pb_write(pStream, (pb_byte_t*)DELIMITER, DELIMITER_SIZE));
}

bool Sender::_putcharStreamCallback(
        pb_ostream_t *stream, const pb_byte_t *buf, ::size_t count) {
    void (*putchar) (char) = reinterpret_cast<void (*) (char)>(stream->state);
    if (putchar == NULL) {return false;}
    for (::size_t i = 0; i < count; i++) {
        putchar(buf[i]);
    }
    return true;
}

bool Sender::send(const pb_msgdesc_t* pFields, const void* pMessage) {
    if (isSending) {return false;}
    if (putblock != nullptr) { // block based
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);
        bool success = _writePacketToStream(&stream, pFields, pMessage);
        isSending = true;
        putblock(reinterpret_cast<char*>(buffer), stream.bytes_written);
    } else if (putchar != nullptr) { // char based
        pb_ostream_t stream = {0};
        stream.max_size = SIZE_MAX;
        stream.state = reinterpret_cast<void*>(putchar);
        stream.callback = Sender::_putcharStreamCallback;
        isSending = true;
        bool success = _writePacketToStream(&stream, pFields, pMessage);
        sendCompleteCallback();
        return success;
    } else { // No hardware callback
        return false;
    }
}

bool Receiver::receive(const pb_msgdesc_t* pFields, void* pMessage) {
    // Save the current tail location in case a new transmission completes
    // during the process.
    size_t tailIdx = this->tailIdx;
    // Packet size.
    size_t pktSize = 0;

    /* 1. Copy to the packet buffer in case of overwriting. */
    if (headIdx < tailIdx) { // Continuous segment
        pktSize = tailIdx-headIdx;
        memcpy(pPktBuffer, pRawBuffer+headIdx, pktSize);
    } else if (headIdx > tailIdx) { // Wrapped
        size_t fstSegSize = rawBufferSize-headIdx;
        pktSize = fstSegSize + tailIdx;
        // headIdx to end
        memcpy(pPktBuffer, pRawBuffer+headIdx, fstSegSize);
        // start to tailIdx
        memcpy(pPktBuffer+fstSegSize, pRawBuffer, tailIdx);
    } else { // (headIdx == tailIdx) No valid data
        errCount++;
        return false;
    }

    /* 2. Validate packet. */
    // Not enough data
    if (pktSize < PREAMBLE_SIZE) {
        headIdx = tailIdx;
        errCount++;
        return false;
    }
    // No valid start
    if (memcmp(pPktBuffer, PREAMBLE, PREAMBLE_SIZE) != 0) {
        headIdx = tailIdx;
        errCount++;
        return false;
    }
    // No valid end. Do not reset head, the transmission may be unfinished.
    uint8_t* pDelimiter = pPktBuffer+pktSize-DELIMITER_SIZE;
    if (memcmp(pDelimiter, DELIMITER, DELIMITER_SIZE) != 0) {
        errCount++;
        return false;
    }

    /* 3. Decode message. */
    uint8_t* pMsg = pPktBuffer+PREAMBLE_SIZE;
    size_t msgSize = pktSize - PREAMBLE_SIZE - DELIMITER_SIZE;
    pb_istream_t istream = pb_istream_from_buffer(pMsg, msgSize);
    // Not valid encoding
    if (!pb_decode(&istream, pFields, pMessage)) {
        errCount++;
        headIdx = tailIdx;
        return false;
    }

    /* 4. Valid message received. */
    headIdx = tailIdx;
    count++;
    return true;
}

void Receiver::receiveCompleteCallback(size_t dataLength) {
    // Calculate message tail index based on DMA remaining data length
    tailIdx = rawBufferSize-dataLength;
}

} // namespace brown

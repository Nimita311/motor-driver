#include "messenger.hh"

namespace brown {

bool Messenger::_putcharStreamCallback(
        pb_ostream_t *stream, const pb_byte_t *buf, size_t count) {
    void (*putchar) (char) = reinterpret_cast<void (*) (char)>(stream->state);
    if (putchar == NULL) {return false;}
    for (size_t i = 0; i < count; i++) {
        putchar(buf[i]);
    }
    return true;
}

} // namespace brown

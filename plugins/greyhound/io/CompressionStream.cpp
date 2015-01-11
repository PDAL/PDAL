#include <stdexcept>
#include <cstring>

#include "CompressionStream.hpp"

CompressionStream::CompressionStream()
    : m_data()
    , m_index(0)
    , m_mutex()
    , m_cv()
{ }

void CompressionStream::putBytes(const uint8_t* bytes, const std::size_t length)
{
    std::unique_lock<std::mutex> lock(m_mutex);

    const std::size_t offset(m_data.size());
    m_data.resize(m_data.size() + length);
    std::memcpy(m_data.data() + offset, bytes, length);

    lock.unlock();
    m_cv.notify_all();
}

uint8_t CompressionStream::getByte()
{
    // Make sure we have enough data to hand out before continuing.
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this]()->bool { return m_index < m_data.size(); });
    return m_data.at(m_index++);
}

void CompressionStream::getBytes(uint8_t* bytes, std::size_t length)
{
    std::unique_lock<std::mutex> lock(m_mutex);

    // Make sure we have enough data to hand out before continuing.
    m_cv.wait(lock, [this, length]()->bool {
        return m_index + length <= m_data.size();
    });

    std::memcpy(bytes, m_data.data() + m_index, length);
    m_index += length;
}


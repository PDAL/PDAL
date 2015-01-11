#pragma once

#include <cstdint>
#include <vector>
#include <mutex>
#include <condition_variable>

class CompressionStream
{
public:
    CompressionStream();

    // Called by us as we receive data.
    void putBytes(const uint8_t* bytes, std::size_t length);

    // Called by laz-perf as it decompresses.
    uint8_t getByte();
    void getBytes(uint8_t* bytes, std::size_t length);

private:
    std::vector<uint8_t> m_data;
    std::size_t m_index;

    std::mutex m_mutex;
    std::condition_variable m_cv;
};


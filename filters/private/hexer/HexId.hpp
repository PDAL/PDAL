#pragma once

#include <h3api.h>

namespace hexer
{

struct HexId : public CoordIJ
{};

inline bool operator<(HexId const& c1, HexId const& c2)
{
    return (c1.i < c2.i) || ((c1.i == c2.i) && (c1.j < c2.j));
}

inline bool operator == (const HexId& h1, const HexId& h2)
{
    return h1.i == h2.i && h1.j == h2.j;
}

inline HexId operator+(HexId const& c1, HexId const& c2)
    {   return {c1.i + c2.i, c1.j + c2.j};  }
} // namespace hexer

namespace std
{
    template<>
    struct hash<hexer::HexId>
    {
        std::size_t operator()(hexer::HexId const & id) const noexcept
        {
            hash<uint32_t> h;

            return h(id.i) ^ (h(id.j) << 1);
        }
    };
} // namespace std

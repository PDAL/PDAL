#pragma once

#include <plugins/nitf/io/NitfFileWriter.hpp>

namespace pdal
{
namespace nitfwrap
{

class error : public std::runtime_error
{
public:
    error(const std::string& msg) : std::runtime_error(msg)
        {}
};

class NitfWrap
{
public:
    NitfWrap(std::vector<std::string>& args);

private:
    std::string m_inputFile;
    std::string m_outputFile;
    NitfFileWriter m_nitfWriter;
    bool m_unwrap;

    bool parseArgs(std::vector<std::string>& args);
    void unwrap();
    void verify(BOX3D& bounds);
    bool verifyLas(ILeStream& in, BOX3D& bounds, bool& compressed);
    bool verifyBpf(ILeStream& in, BOX3D& bounds);
};

} // namespace nitfwrap
} // namespace pdal


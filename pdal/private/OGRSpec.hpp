#pragma once

#include <vector>

#include <pdal/Polygon.hpp>
#include <pdal/pdal_types.hpp>

#include <nlohmann/json.hpp>

namespace pdal
{

struct OGRSpecOptions
{
    std::string datasource;
    std::string layer;
    std::string sql;
    std::string dialect;
    std::string geometry;
    std::vector<std::string> drivers;
    std::vector<std::string> openOpts;
};

class OGRSpec
{
    friend std::ostream& operator<<(std::ostream& out,
        const OGRSpec& bounds);

public:
    OGRSpec(const std::string& ogrJsonStr);
    OGRSpec(const NL::json& ogrJson);
    OGRSpec();

    void update(const std::string& ogrJsonStr);
    void update(const NL::json& ogrJson);

    std::vector<Polygon> getPolygons()
    { return m_geom; }
    bool empty()
    { return m_geom.empty(); }
    // test function
    int size()
    { return m_geom.size(); }

    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

private:
    void initialize();
    void validateInput(const std::string& ogrJsonStr);
    void validateInput(const NL::json& ogrJson);
    void parse();

    template <typename T>
    void assignJSON(const NL::json& field, T& to)
    {
        try
        {
            field.get_to(to);
        }
        catch(const NL::json::exception& e)
        {
            std::string s(e.what());
            auto pos = s.find("]");
            if (pos != std::string::npos)
                s = s.substr(pos + 1);
            std::stringstream msg;

            msg << "Failed to parse JSON field with error: " << s;
            throw error(msg.str());
        }
    }

    std::vector<Polygon> m_geom;
    // m_json mostly only exists for operator<< output
    NL::json m_json;
    OGRSpecOptions m_opts;
};

namespace Utils
{

    template<>
    inline StatusWithReason fromString(const std::string& from, OGRSpec& to)
    {
        try
        {
            to.update(from);
        }
        catch (OGRSpec::error& e)
        {
            return StatusWithReason(-1, e.what());
        }
        return true;
    }

} // nampespace utils

} // namespace pdal

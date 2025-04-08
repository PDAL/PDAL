#include "OGRSpec.hpp"
#include <pdal/private/gdal/GDALUtils.hpp>

#include <nlohmann/json.hpp>

namespace pdal
{

OGRSpec::OGRSpec()
{}


OGRSpec::OGRSpec(const std::string& ogrJsonStr)
{
    validateInput(ogrJsonStr);
    initialize();
}

OGRSpec::OGRSpec(const nlohmann::json& ogrJson)
{
    validateInput(ogrJson);
    initialize();
}

void OGRSpec::update(const std::string& ogrJsonStr)
{
    m_geom.clear();
    validateInput(ogrJsonStr);
    initialize();
}

void OGRSpec::update(const nlohmann::json& ogrJson)
{
    m_geom.clear();
    validateInput(ogrJson);
    initialize();
}

void OGRSpec::validateInput(const std::string& ogrJsonStr)
{
    try
    {
        m_json = nlohmann::json::parse(ogrJsonStr);
    }
    catch(nlohmann::json::parse_error& e)
    {
        std::string s(e.what());
        auto pos = s.find("]");
        if (pos != std::string::npos)
            s = s.substr(pos + 1);
        std::stringstream msg;

        msg << "Failed to parse OGR JSON with error: " << s;
        throw error(msg.str());
    }
    parse();
}

void OGRSpec::validateInput(const nlohmann::json& ogrJson)
{
    m_json = ogrJson;
    parse();
}

void OGRSpec::parse()
{
    // "type" field name is case sensitive, value is not
    if (!(m_json.is_object() && m_json.contains("type")))
        throw error("'ogr' option must be a JSON object with 'type':'ogr' specified!");
    else if (!(Utils::tolower(m_json.at("type").get<std::string>()) == "ogr"))
        throw error("'ogr' option must have 'type':'ogr' specified!");

    m_opts = {};
    for (auto& item : m_json.items())
    {
        std::string key = Utils::tolower(item.key());

        if (item.value().is_null() || (item.value() == ""))
        {
            std::stringstream out;
            out << "invalid value for field '" << key << "' in OGR JSON!";
            throw error(out.str());
        }
        if (key == "datasource")
            assignJSON(item.value(), m_opts.datasource);
        else if (key == "drivers")
            assignJSON(item.value(), m_opts.drivers);
        else if (key == "openoptions")
            assignJSON(item.value(), m_opts.openOpts);
        else if (key == "layer")
            assignJSON(item.value(), m_opts.layer);
        else if (key == "sql")
            assignJSON(item.value(), m_opts.sql);
        else if (key == "options")
        {
            for (auto optItem : item.value().items())
            {
                std::string optKey = Utils::tolower(optItem.key());
                if (optKey == "dialect")
                    assignJSON(optItem.value(), m_opts.dialect);
                else if (optKey == "geometry")
                    assignJSON(optItem.value(), m_opts.geometry);
                else
                    throw error("invalid value for 'options' field in OGR JSON!");
            }
        }
        else if (key == "type")
            continue;
        else
        {
            std::stringstream out;
            out << "unexpected field '" << key << "' in OGR JSON!";
            throw error(out.str());
        }
    }

    if (m_opts.datasource.empty())
        throw error("'ogr' option must contain a 'datasource' field!");
}

void OGRSpec::initialize()
{
    m_geom = gdal::getPolygons(m_opts);
}

std::ostream& operator << (std::ostream& out, const OGRSpec& ogr)
{
    out << ogr.m_json;
    return out;
}

} // namespace pdal

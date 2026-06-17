
#include <kernels/private/stac/StacInfo.hpp>
#include "TIndexProcessor.hpp"

namespace pdal
{
namespace tindex
{

struct Field;

static const std::string STAC_VERSION = "1.1.0";

struct StacFileInfo : FileInfo
{
    StacFileInfo(const std::string& filename) : FileInfo(filename), m_count(0)
    {
        m_encoding = FileUtils::extension(filename);
        MetadataNode self = m_root.addList("links");
        self.add("rel", "derived_from");
        self.add("href", filename);
    }

    void addMetadata(MetadataNode& readerMeta, MetadataNode& statsMeta, MetadataNode& schema)
    {
        auto schemas = schema.findChildren([](MetadataNode& n)
            { return n.name()=="dimensions"; });
        for (auto& s : schemas)
            m_root.addList(s.clone("pc:schemas"));

        // stageMetadata will always have reader metadata, but may not have stats.
        if (!statsMeta.empty())
        {
            auto props = statsMeta.findChildren([](MetadataNode& n)
                { return n.name()=="statistic"; });
            for (auto& p : props)
                m_root.addList(p.clone("pc:statistics"));
        }

        // get LAS/COPC/EPT point count

        // EPT
        MetadataNode points = readerMeta.findChild("points");
        // LAS/COPC
        MetadataNode count = readerMeta.findChild("count");
        if (!points.empty())
            m_count = points.value<point_count_t>();
        else if (!count.empty())
            m_count = count.value<point_count_t>();
    
        addDatetime(m_root, readerMeta);
    }

    std::string schemas()
    {
        return jsonElement(m_root.children("pc:schemas"));
    }
    
    std::string statistics()
    {
        return jsonElement(m_root.children("pc:statistics"));
    }

    std::string links()
    {
        return jsonElement(m_root.children("links"));
    }

    std::string datetime()
    {
        return getChild(m_root, "datetime").value();
    }

    int count()
        { return m_count; }

    std::string encoding()
        { return m_encoding; }

    StringList extensions() const
        { return m_extensions; }

private:
    std::string jsonElement(MetadataNodeList nodeList)
    {
        MetadataNode root(MetadataType::Array);
        for (auto n: nodeList)
            root.add(n);
        std::string jsonStr = Utils::toJSON(root);
        jsonStr.erase(std::remove_if(jsonStr.begin(), jsonStr.end(), 
            [](char c) { return c == '\n' || c == '\r'; }), jsonStr.end());

        return jsonStr;
    }

    MetadataNode m_root;

    StringList m_extensions;
    std::string m_encoding;
    point_count_t m_count;
};

class StacIndexBuilder : public TIndexProcessor
{
public:
    StacIndexBuilder(const Args& args, const std::string& pcType,
        bool statistics);

private:
    FileInfoPtr makeFileInfo(const std::string& filename) override;
    void fillFileInfo(FileInfoPtr& fileInfo) override;
    bool fastBoundary(PipelineManager& manager, FileInfo& fileInfo) override;
    void createExtraFields(const FileInfoPtr& fileInfo,
        Feature& feature) override;

    StringList m_extensions;
    std::string m_pcType;
    bool m_writeStats;
    Field *m_srsField;
    Field *m_datetimeField;
    Field *m_linksField;
    Field *m_idField;
    Field *m_stacExtensionsField;
    Field *m_stacVersionField;
    Field *m_pcCountField;
    Field *m_pcEncodingField;
    Field *m_pcTypeField;
    Field *m_pcSchemasField;
    Field *m_pcStatsField;
};

} // namespace tindex
} // namespace pdal

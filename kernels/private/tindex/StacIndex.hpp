
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
    StacFileInfo(const std::string& filename) : FileInfo(filename)
    {
        // stacProjection & stacPointCloud need these set in the root metadata
        // for it to work. Should refactor to be less metadata dependent
        m_root.add("filename", filename);
        MetadataNode self = m_root.addList("links");
        self.add("rel", "derived_from");
        self.add("href", filename);
        m_properties = m_root.add("properties");
        m_extensions = { "https://stac-extensions.github.io/projection/v1.1.0/",
            "https://stac-extensions.github.io/pointcloud/v1.0.0/" };
    }

    void addMetadata(MetadataNode& statsMeta, MetadataNode& readerMeta,
        MetadataNode& infoMeta, std::string pcType)
    {
        stacPointcloud(m_root, statsMeta, infoMeta, m_properties, pcType);
    }

    std::string schemas()
    {
        return jsonElement(m_properties.children("pc:schemas"));
    }
    
    std::string statistics()
    {
        return jsonElement(m_properties.children("pc:statistics"));
    }

    int count()
    {
        return getChild(m_properties, "pc:count").value<int>();
    }

    std::string encoding()
    {
        return getChild(m_properties, "pc:encoding").value();
    }

    std::string links()
    {
        return jsonElement(m_root.children("links"));
    }

    std::string datetime()
    {
        return getChild(m_properties, "datetime").value();
    }

    StringList extensions() const { return m_extensions; }

private:
    std::string jsonElement(MetadataNodeList node)
    {
        std::string jsonStr = Utils::toJSON(node);
        jsonStr.erase(std::remove_if(jsonStr.begin(), jsonStr.end(), 
            [](char c) { return c == '\n' || c == '\r'; }), jsonStr.end());

        return jsonStr;
    }
    MetadataNode m_root;
    MetadataNode m_properties;
    StringList m_extensions;
};

class StacIndexBuilder : public TIndexProcessor
{
public:
    StacIndexBuilder(const Args& args, const std::string& pcType);

private:
    FileInfoPtr makeFileInfo(const std::string& filename) override;
    void getFileInfo(FileInfoPtr& fileInfo) override;
    void createExtraFields(const FileInfoPtr& fileInfo,
        Feature& feature) override;

    StringList m_extensions;
    std::string m_pcType;
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

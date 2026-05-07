
#include <kernels/private/stac/StacInfo.hpp>
#include "TIndexBuilder.hpp"

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

    MetadataNode propertiesChild(std::string key)
    {
        return getChild(m_properties, key);
    }

    MetadataNodeList propertiesChildren(std::string key)
    {
        return m_properties.children(key);
    }

    MetadataNode rootChild(std::string key)
    {
        return getChild(m_root, key);
    }

    MetadataNodeList rootChildren(std::string key)
    {
        return m_root.children(key);
    }

    StringList extensions() const { return m_extensions; }

private:
    MetadataNode m_root;
    MetadataNode m_properties;
    StringList m_extensions;
};

class StacIndexBuilder : public TIndexBuilder
{
public:
    StacIndexBuilder(const Args& args, const std::string& pcType);

private:
    FileInfoPtr makeFileInfo(const std::string& filename) override;
    void getFileInfo(std::unique_ptr<FileInfo>& fileInfo) override;
    void createExtraFields(const std::unique_ptr<FileInfo>& fileInfo,
        TIndexFeature& feature) override;

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

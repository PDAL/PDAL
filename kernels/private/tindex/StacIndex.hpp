
#include <kernels/private/stac/StacInfo.hpp>
#include "TIndexBuilder.hpp"

namespace pdal
{
namespace tindex
{

static const std::string STAC_VERSION = "1.1.0";

class StacInfo
{
public:
    StacInfo() {}

    void init(std::string const& filename)
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

struct StacFileInfo : FileInfo
{
    StacInfo m_stacInfo;
};

class StacIndex : public TIndexBuilder
{
public:
    StacIndex(const Args& args, const std::string& pcType);
    ~StacIndex();
    //void create(const StringList& files) override;
private:
    std::unique_ptr<FileInfo> makeFileInfo(const std::string& filename) override;
    void getFileInfo(std::unique_ptr<FileInfo>& fileInfo) override;
    void createExtraFields(const std::unique_ptr<FileInfo>& fileInfo,
        TIndexFeature& feature) override;

    StringList m_extensions;
    std::string m_pcType;
    //std::vector<StacInfo> m_stacInfos;
};

} // namespace tindex
} // namespace pdal

#include "TIndexBuilder.hpp"

namespace pdal
{
namespace tindex
{

class TileIndex : public TIndexBuilder
{
public:
    TileIndex(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs);
    ~TileIndex();
    //void create(const StringList& files) override;
private:
    std::unique_ptr<FileInfo> makeFileInfo(const std::string& filename) override;
    void getFileInfo(std::unique_ptr<FileInfo>& fileInfo) override;
    void createExtraFields(const std::unique_ptr<FileInfo>& fileInfo, OGRFeatureH hFeature) override;
};

} // namespace tindex
} // namespace pdal
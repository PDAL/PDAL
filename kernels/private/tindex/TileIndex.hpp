
#include "TIndexBuilder.hpp"

namespace pdal
{
namespace tindex
{

struct field;

class TileIndexBuilder : public TIndexBuilder
{
public:
    TileIndexBuilder(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs);
    //void create(const StringList& files) override;
private:
    std::unique_ptr<FileInfo> makeFileInfo(const std::string& filename) override;
    void getFileInfo(std::unique_ptr<FileInfo>& fileInfo) override;
    void createExtraFields(const std::unique_ptr<FileInfo>& fileInfo,
        TIndexFeature& feature) override;

    Field *m_mtimeField;
    Field *m_ctimeField;
};

} // namespace tindex
} // namespace pdal

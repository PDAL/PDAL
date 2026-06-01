
#include "TIndexProcessor.hpp"

namespace pdal
{
namespace tindex
{

struct field;

class TileIndexBuilder : public TIndexProcessor
{
public:
    TileIndexBuilder(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs);
    //void create(const StringList& files) override;
private:
    FileInfoPtr makeFileInfo(const std::string& filename) override;
    void getFileInfo(FileInfoPtr& fileInfo) override;
    void createExtraFields(const FileInfoPtr& fileInfo,
        Feature& feature) override;

    Field *m_mtimeField;
    Field *m_ctimeField;
};

} // namespace tindex
} // namespace pdal

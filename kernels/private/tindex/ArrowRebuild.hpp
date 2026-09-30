
#include <map>

#include <arrow/type_fwd.h>
#include <arrow/io/type_fwd.h>
#include <arrow/ipc/type_fwd.h>

#include <pdal/pdal_types.hpp>

#include "TIndexError.hpp"

namespace pdal
{
namespace tindex
{
void nestFieldsToStruct(const std::string& filename);
/*
class ArrowRebuilder
{
public:
    ArrowRebuilder(std::string filename) : m_filename(filename)
    {}
    void execute();

private:
    std::string m_filename;
    std::shared_ptr<arrow::io::ReadableFile> m_file;
    std::shared_ptr<arrow::Schema> m_schema;
    std::shared_ptr<arrow::Table> m_table;
    std::map<std::string, std::string> m_structFields;
    //std::unique_ptr<::arrow::RecordBatchReader> m_parquetReader;
    //std::unique_ptr<parquet::arrow::FileReader> m_arrow_reader;
};
*/
} // namespace tindex
} // namespace pdal
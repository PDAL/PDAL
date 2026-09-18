
#include "ArrowRebuild.hpp"

#include <parquet/arrow/schema.h>
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#include <parquet/arrow/writer.h>

namespace pdal
{
namespace tindex
{
namespace
{

// a field to be reassembled into struct - can be nested in children
struct FieldNode
{
    std::shared_ptr<arrow::Field> leafField;
    std::shared_ptr<arrow::Array> leafArray;
    std::map<std::string, FieldNode> children;

    bool isLeaf() const { return children.empty(); }
};

// Split "x.y.z" into ["x", "y", "z"].
std::vector<std::string> splitName(const std::string& name)
{
    std::vector<std::string> parts;
    size_t start = 0;
    size_t dot;
    while ((dot = name.find('.', start)) != std::string::npos)
    {
        parts.push_back(name.substr(start, dot - start));
        start = dot + 1;
    }
    parts.push_back(name.substr(start));
    return parts;
}

void insertColumn(FieldNode& root, const std::string& name,
    const std::shared_ptr<arrow::Field>& field,
    const std::shared_ptr<arrow::Array>& array)
{
    std::vector<std::string> parts = splitName(name);

    FieldNode *node = &root;
    for (size_t i = 0; i < parts.size() - 1; ++i)
        node = &node->children[parts[i]];

    FieldNode& leaf = node->children[parts.back()];
    leaf.leafField = field;
    leaf.leafArray = array;
}

std::pair<std::shared_ptr<arrow::Field>, std::shared_ptr<arrow::Array>>
buildNode(const std::string& name, const FieldNode& node)
{
    if (node.isLeaf())
        return { node.leafField->WithName(name), node.leafArray };

    std::vector<std::shared_ptr<arrow::Field>> childFields;
    std::vector<std::shared_ptr<arrow::Array>> childArrays;
    for (auto& [childName, childNode] : node.children)
    {
        auto [childField, childArray] = buildNode(childName, childNode);
        childFields.push_back(childField);
        childArrays.push_back(childArray);
    }

    arrow::Result<std::shared_ptr<arrow::StructArray>> structArrayResult =
        arrow::StructArray::Make(childArrays, childFields);
    if (!structArrayResult.ok())
        throw TIndexError("Failed to build struct for field '" + name +
            "': " + structArrayResult.status().ToString());

    auto field = arrow::field(name, arrow::struct_(childFields), /*nullable=*/true);
    return { field, *structArrayResult };
}

std::shared_ptr<arrow::Table> nestTable(const std::shared_ptr<arrow::Table>& flat)
{
    arrow::Result<std::shared_ptr<arrow::Table>> combinedResult = flat->CombineChunks();
    if (!combinedResult.ok())
        throw TIndexError("Failed to combine table chunks: " +
            combinedResult.status().ToString());
    std::shared_ptr<arrow::Table> combined = *combinedResult;

    FieldNode root;
    const std::shared_ptr<arrow::Schema>& schema = combined->schema();
    for (int i = 0; i < schema->num_fields(); ++i)
    {
        std::shared_ptr<arrow::ChunkedArray> col = combined->column(i);
        insertColumn(root, schema->field(i)->name(), schema->field(i),
            col->chunk(0));
    }

    std::vector<std::shared_ptr<arrow::Field>> topFields;
    std::vector<std::shared_ptr<arrow::Array>> topArrays;
    for (auto& [name, node] : root.children)
    {
        auto [field, array] = buildNode(name, node);
        topFields.push_back(field);
        topArrays.push_back(array);
    }

    // preserve the original metadata
    auto newSchema = arrow::schema(topFields)->WithMetadata(schema->metadata());
    return arrow::Table::Make(newSchema, topArrays, combined->num_rows());
}

} // unnamed namespace

void nestFieldsToStruct(const std::string& filename)
{
    std::shared_ptr<arrow::io::ReadableFile> file;
    auto result = arrow::io::ReadableFile::Open(filename);
    if (result.ok())
        file = result.ValueOrDie();
    else
    {
        std::stringstream msg;
        msg << "Unable to open '" << filename << "' to read data with message '"
            << result.status().ToString() <<"'";
        throw TIndexError(msg.str());
    }

    auto reader_result = parquet::arrow::OpenFile(file, arrow::default_memory_pool());
    if (!reader_result.ok())
    {
        std::stringstream msg;
        msg << "Unable to open file '" << filename << "' with message '"
            << reader_result.status().ToString() << "'";
        throw TIndexError(msg.str());
    }
    auto arrow_reader = std::move(reader_result).ValueOrDie();

    std::shared_ptr<arrow::Table> flatTable;
    auto status = arrow_reader->ReadTable(&flatTable);
    if (!status.ok())
    {
        std::stringstream msg;
        msg << "Unable to open file '" << filename << "' with message '"
            << status.ToString() << "'";
        throw TIndexError(msg.str());
    }
    std::shared_ptr<arrow::Table> nested = nestTable(flatTable);

    /*
    auto schema_result = arrow_reader->GetSchema(&m_schema);
    if (!schema_result.ok())
    {
        std::stringstream msg;
        msg << "Unable to open schema for file '" << filename << "' with message '"
            << reader_result.status().ToString() << "'";
        throw TIndexError(msg.str());
    }
    StringList field_names;
    field_names.reserve(m_schema->num_fields());
    for (int i = 0; i < m_schema->num_fields(); ++i)
    {
        std::string fieldName = m_schema->field(i)->name();
        field_names.push_back(m_schema->field(i)->name());
    }
    */
    // write to a tempfile instead?
    arrow::Result<std::shared_ptr<arrow::io::FileOutputStream>> createResult =
        arrow::io::FileOutputStream::Open(filename);
    if (!createResult.ok())
        throw TIndexError("Unable to overwrite file '" + tmpFilename +
            "': " + createResult.status().ToString());
    std::shared_ptr<arrow::io::FileOutputStream> outfile = *createResult;

    status = parquet::arrow::WriteTable(*nested, arrow::default_memory_pool(), outfile,
        nested->num_rows() > 0 ? nested->num_rows() : 1);
    if (!status.ok())
        throw TIndexError("Unable to write nested Parquet file '" + tmpFilename +
            "': " + status.ToString());
    status = file->Close();
    if (!status.ok())
    {
        std::stringstream msg;
        msg << "Unable to read next batch for file '" << filename << "' with message '"
            << status.ToString() <<"'";
        throw TIndexError(msg.str());
    }
}

} // namespace tindex
} // namespace pdal

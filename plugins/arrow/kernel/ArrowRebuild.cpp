
#include "ArrowRebuild.hpp"

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

// Add a field to an array column
void insertColumn(FieldNode& root, const std::string& name,
    const std::shared_ptr<arrow::Field>& field,
    const std::shared_ptr<arrow::Array>& array)
{
    std::vector<std::string> parts = splitName(name);

    FieldNode *node = &root;
    for (size_t i = 0; i < parts.size() - 1; ++i)
        node = &node->children[parts[i]];

    // Always only add the leaf node
    FieldNode& leaf = node->children[parts.back()];
    leaf.leafField = field;
    leaf.leafArray = array;
}

// Turn a FieldNode into an Arrow field. Leaf nodes are returned as-is
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

// Take the flat table and return one where dotted column names have been 
// grouped into structs.
std::shared_ptr<arrow::Table> nestTable(const std::shared_ptr<arrow::Table>& flat)
{
    // Merge any chunks in the table to make sure there is only one chunk being dealt with
    arrow::Result<std::shared_ptr<arrow::Table>> combinedResult = flat->CombineChunks();
    if (!combinedResult.ok())
        throw TIndexError("Failed to combine table chunks: " +
            combinedResult.status().ToString());
    std::shared_ptr<arrow::Table> combined = *combinedResult;

    // Put all the columns into the table
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

    // preserve the original metadata, contains geoparquet metadata
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

    // Read Parquet logical types like JSON back as Arrow extension types.
    // Without this the JSON columns come back as plain strings, and OGR
    // no longer treats them as String(JSON)
    parquet::ArrowReaderProperties reader_props;
    reader_props.set_arrow_extensions_enabled(true);

    parquet::arrow::FileReaderBuilder builder;
    auto status = builder.Open(file);
    if (!status.ok())
    {
        throw TIndexError("Unable to open file with builder: " + status.ToString());
    }
    builder.properties(reader_props);

    std::unique_ptr<parquet::arrow::FileReader> arrow_reader;
    auto build_result = builder.Build(&arrow_reader);
    if (!build_result.ok())
    {
        throw TIndexError("Unable to build FileReader: " + build_result.ToString());
    }

    // read everything into memory.
    std::shared_ptr<arrow::Table> flatTable;
    status = arrow_reader->ReadTable(&flatTable);
    if (!status.ok())
    {
        std::stringstream msg;
        msg << "Unable to open file '" << filename << "' with message '"
            << status.ToString() << "'";
        throw TIndexError(msg.str());
    }
    std::shared_ptr<arrow::Table> nested = nestTable(flatTable);

    // write to a tempfile instead?
    arrow::Result<std::shared_ptr<arrow::io::FileOutputStream>> createResult =
        arrow::io::FileOutputStream::Open(filename);
    if (!createResult.ok())
        throw TIndexError("Unable to overwrite file '" + filename +
            "': " + createResult.status().ToString());
    std::shared_ptr<arrow::io::FileOutputStream> outfile = *createResult;

    // store_schema() writes the schema to metadata - allows extension types
    // like OGR StringJSON to be preserved 
    std::shared_ptr<parquet::ArrowWriterProperties> writer_props =
        parquet::ArrowWriterProperties::Builder().store_schema()->build();

    // Write everything as a single row group
    int64_t chunk_size = nested->num_rows() > 0 ? nested->num_rows() : 1;
    status = parquet::arrow::WriteTable(*nested, arrow::default_memory_pool(), outfile,
                                        chunk_size, parquet::default_writer_properties(),
                                        writer_props);
    if (!status.ok())
        throw TIndexError("Unable to write nested Parquet file '" + filename +
            "': " + status.ToString());
    status = file->Close();
    if (!status.ok())
    {
        std::stringstream msg;
        msg << "Arrow error closing file '" << filename << "' with message '"
            << status.ToString() <<"'";
        throw TIndexError(msg.str());
    }
    status = outfile->Close();
    if (!status.ok())
    {
        std::stringstream msg;
        msg << "Arrow error re-writing file '" << filename << "' with message '"
            << status.ToString() <<"'";
        throw TIndexError(msg.str());
    }
}

} // namespace tindex
} // namespace pdal

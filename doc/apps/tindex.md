(tindex_command)=

# tindex

The `tindex` command is used to create a [GDAL]-style tile index for
PDAL-readable point cloud types (see [gdaltindex]).

The `tindex` command has two modes.  The first mode creates a spatial index
file for a set of point cloud files.  The second mode creates a point cloud
file that is the result of merging the points from files referred to in a
spatial index file that meet some criteria (usually a geographic region filter).

## tindex Creation Mode

```
$ pdal tindex create <tindex> <infile>
```

```
--tindex               OGR-readable/writeable tile index output
--glob                 Glob pattern of files to index, or a single text file from which 
                       to read a list of files to index (if `filelist` is specified).
--stdin, -s            Read list of input files from standard input
--filelist             Read list of input files from text file input (newline delimited)
--fast_boundary        Use extent instead of exact boundary
--lyr_name             OGR layer name to write into datasource
--tindex_name          Tile index column name
--ogrdriver, -f        OGR driver name to use
--t_srs                Target SRS of tile index
--a_srs                Assign SRS of tile with no SRS to this value
--write_absolute_path  Write absolute rather than relative file paths
--skip_different_srs   Reject files to be indexed with different SRS values
--threads              Number of threads to use for file boundary creation
--simplify             Simplify the file's exact boundary
--threshold            Number of points a cell must contain to be declared 
                       positive space, when creating exact boundaries
--resolution           Cell edge length to be used when creating exact boundaries
--sample_size          Sample size for auto-edge length calculation in internal
                       hexbin filter (exact boundary)
--where                Expression describing points to be processed for exact
                       boundary creation
```

This command will index the files referred to by `glob` or `filelist` and place the
result in `tindex`.  The `tindex` is a vector file or database that
will be created by `pdal` as necessary to store the file index.
The type of the index
file can be specified by specifying the OGR code for the format using the
`--ogrdriver` option.  If no driver is specified, the format defaults to "ESRI
Shapefile".  Any filetype that can be handled by
[OGR](https://gdal.org/en/latest/drivers/vector/) is acceptable.

In vector file-speak, each file specified by `glob` is stored as a
feature in a layer in the index file. The [glob pattern](http://man7.org/linux/man-pages/man7/glob.7.html) 
normally needs to be quoted to prevent shell expansion of wildcard characters.

Exact file boundaries (used when `--fast_boundary` is not set to `true`)
are created with a grid of tessellated hexagons, in a modified version of
{ref}`filters.hexbin`. This is controlled with the `simplify`, `threshold`,
`resolution` and `sample_size` options.

Creation mode also supports parallel file processing by specifying the `threads`
option.

## tindex Merge Mode

```
$ pdal tindex merge <tindex> <filespec>
```

```
--tindex         OGR-readable/writeable tile index output
--filespec       Output filename
--lyr_name       OGR layer name to write into datasource
--tindex_name    Tile index column name
--ogrdriver, -f  OGR driver name to use
--bounds         Extent (in XYZ) to clip output to
--polygon        Well-known text of polygon to clip output
--t_srs          Spatial reference of the clipping geometry.
```

This command will read the existing index file `tindex` and merge the
points in the indexed files that pass any filter that might be specified,
writing the output to the point cloud file specified in `filespec`.
The type of the output file is determined automatically from the filename
extension.

## Example 1:

Find all LAS files via `find`, send that file list via STDIN to
`pdal tindex`, and write a SQLite tile index file with a layer named `pdal`:

```
$ find las/ -iname "*.las" | pdal tindex create index.sqlite -f "SQLite" \
    --stdin --lyr_name pdal
```

## Example 2:

With a list of LAS files created via `find`, write that file list to `pdal tindex` 
using `--filelist`, and write a SQLite tile index file with a layer named `pdal`:

```
$ find las/ -iname "*.las" > files.txt
$ pdal tindex create index.sqlite files.txt -f "SQLite" \
    --filelist --lyr_name pdal
```

## Example 3:

Glob a list of LAS files, output the SRS for the index entries to EPSG:4326, and
write out an [SQLite] file.

```
$ pdal tindex create index.sqlite "*.las" -f "SQLite" --lyr_name "pdal" \
    --t_srs "EPSG:4326"
```

[gdal]: https://gdal.org
[gdaltindex]: https://gdal.org/en/latest/programs/gdaltindex.html
[sqlite]: http://www.sqlite.org

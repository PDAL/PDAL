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
$ pdal tindex create <tindex> <filespec>
```

```
--tindex               OGR-readable/writeable tile index output
--filespec             Build: Pattern of files to index. Merge: Output filename
--fast_boundary        Use extent instead of exact boundary
--lyr_name             OGR layer name to write into datasource
--tindex_name          Tile index column name
--ogrdriver, -f        OGR driver name to use
--t_srs                Target SRS of tile index
--a_srs                Assign SRS of tile with no SRS to this value
--write_absolute_path  Write absolute rather than relative file paths
--stdin, -s            Read filespec pattern from standard input
```

This command will index the files referred to by `filespec` and place the
result in `tindex`.  The `tindex` is a vector file or database that
will be created by `pdal` as necessary to store the file index.
The type of the index
file can be specified by specifying the OGR code for the format using the
`--ogrdriver` option.  If no driver is specified, the format defaults to "ESRI
Shapefile".  Any filetype that can be handled by
[OGR](https://gdal.org/en/latest/drivers/vector/) is acceptable.

In vector file-speak, each file specified by `filespec` is stored as a
feature in a layer in the index file. The `filespec` is a [glob pattern](http://man7.org/linux/man-pages/man7/glob.7.html).  and normally needs to be
quoted to prevent shell expansion of wildcard characters.

## tindex Merge Mode

```
$ pdal tindex merge <tindex> <filespec>
```

This command will read the existing index file `tindex` and merge the
points in the indexed files that pass any filter that might be specified,
writing the output to the point cloud file specified in `filespec`.
The type of the output file is determined automatically from the filename
extension.

```
--tindex         OGR-readable/writeable tile index output
--filespec       Build: Pattern of files to index. Merge: Output filename
--lyr_name       OGR layer name to write into datasource
--tindex_name    Tile index column name
--ogrdriver, -f  OGR driver name to use
--bounds         Extent (in XYZ) to clip output to
--polygon        Well-known text of polygon to clip output
--t_srs          Spatial reference of the clipping geometry.
```

## Example 1:

Find all LAS files via `find`, send that file list via STDIN to
`pdal tindex`, and write a SQLite tile index file with a layer named `pdal`:

```
$ find las/ -iname "*.las" | pdal tindex create index.sqlite -f "SQLite" \
    --stdin --lyr_name pdal
```

## Example 2:

Glob a list of LAS files, output the SRS for the index entries to EPSG:4326, and
write out an [SQLite] file.

```
$ pdal tindex create index.sqlite "*.las" -f "SQLite" --lyr_name "pdal" \
    --t_srs "EPSG:4326"
```

[gdal]: https://gdal.org
[gdaltindex]: https://gdal.org/en/latest/programs/gdaltindex.html
[sqlite]: http://www.sqlite.org

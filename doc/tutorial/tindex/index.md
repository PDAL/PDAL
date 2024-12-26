(tindex-tutorial)=

# Working with large collections of data using `pdal tindex`

```{contents} Table of Contents
:depth: 3
```

This tutorial will introduce some workflows that use PDAL's {ref}`tindex <tindex_command>`
functionality:
* Creating a new tile index with `pdal tindex create`
    * Make the index using a large, cloud-hosted lidar dataset
    * Fine-tune the creation options
* Querying a PDAL tile index using {ref}`readers.tindex`
    * Read a tile index stored on the web
    * Chain shell commands to feed polygon geometries into a {ref}`pipeline`
    * Read point clouds within a specific area, and output data products

## Introduction

A {ref}`PDAL tile index <tindex_command>` is essentially a version of GDAL's [gdaltindex], only
for point clouds instead of raster files. Running the `pdal tindex create` command can
create a new feature layer that contains the file locations and extents of a
data collection, and `pdal tindex merge` can use this tile index to combine and process
the files that it references.

Very often, data from lidar surveys is broken up into rectangular tiles in order to keep
file sizes manageable. A workflow that incorporates PDAL tile index functionality is able
to speed up processing for large datasets like this, since it reads only the files it needs
to; in the following examples, we read the tile index within a polygon, but filtering can also
be done with a bounding box or SQL query.

In this tutorial we will make a new tile index for an entire county, and use
{ref}`readers.tindex` to generate a canopy height raster for every town that intersects the dataset.

## Getting Started

To run this tutorial, we need to create a new {ref}`Conda environment<conda>`
with PDAL installed.

### Environment setup

Make and activate a new environment with PDAL installed. We're going to call
ours `tindex`

```bash
conda create -c conda-forge -n tindex -y pdal curl
conda activate tindex
```

### File list

We will use the 2019 USGS 3DEP LiDAR survey of Adams County, Wisconsin for this
tutorial. Rather than saving the 1025 files and 68gb of data locally, we will
be using them from the cloud in an S3 bucket location. For the ["Creating an
Index"](#creating-an-index), section you can manually download the list of S3
URLs [here](https://s3.amazonaws.com/hobu-lidar/WI_Adams_2019/filelist.txt),
but we will typically use the URL in combination with [curl] commands to
dynamically build the index.



## Creating an Index

The `pdal tindex create` command uses [OGR] to write a new vector dataset, with
a separate polygon feature for each file. The features can be highly detailed 2D
polygons describing where data exist, or they can be a simple bounding box. The
tile index can be any OGR vector format, but some are more suitable than
others. For this exercise, we will be creating and modifying a [GeoParquet]
file. GeoParquet offers efficient access to large geospatial datasets through
an implementation of the Apache Parquet format that is streamable from remotely
accessible network locations.

`tindex create` can take input from [stdin] (via the ``-s`` flag)
or a glob pattern (via the ``--filespec`` option). Filespec can read files from
a local directory, using wildcard characters; in this example, we grab every
file ending in `.copc.laz` from a folder named `data`.

```bash
pdal tindex create WI_Adams.parquet -f Parquet --filespec "data/*.copc.laz"
```


### Creation Options

For each of these files, an implementation of {ref}`filters.hexbin` reads the
points and outputs an exact boundary. The following options of `pdal tindex
create` control the creation of these boundaries:

```
--tindex               OGR-readable/writeable tile index output
--filespec             Pattern of files to index
--fast_boundary        Use extent instead of exact boundary
--lyr_name             OGR layer name to write into datasource
--tindex_name          Tile index column name
--ogrdriver, -f        OGR driver name to use
--t_srs                Target SRS of tile index
--a_srs                Assign SRS of tile with no SRS to this value
--write_absolute_path  Write absolute rather than relative file paths
--stdin, -s            Read filespec pattern from standard input
--path_prefix          Prefix to be added to file paths when writing output
--threads              Number of threads to use for file boundary creation
--simplify             Simplify the file's exact boundary
--threshold            Number of points a cell must contain to be declared positive space, when creating exact boundaries
--resolution           cell edge length to be used when creating exact boundaries
--sample_size          Sample size for auto-edge length calculation in internal hexbin filter (exact boundary)
--where                Expression describing points to be processed for exact boundary creation
```

#### Boundary Control

* `simplify`: attempt to smooth the hexagons created by {ref}`filters.hexbin`
* `threshold`: the number of points a hexagon must contain to be considered
  positive space. Setting this to a larger number allows you to filter out
  areas with spurious data.
* `sample_size`: a number of points to sample to attempt to estimate a
  reasonable automatic hexagon size. Use `resolution` if you know the size you
  desire.
* `resolution`: an explicit edge size of the {ref}`filters.hexbin` polygons
  (not to be confused with {ref}`readers.copc` `resolution`, which is the
  resolution to *read* the point data)
* `fast_boundary`: Use a simple bounding box instead of an accumulated polygon
  to estimate the extent of the data

#### Filenames

* `path_prefix`: Insert this string before the path name of every entry in the
  tile index. Useful for adjusting relative paths after-the-fact or indexing a local
  tree of data and then setting its path names to URLs.
* `stdin`: Use [stdin] to read the list of filenames
* `write_absolute_path`: Insert absolute path names into the tile index

#### Performance

* `threads`: Number of files to read at a time
* `fast_boundary`: Reads the file headers for its bounding box if available
* `where`: Limit the data inserted into the {ref}`filters.hexbin`. This is
  useful for making boundaries of different attribute types.

```{note}
PDAL 2.9 introduced support for parallel creation of boundaries in `tindex
create` with the `--threads` option. Consider that many readers use multiple
threads by default, and since we're running a reader in each tindex boundary
process, the actual number of threads in use will be (reader threads * tindex
threads).
```

### Constructing the Index

We will build the index of Adams County, Wisconsin using a file list of
URLs. Each URL is a COPC file on AWS in the `s3://hobu-lidar` bucket,
which is publicly accessible.  HTTP URLs are non-globbable, so we must
specify stdin with a specific list of URLs that we are fetching via
[curl]:

`````{tab-set}
````{tab-item} Linux & macOS
```bash
curl https://s3.amazonaws.com/hobu-lidar/WI_Adams_2019/filelist.txt | \
    pdal tindex create --tindex "WI_Adams_2019.parquet" \
    --lyr_name "WI_Adams_2019" \
    -f Parquet -s \
    --readers.copc.resolution=10 \
    --readers.copc.threads=10 \
    --threads=8 \
    --edge_length=20
```
````

````{tab-item} Windows
Powershell:
```powershell
curl https://s3.amazonaws.com/hobu-lidar/WI_Adams_2019/filelist.txt | `
    pdal tindex create --tindex "WI_Adams_2019.parquet" `
    --lyr_name "WI_Adams_2019" `
    -f Parquet -s `
    --readers.copc.resolution=10 `
    --readers.copc.threads=10 `
    --threads=8 `
    --edge_length=20
```

Batch:
```batch
curl https://s3.amazonaws.com/hobu-lidar/WI_Adams_2019/filelist.txt | ^
    pdal tindex create --tindex "WI_Adams_2019.parquet" ^
    --lyr_name "WI_Adams_2019" ^
    -f Parquet -s ^
    --readers.copc.resolution=10 ^
    --readers.copc.threads=10 ^
    --threads=8 ^
    --edge_length=20
```
````
`````

```{note}
The {ref}`readers.copc` `resolution` option limits the resolution that
{ref}`readers.copc` (or {ref}`readers.ept`) reads points. This is very useful
for testing expensive commands to validate they are working as expected before
reading all of the points. In the case of a tindex, they are useful to building
a geometry to a specified resolution.
```

```{warning}
In this example, We're using 80 threads (8 `tindex` threads * 10
{ref}`readers.copc` threads) and a relatively coarse `readers.copc.resolution`.
Most of the time in each of these threads is spent waiting for the data to be
fetched from S3 over the network, so we are able to use a lot of them without
any issues. Using this many requests for `pdal tindex create` with local data
rather than cloud-hosted files could cause contention, since most of the
threads would be doing work instead of waiting on i/o.
```


```{figure} tiled.png
:scale: 40%

Viewing our completed tile index in QGIS.
```

## Merging the Index

We can now filter our [GeoParquet] tile index to determine which [COPC] files
to read within the city limits of each town in the county and create a raster
model of height above ground for each.  One advantage of [GeoParquet] is
efficient random reading of data from the web with GDAL's [virtual file
system]; this allows us to read a file from HTTP by streaming it, rather than
locally caching the entire file. GeoParquet files are also filterable and can
be used to [manage STAC items], making them well-suited to other types of large
geospatial datasets.

The ability to process multiple files and output a single product is one of the
key advantages of a tile index. The PDAL {ref}`batch processing tutorial
<workshop-batch-processing>` is an excellent example that demonstrates why it
is effective to manage groups of data as single resources. The exercise has a
similar goal of processing multiple point cloud tiles into a single raster, but
you must run a new pipeline for each tile and merge the results at the end.
{ref}`readers.tindex` can process the data referenced in the tile index as if
it's a single point cloud file, with the spatial extent of the output
controlled by the reader's `wkt` or `bounds` options if it supports such
options.

````{note}
If you skipped the ["Creating an Index"](#creating-an-index) section, download
the tile index from
[https://s3.amazonaws.com/hobu-lidar/WI_Adams_2019/WI_Adams_2019.parquet](https://s3.amazonaws.com/hobu-lidar/WI_Adams_2019/WI_Adams_2019.parquet)
and use it locally, or you can use it via [GDAL]'s
[vsicurl](https://gdal.org/en/stable/user/virtual_file_systems.html#vsicurl-http-https-ftp-files-random-access)
mechanism to point to the file by substituting the following for any

```
/vsicurl/https://s3.amazonaws.com/hobu-lidar/WI_Adams_2019/WI_Adams_2019.parquet
```
````

### Using `readers.tindex` to merge data

Once we have the tile index, we write a {ref}`pipeline` that uses a
{ref}`readers.tindex` reader to point at it. Our tile index could
be a local file or a remotely-accessible URL, but formats such as [GeoParquet]
support efficient reading over HTTP.

After reading the point clouds referenced in the tile index (with the same
{ref}`readers.copc` options as the creation command used),
{ref}`filters.hag_nn` creates a new height above ground dimension, which gets
written to a new GeoTIFF raster using {ref}`writers.gdal`. You might notice
that we don't supply some important options yet, like the boundary polygons for
{ref}`readers.tindex` and the filenames for {ref}`writers.gdal`: these values
are going to be substituted in when we run the `pdal pipeline` command.

```json
[
    {
        "type":"readers.tindex",
        "filename":"/vsicurl/https://s3.amazonaws.com/hobu-lidar/WI_Adams_2019.parquet",
        "lyr_name":"WI_Adams_2019",
        "reader_args":[{"type": "readers.copc", "threads": 10, "resolution": 10}]
    },
    {
        "type":"filters.hag_nn"
    },
    {
        "type":"writers.gdal",
        "gdaldriver":"GTiff",
        "dimension":"HeightAboveGround",
        "data_type":"float32",
        "output_type":"mean",
        "resolution": 0.00003
    }
]
```

```{note}
In the pipeline above, we are remotely reading our tile index from an S3 bucket
using GDAL's [virtual file system].  If you created the index [in the previous
section](#creating-a-new-index), feel free to replace the
'readers.tindex.filename' option with a path to the file on your machine.
```

Once the pipeline is saved to a new `pipeline.json` file, we need to construct
commands that fetch the town boundaries and substitute their polygons into our
pipeline. **We are going to use these to create one long command at the end, so
don't run any of them yet.** The following instructions will walk you through
what each part is doing, so our final product makes a little more sense.

1. First, we want to get the geometry of all administrative boundaries within
   Adams County. This can be done through an OpenStreetMap [Overpass] API query
   - in this case, we get all admin-level 8 (municipality) boundaries within
   the county.

```
area["wikipedia"="en:Adams County, Wisconsin"];(relation["boundary"="administrative"]["admin_level"="8"](area);>;);out meta;
```

Here is the command we run which fetches the overpass XML data containing our features of interest:

```
curl "https://overpass-api.de/api/interpreter?data=area%5B%22wikipedia%22%3D%22en%3AAdams+County%2C+Wisconsin%22%5D%3B%28relation%5B%22boundary%22%3D%22administrative%22%5D%5B%22admin_level%22%3D%228%22%5D%28area%29%3B%3E%3B%29%3Bout+meta%3B"
```
2. We can use GDAL [ogr2ogr] to convert the OSM data to [GeoParquet]-formatted text with
   only the "name" field and the multipolygon geometries. The SQL query also skips features without a name;
   there are some weird inner ring geometries that get created, so we only want
   the ones that have a town name associated with them.

`````{tab-set}
````{tab-item} Linux & macOS
```bash
curl -s "https://overpass-api.de/api/interpreter?data=area%5B%22wikipedia%22%3D%22en%3AAdams+County%2C+Wisconsin%22%5D%3B%28relation%5B%22boundary%22%3D%22administrative%22%5D%5B%22admin_level%22%3D%228%22%5D%28area%29%3B%3E%3B%29%3Bout+meta%3B" \
| ogr2ogr  \
    -sql "SELECT \"_ogr_geometry_\", \"name\" FROM \"multipolygons\" WHERE \"name\" != ''" \
    -f Parquet \
    municipalities.parquet /vsistdin/
```
````

````{tab-item} Windows
Powershell:
```powershell
curl -s "https://overpass-api.de/api/interpreter?data=area%5B%22wikipedia%22%3D%22en%3AAdams+County%2C+Wisconsin%22%5D%3B%28relation%5B%22boundary%22%3D%22administrative%22%5D%5B%22admin_level%22%3D%228%22%5D%28area%29%3B%3E%3B%29%3Bout+meta%3B" `
| ogr2ogr  `
    -sql "SELECT \"_ogr_geometry_\", \"name\" FROM \"multipolygons\" WHERE \"name\" != ''" `
    -f Parquet `
    municipalities.parquet /vsistdin/
```

Batch:
```batch
curl -s "https://overpass-api.de/api/interpreter?data=area%5B%22wikipedia%22%3D%22en%3AAdams+County%2C+Wisconsin%22%5D%3B%28relation%5B%22boundary%22%3D%22administrative%22%5D%5B%22admin_level%22%3D%228%22%5D%28area%29%3B%3E%3B%29%3Bout+meta%3B" ^
| ogr2ogr  ^
    -sql "SELECT \"_ogr_geometry_\", \"name\" FROM \"multipolygons\" WHERE \"name\" != ''" ^
    -f Parquet ^
    municipalities.parquet /vsistdin/
```
````
`````


3. Once we have the polygon geometries, the output contains a line of text
   listing field names at the beginning, and has quotes surrounding the WKT. We
   need to remove these from the output before it gets processed.


`````{tab-set}
````{tab-item} Linux & macOS
```bash
$ sed '1d' | tr -d '"'
```
````

````{tab-item} Windows
Powershell:
```powershell
PS > Select-Object -Skip 1
```

Batch:
```batch
ogr2ogr ^
    -sql "SELECT `"_ogr_geometry_`", `"name`" FROM `"multipolygons`" WHERE `"name`" != ''" ^
    -lco GEOMETRY=AS_WKT ^
    -lco SEPARATOR=SEMICOLON ^
    -f CSV ^
    /vsistdout/ /vsistdin/
```
````
`````

```
$ sed '1d' | tr -d '"'
```
In PowerShell, we can trim the quotes out later.
```
PS > Select-Object -Skip 1
```
4. Now, we make a loop that splits the WKT from the name, and runs a pipeline for each line.
The `readers.tindex` stage adds the same `readers.copc` options our creation command used, and
adds the WKT polygon. Next, `filters.hag_nn` writes a
new HeightAboveGround dimension. In the `writers.gdal` stage, we rasterize the height above
ground, and save the GeoTIFF in our current working directory.

```
$ while IFS=';' read -r wkt name; do pdal pipeline ./height_model.json \
    --readers.tindex.wkt=$wkt \
    --writers.gdal.filename="${name// /}.tif"; \
done;
```

```
PS > ForEach-Object { `
        $wkt_name=$_.split(';'); `
        pdal pipeline ./height_model.json `
            --readers.tindex.wkt=$($wkt_name[0].Trim('"')) `
            --writers.gdal.filename="./rasters/$($wkt_name[1].Trim()).tif"" }
```

All of these are piped together to create the fully completed command; paste it into your shell and run it!
```
$ curl "https://overpass-api.de/api/interpreter?data=area%5B%22wikipedia%22%3D%22en%3AAdams+County%2C+Wisconsin%22%5D%3B%28relation%5B%22boundary%22%3D%22administrative%22%5D%5B%22admin_level%22%3D%228%22%5D%28area%29%3B%3E%3B%29%3Bout+meta%3B" \
| ogr2ogr -sql "SELECT \"_ogr_geometry_\", \"name\" FROM \"multipolygons\" WHERE \"name\" != ''" \
    -lco GEOMETRY=AS_WKT -lco SEPARATOR=SEMICOLON -f CSV /vsistdout/ /vsistdin/ \
| sed '1d' | tr -d '"' \
| while IFS=';' read -r wkt name; do pdal pipeline ./height_model.json \
    --readers.tindex.wkt=$wkt \
    --writers.gdal.filename="${name// /}.tif"; \
done;
```
```
PS > curl "https://overpass-api.de/api/interpreter?data=area%5B%22wikipedia%22%3D%22en%3AAdams+County%2C+Wisconsin%22%5D%3B%28relation%5B%22boundary%22%3D%22administrative%22%5D%5B%22admin_level%22%3D%228%22%5D%28area%29%3B%3E%3B%29%3Bout+meta%3B" `
| ogr2ogr -sql "SELECT `"_ogr_geometry_`", `"name`" FROM `"multipolygons`" WHERE `"name`" != ''" `
    -lco GEOMETRY=AS_WKT -lco SEPARATOR=SEMICOLON -f CSV /vsistdout/ /vsistdin/ `
| Select-Object -Skip 1 | ForEach-Object { `
    $wkt_name=$_.split(';'); `
    pdal pipeline ./height_model.json `
        --readers.tindex.wkt=$($wkt_name[0].Trim('"')) `
        --writers.gdal.filename="$($wkt_name[1].Trim()).tif" }
```

```{figure} final_output.png
:scale: 45%

The final product: centered on the towns of Friendly and Adams.
```

[gdaltindex]: https://gdal.org/en/latest/programs/gdaltindex.html
[Conda environment]: https://conda.io/en/latest/
[Miniconda]: https://docs.anaconda.com/miniconda/
[OGR]: https://gdal.org/en/latest/drivers/vector/
[GeoParquet]: https://geoparquet.org/
[COPC]: https://copc.io/
[virtual file system]: https://gdal.org/en/latest/user/virtual_file_systems.html
[manage STAC items]: https://cloudnativegeo.org/blog/2024/08/introduction-to-stac-geoparquet/
[Overpass]: https://wiki.openstreetmap.org/wiki/Overpass_API
[ogr2ogr]: https://gdal.org/en/latest/programs/ogr2ogr.html
[curl]: https://curl.se
[Well Known Text]: https://en.wikipedia.org/wiki/Well-known_text_representation_of_coordinate_reference_systems
[stdin](https://en.wikipedia.org/wiki/Standard_streams)

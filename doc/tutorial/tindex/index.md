(tindex-tutorial)=

# Datasets with PDAL tindex

```{contents} Table of Contents
:depth: 2
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

To run this tutorial, we need to create a new {ref}`Conda environment<download>` with PDAL installed.
Using [Miniconda] is the easiest way to do this; download the appropriate installer
for your device [here](https://www.anaconda.com/download/success#miniconda) and follow the setup
instructions.

### Environment setup

1. Make a new environment. We're going to call ours "tindex"
```
conda create -c conda-forge -n tindex -y
```
2. Activate the environment you just created
```
conda activate tindex
```
3. Install PDAL into the environment from the conda-forge channel.
```
conda install -c conda-forge pdal 
```

The data used in this tutorial is a 2019 lidar survey of Adams County,
Wisconsin. Rather than saving the files locally, we will be fetching them
from an S3 bucket. For the "Creating a New Index" section, you can download the 
list of S3 URLs [here](https://github.com/PDAL/PDAL/blob/master/doc/tutorial/tindex/S3files.txt),
or save them when we run the command to create our index.

If you want to skip the creation of the tile index, move to the ["Merging the Index"
section.](#merging-the-index)

## Creating a New Index

The `pdal tindex create` command uses [OGR] to write a new vector dataset, with
separate polygon features for each file. A tile index can be created with
any OGR vector format, but some are more suitable than others. For this
exercise, we will be creating and modifying a [GeoParquet] file. This offers 
fast & efficient access to large geospatial datasets through an 
implementation of the Apache Parquet format.

`tindex create` can take input from stdin (the -s flag) or a glob pattern 
(the --filespec option). Filespec can read files from a local directory, using 
wildcard characters; in this example, we grab every file ending in '.copc.laz' 
from a folder named 'data'.
```
pdal tindex create WI_Adams.parquet -f Parquet --filespec "data/*.copc.laz"
```
In our case, since we're fetching from URLs (which are non-globbable) we need
to specify stdin with a specific list of filenames:
```
pdal tindex create WI_Adams.parquet -f Parquet -s < files.txt
```

For each of these files, an implementation of {ref}`filters.hexbin`
reads the points and outputs an exact boundary. The following options control the 
creation of these boundaries:
```
--simplify             Simplify the file's exact boundary (default: true)
--threshold            Number of points a cell must contain to be declared 
                       positive space (default: 15)
--resolution           Cell edge length to be used when creating exact boundaries
--sample_size          Sample size for auto-edge length calculation in internal
                       hexbin filter (default: 5000)
```
```{warning}
PDAL has introduced support for parallel creation of boundaries in 
`tindex create`, specified with the `--threads` option. Keep in mind that many 
readers use multiple threads by default, and since we're running a reader in each
tindex boundary process, the actual number of threads in use will be 
(reader threads * tindex threads).
```

This command saves the list of S3 URLs in your current working directory, and 
creates a new tile index for the entire dataset. Copy the following into your 
conda shell: it might take a while to run.
```
$ curl https://raw.githubusercontent.com/PDAL/PDAL/refs/heads/master/doc/tutorial/tindex/S3files.txt \
    -o files.txt \
| pdal tindex create -s --tindex "WI_Adams_2019.parquet" -f Parquet \
    --readers.copc.threads=10 --readers.copc.resolution=10 \
    --lyr_name "WI_Adams_2019" --threads=8 --edge_length=20 \
    < files.txt
```
```{note}
In this example, We're using 80 threads (8 `tindex` threads * 10 `readers.copc` threads) and a 
relatively coarse `readers.copc.resolution`. Most of the time in each of these threads is 
spent waiting for the data to be fetched from S3 over the network, so we are able to use a lot of 
them without any issues. Using this many requests for `pdal tindex create` with local data rather than
cloud-hosted files could cause contention, since most of the threads would be doing work instead of waiting
on i/o.
```

```{figure} tiled.png
:scale: 30%

Viewing our completed tile index in QGIS.
```

## Merging the Index

With our new tile index [GeoParquet], we are going to read the indexed [COPC] files within the 
city limits of each town in the county, creating a raster model of height above ground for each.
One advantage of this file type is the efficient random reading of data from the web 
with GDAL's [virtual file system]; this allows us to read a file from HTTP by streaming
it, rather than saving the entire thing. GeoParquet files are also queryable and can 
[manage STAC items], making them well-suited to other types of large geospatial datasets.

The ability to process multiple files and output a single product is one of the key advantages 
of a tile index. Take the PDAL workshop {ref}`batch processing tutorial <workshop-batch-processing>`
for an example of why. That exercise has a similar goal of processing multiple point cloud tiles
into a single raster, but it has to run a new pipeline for each tile and merge the results. 
{ref}`readers.tindex` can process the data referenced in the tile index as if it's a single point cloud
file, with the spatial extent of the output controlled by the reader's `wkt` or `bounds` options.

### Walking Through the Workflow

Making a command that reads a tile index within multiple extents requires a bit of data preparation
before PDAL can process it. However, the concepts in this workflow can be applied to other
{ref}`readers.tindex` data processing tasks with a little modification. Bash commands are provided for Unix
users, and their Powershell equivalents for Windows.

First, we make a new {ref}`pipeline`. After reading the point clouds referenced in the tile index
(with the same {ref}`readers.copc` options as the creation command used),
{ref}`filters.hag_nn` creates a new height above ground dimension, which gets written to a new GeoTIFF raster
using {ref}`writers.gdal`. You might notice that we don't supply some important options
yet, like the boundary polygons for {ref}`readers.tindex` and the filenames for {ref}`writers.gdal`:
these values are going to be substituted in when we run the `pdal pipeline` command.

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
In the pipeline above, we are pulling our tile index from an S3 bucket using GDAL's [virtual file system].
If you created the index [in the previous section](#creating-a-new-index), feel free to replace the 
'readers.tindex.filename' option with a path to the file on your machine.
```

Once that pipeline has been saved to a new .json file, it's time to construct commands that read 
the town boundaries and substitute their polygons into our pipeline. **We are going to use these to 
create one long command at the end, so don't run any of them yet.** The following instructions will walk you through what 
each part is doing, so our final product makes a little more sense.

1. First, we want to get the geometry of all administrative boundaries within Adams County. This can
be done through an OpenStreetMap [Overpass] API query - in this case, we get all admin-level 8 (municipality)
boundaries within the county.
```
area["wikipedia"="en:Adams County, Wisconsin"];(relation["boundary"="administrative"]["admin_level"="8"](area);>;);out meta;
```
Here is the command we run which fetches the overpass XML data containing our features of interest:
```
curl "https://overpass-api.de/api/interpreter?data=area%5B%22wikipedia%22%3D%22en%3AAdams+County%2C+Wisconsin%22%5D%3B%28relation%5B%22boundary%22%3D%22administrative%22%5D%5B%22admin_level%22%3D%228%22%5D%28area%29%3B%3E%3B%29%3Bout+meta%3B"
```
2. We use GDAL [ogr2ogr] to convert the OSM data to CSV-formatted text with only the "name" field and
the multipolygon geometries as [Well Known Text], separated by a semicolon. The SQL query also skips features 
without a name; there are some weird inner ring geometries that get created, so we only want the ones that 
have a town name associated with them.
```
$ ogr2ogr -sql "SELECT \"_ogr_geometry_\", \"name\" FROM \"multipolygons\" WHERE \"name\" != ''" \
    -lco GEOMETRY=AS_WKT -lco SEPARATOR=SEMICOLON -f CSV /vsistdout/ /vsistdin/
```
```
PS > ogr2ogr -sql "SELECT `"_ogr_geometry_`", `"name`" FROM `"multipolygons`" WHERE `"name`" != ''" `
    -lco GEOMETRY=AS_WKT -lco SEPARATOR=SEMICOLON -f CSV /vsistdout/ /vsistdin/
```
3. Once we have the polygon geometries, the output contains a line of text listing field names at 
the beginning, and has quotes surrounding the WKT. We need to remove these from the output before
it gets processed.
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
[GeoParquet]: https://getindata.com/blog/introducing-geoparquet-data-format/
[COPC]: https://copc.io/
[virtual file system]: https://gdal.org/en/latest/user/virtual_file_systems.html
[manage STAC items]: https://cloudnativegeo.org/blog/2024/08/introduction-to-stac-geoparquet/
[Overpass]: https://wiki.openstreetmap.org/wiki/Overpass_API
[ogr2ogr]: https://gdal.org/en/latest/programs/ogr2ogr.html
[Well Known Text]: https://en.wikipedia.org/wiki/Well-known_text_representation_of_coordinate_reference_systems

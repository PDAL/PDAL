(writers)=

# Writers

Writers consume data provided by {ref}`readers`. Some writers can consume any
dimension type, while others only understand fixed dimension names.

:::{note}
PDAL predefined dimension names can be found in the dimension registry:
{ref}`dimensions`
:::

```{toctree}
:glob: true
:hidden: true
:maxdepth: 1

writers.arrow
writers.bpf
writers.copc
writers.draco
writers.ept_addon
writers.e57
writers.fbi
writers.fbx
writers.gdal
writers.gltf
writers.las
writers.matlab
writers.nitf
writers.null
writers.ogr
writers.pcd
writers.pgpointcloud
writers.ply
writers.raster
writers.sbet
writers.text
writers.tiledb
```

{ref}`writers.arrow`

: write Apache Arrow Feather- or Parquet-formatted files

{ref}`writers.bpf`

: write BPF version 3 files. BPF is an NGA specification for point cloud data.

{ref}`writers.copc`

: COPC, or Cloud Optimized Point Cloud, is an LAZ 1.4 file stored as a
  clustered octree.

{ref}`writers.draco`

: Write a buffer in Google Draco format

{ref}`writers.ept_addon`

: Append additional dimensions to Entwine resources.

{ref}`writers.e57`

: Write data in the E57 format.

{ref}`writers.fbi`

: Write TerraSolid FBI format

{ref}`writers.fbx`

: Write mesh output in the Adobe FBX format.

{ref}`writers.gdal`

: Create a raster from a point cloud using an interpolation algorithm.

{ref}`writers.gltf`

: Write mesh data in GLTF format.  Point clouds without meshes cannot be
  written.

{ref}`writers.las`

: Write ASPRS LAS and LAZ versions 1.0 - 1.4 formatted data.

{ref}`writers.matlab`

: Write MATLAB .mat files. The output has a single array struct.

{ref}`writers.nitf`

: Write LAS and LAZ point cloud data, wrapped in a NITF 2.1 file.

{ref}`writers.null`

: Provides a sink for points in a pipeline. It's the same as sending pipeline
  output to /dev/null.

{ref}`writers.ogr`

: Write a point cloud as a set of OGR points/multipoints

{ref}`writers.pcd`

: Write PCD-formatted files in the ASCII, binary, or compressed format.

{ref}`writers.pgpointcloud`

: Write to a PostgreSQL database that has the PostgreSQL Pointcloud extension
  enabled.

{ref}`writers.ply`

: Write points as PLY vertices. Can also emit a mesh as a set of faces.

{ref}`writers.raster`

: Writes rasters using GDAL. Rasters must be created using a PDAL filter.

{ref}`writers.sbet`

: Write data in the SBET format.

{ref}`writers.text`

: Write points in a text file. GeoJSON and CSV formats are supported.

{ref}`writers.tiledb`

: Write points into a TileDB database.

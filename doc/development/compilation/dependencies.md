(dependencies)=

# Dependencies

```{note}
The absolute best source of build and configuration examples is the
PDAL GitHub repository. Specifically, the continuous integration
scripts at <https://github.com/PDAL/PDAL/tree/master/scripts/ci>
```

PDAL depends on a number of libraries to do its work.  Make sure
those dependencies are installed on your system before installing PDAL
or use a packaging system that will automatically load prerequisites.
Packaging system such as [apt] or [Conda] can be used to install dependencies
on your system.

## Required Dependencies

### GDAL (3.0+)

PDAL uses GDAL for spatial reference system description manipulation, and image
reading supporting for the NITF driver. In
conjunction with [GeoTIFF], GDAL is used to convert GeoTIFF keys and OGC WKT SRS
description strings into formats required by specific drivers.

```
Source: https://github.com/OSGeo/gdal
Conda: https://anaconda.org/conda-forge/gdal
```

### GeoTIFF (1.3+)

PDAL uses GeoTIFF in conjunction with GDAL for GeoTIFF key support in the
LAS driver.  GeoTIFF is typically a dependency of GDAL, so installing GDAL
from a package will generally install GeoTIFF as well.

```
Source: https://github.com/OSGeo/libgeotiff
Conda: https://anaconda.org/conda-forge/geotiff
```

```{note}
`GDAL` surreptitiously embeds a copy of [GeoTIFF] in its library build
but there is no way for you to know this.  In addition to embedding
libgeotiff, it also strips away the library symbols that PDAL needs,
meaning that PDAL can't simply link against [GDAL].  If you are
building both of these libraries yourself, make sure you build GDAL
using the "External libgeotiff" option, which will prevent the
insanity that can ensue on some platforms.  [Conda Forge] users, including
those using that platform to link and build PDAL themselves, do
not need to worry about this issue.
```

## Plugin Dependencies

PDAL comes with optional plugin stages that require other libraries in order
to run.  Many of these libraries are licensed in a way incompatible with
the PDAL license or they may be commercial products that require purchase.

### Nitro (Requires specific source package)

Nitro is a library that provides [NITF] support for PDAL to write LAS-in-NITF
files for {ref}`writers.nitf`.  You must use the specific version of Nitro
referenced below for licensing and compatibility reasons.:

```
Source: http://github.com/hobu/nitro
```

### TileDB  (1.4.1+)

[TileDB] is an efficient multi-dimensional array management system which
introduces a novel on-disk format that can effectively store dense and sparse
array data with support for fast updates and reads. It features excellent
compression, and an efficient parallel I/O system with high scalability. It is
used by {ref}`writers.tiledb` and {ref}`readers.tiledb`.:

```
Source: https://github.com/TileDB-Inc/TileDB
Conda: https://anaconda.org/conda-forge/tiledb
```

[apt]: https://help.ubuntu.com/lts/serverguide/apt.html
[asprs las]: http://www.asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
[cmake]: http://www.cmake.org
[conda]: https://conda.io/en/latest/
[conda forge]: https://anaconda.org/conda-forge/pdal
[debian]: http://www.debian.org
[debiangis]: http://wiki.debian.org/DebianGis
[gdal]: http://www.gdal.org
[geotiff]: http://trac.osgeo.org/geotiff
[laszip]: http://laszip.org
[libxml2]: http://xmlsoft.org
[nitf]: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format
[nitro]: http://nitro-nitf.sourceforge.net/wikka.php?wakka=HomePage
[point cloud library (pcl)]: http://pointclouds.org
[tiledb]: https://www.tiledb.io

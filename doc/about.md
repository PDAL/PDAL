(about)=

# About

## What is PDAL?

{{ PDAL }} is Point Data Abstraction Library.  It is a C/C++ open source library
and applications for translating and processing [point cloud data]. It is not
limited to {{ LiDAR }} data, although the focus and impetus for many of the
tools in the library have their origins in LiDAR.

## What is its big idea?

PDAL allows you to compose {ref}`operations <filters>` on point clouds into
{ref}`pipelines <pipeline>` of stages. These pipelines can
be written in a declarative JSON syntax or constructed using the available API.

### Why would you want to do that?

A task might be to load some [ASPRS LAS] (the most common LiDAR binary format)
data into a database, but you wanted to transform it into a common coordinate
system along the way.

One option would be to write a specialized monolithic
program that reads LAS data, reprojects it as necessary, and then handles the
necessary operations to insert the data in the appropriate format in the
database.  This approach has a distinct disadvantage in that without careful
planning it could quickly spiral out of control as you add new little tweaks
and features to the operation. It ends up being very specific, and it
does not allow you to easily reuse the component that reads the LAS data
separately from the component that transforms the data.

The PDAL approach is to chain together a set of components,
each of which encapsulates specific functionality.  The components allow for
reuse, composition, and separation of concerns.  PDAL views point cloud
processing operations as a pipeline composed as a series of stages.  You might
have a simple pipeline composed of a {ref}`LAS Reader <readers.las>` stage, a
{ref}`Reprojection <filters.reprojection>` stage, and a {ref}`PostgreSQL Writer
<writers.pgpointcloud>`, for example. Rather than writing a single, monolithic
specialized program to perform this operation, you can dynamically compose it
as a sequence of steps or operations.

```{figure} ./images/las-reproject-pgpointcloud.png
A simple PDAL pipeline composed of a reader, filter, and writer
stages.
```

A PDAL JSON {ref}`pipeline` that composes this operation to reproject
and load the data into PostgreSQL might look something like the following:

```{code-block} json
:emphasize-lines: 4, 8, 12
:linenos: true

{
  "pipeline":[
    {
      "type":"readers.las",
      "filename":"input.las"
    },
    {
      "type":"filters.reprojection",
      "out_srs":"EPSG:3857"
    },
    {
      "type":"writers.pgpointcloud",
      "connection":"host='localhost' dbname='lidar' user='hobu'",
      "table":"output",
      "srid":"3857"
    }
  ]
}
```

PDAL can compose intermediate stages for operations such as filtering,
clipping, tiling, transforming into a processing pipeline and reuse as
necessary. It allows you to define these pipelines as [JSON], and it
provides a command, {ref}`pipeline_command`, to allow you to execute them.

```{note}
Raster processing tools often compose operations with this approach.
PDAL conceptually steals its pipeline modeling from {{ GDAL }}'s
[Virtual Raster Format].
```

## How is it different than other tools?

### LAStools

```{index} LAStools
```

One of the most common open source processing tool suites available for LiDAR
processing is [LAStools] from [Martin Isenburg]. PDAL is different in
philosophy in a number of important ways:

1. All components of PDAL are released as open source software under an
   [OSI]-approved license.
2. PDAL allows application developers to provide proprietary extensions that
   act as stages in processing pipelines. These might be things like custom
   format readers, specialized exploitation algorithms, or entire processing
   pipelines.
3. PDAL can operate on point cloud data of any format
   -- not just [ASPRS LAS]. [LAStools] can read and write formats other than
   LAS, but relates all data to its internal handling of LAS data, limiting
   it to {ref}`dimension <dimensions>` types provided by the LAS format.
4. PDAL is coordinated by users with its declarative {ref}`JSON <pipeline>`
   syntax. LAStools is coordinated by linking lots of small, specialized
   command line utilities together with intricate arguments.
5. PDAL is an open source project, with all of its development activities
   available online at <https://github.com/PDAL/PDAL>

(about_pcl)=

### PCL

```{index} PCL
```

[PCL] is a complementary, rather than substitute, open source software
processing suite for point cloud data. The developer community of the PCL
library is focused on algorithm development, robotic and computer vision, and
real-time laser scanner processing. PDAL can read and write PCL's PCD format.

### Entwine

```{index} Entwine
```

[Entwine] is open source software from Hobu, Inc. that organizes massive point
cloud collections into streamable data services. These two software projects
allow province-scale LiDAR collections to be organized and served via HTTP
clients over the internet. PDAL provides {ref}`readers.ept` to allow users to
read data from those [Entwine Point Tile] collections that Entwine produces..

### Untwine

[Untwine] is open source software from Hobu, Inc. that organizes massive point
just like Entwine, but it does so in a bottom-up rather than top-down way.

```{index} Untwine
```

### viewer.copc.io

The Eptium viewer from Hobu, Inc. is a commercial lidar exploitation and
visualization platform based on Cesium that can be used to visualize [COPC](https://copc.io)
and [Entwine Point Tile] content.

### Potree

[Potree] is a [WebGL] HTML5 point cloud renderer that speaks [ASPRS LAS] and
[LASzip] compressed LAS. You can find the software at
<https://github.com/potree/potree/>

```{note}
See Potree in action using the USGS 3DEP AWS Public Dataset at
<https://usgs.entwine.io>
```

### Others

```{index} OrfeoToolbox, libLAS, CloudCompare, Fusion
```

Other open source point cloud softwares tend to be Desktop GUI, rather than
library, focused.  They include some processing operations, and sometimes they
even embed tools such as PDAL. We're obviously biased toward PDAL, but you
might find useful bits of functionality in them. These other tools include:

- [libLAS]
- [CloudCompare]
- [Fusion]
- [OrfeoToolbox]

```{note}
The [libLAS] project is an open source project that predates PDAL, and
provides some of the processing capabilities provided by PDAL. It is
currently in maintenance mode due to its dependence on LAS, the release of
relevant LAStools capabilities as open source, and the completion of
[Python LAS] software.
```

## Where did PDAL come from?

PDAL takes its cue from another very popular open source project -- {{ GDAL }}.
GDAL is Geospatial Data Abstraction Library, and it is used throughout the
geospatial software industry to provide translation and processing support for
a variety of raster and vector formats. PDAL provides the same capability for
point cloud data types.

PDAL evolved out of the development of database storage and access capabilities
for the {{ USACE }} [CRREL](https://www.erdc.usace.army.mil/Locations/CRREL/) {{ GRID }} project. Functionality that was creeping into {{ libLAS }}
was pulled into a new library, and it was designed from the ground up to mimic
successful extract, transform, and load libraries in the geospatial software
domain. PDAL has steadily attracted more contributors as other software
developers use it to provide point cloud data translation and processing
capability to their software.

### How is point cloud data different than raster or vector geo data?

Point cloud data are indeed very much like the typical vector point data type
of which many geospatial practitioners are familiar, but their volume causes
some significant challenges. Besides their `X`, `Y`, and `Z` locations, each
point often has full attribute information of other things like `Intensity`,
`Time`, `Red`, `Green`, and `Blue`.

Typical vector coverages of point data might max out at a million or so
features. Point clouds quickly get into the billions and even trillions, and
because of this specialized processing and management techniques must be used
to handle so much data efficiently.

The algorithms used to extract and exploit point cloud data are also
significantly different than typical vector GIS work flows, and data
organization is extremely important to be able to efficiently leverage the
available computing. These characteristics demand a library oriented toward
these approaches and PDAL achieves it.

```{note}
Possible point cloud dimension types provided and supported by PDAL
can be found at {ref}`dimensions`.
```

## What tasks are PDAL good at?

PDAL is great at point cloud data translation work flows. It allows users to
apply algorithms to data by providing an abstract API to the content -- freeing
users from worrying about many data format issues.  PDAL's format-free worry
does come with a bit of overhead cost. In most cases this is not significant,
but for specific processing work flows with specific data, specialized tools
will certainly outperform it.

In exchange for possible performance penalty or data model impedance,
developers get the freedom to access data over an abstract API, a multitude of
algorithms to apply to data within easy reach, and the most complete set of
point cloud format drivers in the industry. PDAL also provides a
straightforward command line, and it extends simple generic Python processing
through Numpy. These features make it attractive to software developers, data
managers, and scientists.

## What are PDAL's weak points?

PDAL doesn't provide a friendly GUI interface, it expects that you have the
confidence to dig into the options of {ref}`filters`, {ref}`readers`, and
{ref}`writers`. We sometimes forget that you don't always want to read source
code to figure out how things work. PDAL is an open source project in active
development, and because of that, we're always working to improve it. Please
visit {ref}`community` to find out how you can participate if you are
interested. The project is always looking for contribution, and the mailing
list is the place to ask for help if you are stuck.

## High Level Overview

PDAL is first and foremost a software library. A successful software library
must meet the needs of software developers who use it to provide its software
capabilities to their own software. In addition to its use as a software
library, PDAL provides some {ref}`command line applications <apps>` users can
leverage to conveniently translate, filter, and process data with PDAL.
Finally, PDAL provides {{ Python }} support in the form of embedded operations
and Python extensions.

### Core C++ Software Library

PDAL provides a {ref}`C++ API <api>` software developers can use to provide
point cloud processing capabilities in their own software. PDAL is
cross-platform C++, and it can compile and run on Linux, OS X, and Windows. The
best place to learn how to use PDAL's C++ API is the {ref}`test suite
<pdal_test>` and its [source code](https://github.com/PDAL/PDAL/tree/master/test/unit).

```{seealso}
PDAL {ref}`software <reading>` {ref}`development <writing>`
{ref}`tutorials <writing-reader>` have more information on how to
use the library from a software developer's perspective.
```

### Command Line Utilities

```{index} Command line, Apps
```

PDAL provides a number of {ref}`applications <apps>` that allow users to
coordinate and construct point cloud processing work flows. Some key tasks
users can achieve with these applications include:

- Print {ref}`info <info_command>` about a data set

- Data {ref}`translation <translate_command>` from one point cloud format to
  another

- Application of exploitation algorithms

  - Generate a DTM
  - Remove noise
  - Reproject from one coordinate system to another
  - Classify points as {ref}`ground/not ground <ground_command>`

- {ref}`Merge <merge_command>` or {ref}`split <split_command>` data

- {ref}`Catalog <tindex_command>` collections of data

```{note}
The command line utilities are often simply {ref}`pipeline_command` and
{ref}`pipeline` collected into a convenient application. In
many cases you can replicate the functionality of an application
entirely within a single pipeline.
```

### Python API

```{index} Numpy, Python
```

PDAL supports both embedding {{ Python }} and extending with {{ Python }}. These
allow you to dynamically interact with point cloud data in a more
comfortable and familiar language environment for geospatial practitioners.

```{seealso}
The {ref}`python` document contains information on how to
install and use the PDAL Python extension.
```

### Julia Plugin

```{index} Julia
```

PDAL supports embedding {{ Julia }} filters. These allow you to dynamically
interact with point cloud data in a more comfortable and familiar language
environment for geospatial practitioners, while still maintaining high
performance.

Additionally the TypedTables.jl, RoamesGeometry.jl and AcceleratedArrays.jl
libraries provide some very high-level interfaces for writing efficient
filters.

```{seealso}
The github repo at <https://github.com/cognitive-earth/PDAL-julia> contains
a docker image, build instructions and some sample filters.

Documentation for the stage {ref}`filters.julia`
```

## Conclusion

PDAL is an open source project for translating, filtering, and processing
point cloud data. It provides a C++ API, command line utilities, and Python
extensions. There are many open source software projects for interacting
with point cloud data, and PDAL's niche is in processing, translation,
and automation.

[asprs las]: https://www.asprs.org/divisions-committees/lidar-division/laser-las-file-format-exchange-activities
[cloudcompare]: https://www.danielgm.net/cc/
[entwine]: https://entwine.io
[entwine point tile]: https://entwine.io/entwine-point-tile.html
[fusion]: https://research.fs.usda.gov/pnw/products/dataandtools/tools/fusion/ldv-lidar-processing-and-visualization-software-version-4.40
[hobu, inc.]: https://hobu.co
[json]: https://en.wikipedia.org/wiki/JSON
[lastools]: https://lastools.org
[laszip]: https://laszip.org
[liblas]: https://liblas.org
[martin isenburg]: https://lidarmag.com/2021/10/30/in-memoriam-martin-isenburg-1972-2021/
[numpy]: https://numpy.org/
[orfeotoolbox]: https://www.orfeo-toolbox.org/
[osi]: https://opensource.org/licenses
[pcl]: https://pointclouds.org
[point cloud data]: https://en.wikipedia.org/wiki/Point_cloud
[potree]: https://potree.org
[python las]: https://pypi.python.org/pypi/laspy/
[untwine]: https://github.com/hobuinc/untwine
[virtual raster format]: https://gdal.org/en/latest/drivers/raster/vrt.html
[webgl]: https://en.wikipedia.org/wiki/WebGL

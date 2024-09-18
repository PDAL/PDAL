(home)=

# PDAL - Point Data Abstraction Library

```{image} ./_static/pdal_logo.png
:align: right
:alt: PDAL logo
```

PDAL is a C++ library for translating and manipulating [point cloud
data][point cloud data].  It is very much like the [GDAL] library which handles raster and
vector data.  The {ref}`about` page provides high level overview of the library
and its philosophy. Visit {ref}`readers` and {ref}`writers` to list data
formats it supports, and see {ref}`filters` for filtering operations that you
can apply with PDAL.

In addition to the library code, PDAL provides a suite of command-line
applications that users can conveniently use to process, filter, translate, and
query point cloud data.  {ref}`apps` provides more information on that topic.

Finally, PDAL speaks Python by both embedding and extending it. Visit
{ref}`python` to find out how you can use PDAL with Python to process point
cloud data.

The entire website is available as a single PDF at <http://pdal.io/_/downloads/en/latest/pdf/>

## News

### **09-05-2024**

PDAL 2.8.0 was released. Visit {ref}`download` to grab a copy.

### **06-28-2024**

PDAL 2.7.2 was released. Visit {ref}`download` to grab a copy.

### **02-05-2024**

PDAL 2.6.3 was released. Visit {ref}`download` to grab a copy.

### **08-18-2023**

PDAL 2.5.6 was released. Visit {ref}`download` to grab a copy.

### **01-13-2023**

PDAL 2.5.0 was released. Visit {ref}`download` to grab a copy. See it in
action in Jupyter by visiting Google Colab at <https://colab.research.google.com/drive/1JQpcVFFJYMrJCfodqP4Nc_B0_w6p5WOV?usp=sharing>

### **06-28-2021**

Howard Butler of [Hobu, Inc.](https://hobu.co) was recently interviewed by the
[MapScaping Podcast](https://mapscaping.com). Listen to the episode where he
talks about PDAL and point cloud processing.

<https://mapscaping.com/blogs/the-mapscaping-podcast/pdal-point-data-abstraction-library>

### **05-19-2021**

PDAL 2.2.1 has been released. You can {ref}`download <download>` the source
code or follow the {ref}`quickstart <quickstart>` to get going in a
hurry with Conda.

## Quickstart

```{toctree}
:maxdepth: 2

quickstart
```

## Applications

```{toctree}
:maxdepth: 2

apps/index
```

## Community

```{toctree}
:maxdepth: 2

community
```

## Drivers

```{toctree}
:glob: true
:maxdepth: 2

pipeline
stages/stages
stages/readers
stages/writers
stages/filters
```

## Dimensions

```{toctree}
:maxdepth: 2

dimensions
```

## Types

```{toctree}
:maxdepth: 2

types
```

## Python

```{toctree}
:maxdepth: 2

python
```

## Java

```{toctree}
:maxdepth: 2

java
```

## Tutorials

```{toctree}
:glob: true
:maxdepth: 2

tutorial/index
```

## Workshop

```{toctree}
:maxdepth: 2

Workshop <workshop/index>
```

## Development

```{toctree}
:maxdepth: 2

development/index
project/index
api/index
faq
copyright
```

```{toctree}
:includehidden: true

references
```

## Indices and tables

- {ref}`genindex`
- {ref}`search`

[brad chambers]: http://github.com/chambbj
[bsd]: http://www.opensource.org/licenses/bsd-license.php
[gdal]: http://www.gdal.org
[howard butler]: http://github.com/hobu
[lidar]: http://en.wikipedia.org/wiki/LIDAR
[point cloud data]: http://en.wikipedia.org/wiki/Point_cloud

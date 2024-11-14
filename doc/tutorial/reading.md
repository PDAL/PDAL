---
Author: Bradley Chambers
Contact: <mailto:brad.chambers@gmail.com>
Date: 01/21/2015
---

(reading)=

# Reading with PDAL

```{contents} Contents
:backlinks: none
:depth: 3
```

This tutorial is an introduction to using PDAL to read data using pdal
from the command line.

## A basic inquiry example

Our first example to demonstrate PDAL's utility will be to simply query an
[LAS] file to determine the data that are in it in the very first point.

```{note}
The [interesting.las] file in these examples can be found on github.
```

`pdal info` outputs JavaScript [JSON].

```
$ pdal info interesting.las -p 0
```

```javascript
{
  "filename": "interesting.las",
  "pdal_version": "1.0.1 (git-version: 80644d)",
  "points":
  {
    "point":
    {
      "Blue": 88,
      "Classification": 1,
      "EdgeOfFlightLine": 0,
      "GpsTime": 245381,
      "Green": 77,
      "Intensity": 143,
      "NumberOfReturns": 1,
      "PointId": 0,
      "PointSourceId": 7326,
      "Red": 68,
      "ReturnNumber": 1,
      "ScanAngleRank": -9,
      "ScanDirectionFlag": 1,
      "UserData": 132,
      "X": 637012,
      "Y": 849028,
      "Z": 431.66
    }
  }
}
```

## A conversion example

Conversion of data from one format to another may be lossy, in that some
data in the source format may not be representable in the same format or
at all in the destination format.  For example, some formats don't support
spatial references for point data, some have no metadata support and others
have limited {ref}`dimension <dimensions>` support.  Even when data types are
supported in both source and destination formats, there may be limitations
with regard to data type, precision or, scaling.  PDAL attempts to convert
data as accurately as possible, but you should make sure that you're
aware of the capabilities of the data formats you're using.

```
$ pdal translate interesting.las output.txt
```

```
"X","Y","Z","Intensity","ReturnNumber","NumberOfReturns","ScanDirectionFlag","EdgeOfFlightLine","Classification","ScanAngleRank","UserData","PointSourceId","Time","Red","Green","Blue"
637012.24,849028.31,431.66,143,1,1,1,0,1,-9,132,7326,245381,68,77,88
636896.33,849087.70,446.39,18,1,2,1,0,1,-11,128,7326,245381,54,66,68
636784.74,849106.66,426.71,118,1,1,0,0,1,-10,122,7326,245382,112,97,114
636699.38,848991.01,425.39,100,1,1,0,0,1,-6,124,7326,245383,178,138,162
636601.87,849018.60,425.10,124,1,1,1,0,1,-4,126,7326,245383,134,104,134
636451.97,849250.59,435.17,48,1,1,0,0,1,-9,122,7326,245384,99,85,95
...
```

The text format supports all point attributes, but provides no support for
metadata such as the input spatial reference system or the [LAS] header
fields, such as [UUID].
You may need to preserve some more information as part of
your conversion to make it useful down the road.

### Metadata

PDAL carries {ref}`metadata <metadata>` for each stage through the PDAL
{ref}`processing pipeline <pipeline>`.  The metadata can be written in
JSON form using the pdal {ref}`info <info_command>` command

```
$ pdal info --metadata interesting.las
```

This produces metadata that looks like
[this](../_images/info-interesting-metadata.png). You can use
your [JSON] manipulation tools to extract this information.
For formats that do not have the ability to
preserve this metadata internally, you can keep a `.json` file
alongside the `.txt` file as auxiliary information.

## A Pipeline Example

The full power of PDAL comes in the form of {ref}`pipeline_command` invocations.
Pipelines allow you to take advantage of PDAL's ability to manipulate data
as they are converted. This section will provide a basic example and
demonstration of pipeline usage.  See the
{ref}`pipeline specification <pipeline>`, for more detailed exposition of the
topic.

The {ref}`pipeline_command` describes a series of processing stages to
be performed in JSON format.  Each stage can be provided a set of options
that control the details of processing. PDAL is single-threaded and stages
are executed in a linear order.  Some stages support what is known as
"stream mode".  If all stages in a pipeline support stream mode the command
is run using using stream mode to reduce the memory processing footprint.
Even when run in stream mode, execution is single-threaded and can be
thought of as linear.

### Simple conversion

The following [JSON] document defines a pipeline that takes the `file.las`
[LAS] file and converts it to a new file called `output.las`.

```json
[
    "file.las",
    "output.las"
]
```

### Loop a directory and filter it through a pipeline

This bash script loops through a directory and pushes the las files through
a pipeline, substituting the input and output as it goes.

```
ls *.las | cut -d. -f1 | xargs -P20 -I{} pdal pipeline -i /path/to/proj.json --readers.las.filename={}.las --writers.las.filename=output/{}.laz
```

Here is an example doing something similar with Windows PowerShell

```
$indir="Documents\inlas"
$outdir="Documents\outlas"
get-childitem $indir |
foreach-object {
    if ($_.extension -ne ".las") {
        continue
    }
    $outname = $outdir + "\" + $_.name
    pdal pipeline -i \path\to\proj.json $_.fullname $outname
}
```

[interesting.las]: https://github.com/PDAL/PDAL/blob/master/test/data/las/interesting.las?raw=true
[json]: http://www.json.org/
[las]: https://www.asprs.org/divisions-committees/lidar-division/laser-las-file-format-exchange-activities
[uuid]: http://en.wikipedia.org/wiki/Universally_unique_identifier

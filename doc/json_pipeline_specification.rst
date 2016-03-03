.. _json_pipeline_specification:

======================================
Draft PDAL JSON Pipeline Specification
======================================

:Authors:
    Bradley J. Chambers,
    Connor Manning,
    Andrew Bell,
    Howard Butler

:Revision: 0.1
:Date: 22 December 2015

The PDAL JSON specification is a point cloud processing pipeline interchange
format based on JavaScript Object Notation (JSON).

.. sectnum::
.. contents::
   :depth: 4
   :backlinks: none

============
Introduction
============

A PDAL JSON object represents a processing pipeline.

A complete PDAL JSON data structure is always an object (in JSON terms). In PDAL
JSON, an object consists of a collection of name/value pairs -- also called
members. For each member, the name is always a string. Member values are either
a string, number, object, array or one of the literals: "true", "false", and
"null". An array consists of elements where each element is a value as
described above.

Examples
--------

A simple PDAL pipeline, inferring the appropriate drivers for the reader and
writer from filenames, and able to be specified as a set of sequential steps:

.. code-block:: json

  {
      "pipeline":[
          "input.las",
          {
              "type":"crop",
              "bounds":"([0,100],[0,100])"
          },
          "output.bpf"
      ]
  }

A more complex PDAL pipeline, that reprojects the stage tagged ``A1``, merges
the result with ``B``, and writes the merged output with the ``points2grid``
plugin.:

.. code-block:: json

  {
      "pipeline":[
          {
              "filename":"A.las",
              "spatialreference":"EPSG:26916"
          },
          {
              "type":"filters.reprojection",
              "in_srs":"EPSG:26916",
              "out_srs":"EPSG:4326",
              "tag":"A2"
          },
          {
              "filename":"B.las",
              "tag":"B"
          },
          {
              "type":"filters.merge",
              "tag":"merged",
              "inputs":[
                  "A2",
                  "B"
              ]
          },
          {
              "type":"writers.p2g",
              "filename":"output.tif"
          }
      ]
  }

Definitions
-----------

* JavaScript Object Notation (JSON), and the terms object, name, value, array,
  and number, are defined in IETF RTC 4627, at
  http://www.ietf.org/rfc/rfc4627.txt.

* The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT", "SHOULD",
  "SHOULD NOT", "RECOMMENDED", "MAY", and "OPTIONAL" in this documention are to
  be interpreted as described in IETF RFC 2119, at
  http://www.ietf.org/rfc/rfc2119.txt.

=====================
PDAL Pipeline Objects
=====================

PDAL JSON pipelines always consist of a single object. This object (referred to
as the PDAL JSON object below) represents a processing pipeline.

* The PDAL JSON object may have any number of members (name/value pairs).

* The PDAL JSON object must have a :ref:`pipeline_array`.

.. _pipeline_array:

Pipeline Array
--------------

* The pipeline array may have any number of string or :ref:`stage_object`
  elements.

* String elements shall be interpreted as filenames. PDAL will attempt to infer
  the proper driver from the file extension and position in the array. A writer
  stage will only be created if the string is the final element in the array.

.. _stage_object:

Stage Objects
-------------

For more on PDAL stages and their options, check the PDAL documentation on
`Readers, Writers, and Filters <http://www.pdal.io/stages/index.html>`_.

* A stage object may have a member with the name ``tag`` whose value is a
  string. The purpose of the tag is to cross-reference this stage within other
  stages. Each ``tag`` must be unique.

* A stage object may have a member with the name ``inputs`` whose value is an
  array of strings. Each element in the array is the tag of another stage to be
  set as input to the current stage.

* Reader stages will disregard the ``inputs`` member.

* If ``inputs`` is not specified for the first non-reader stage, all reader
  stages leading up to the current stage will be used as inputs.

* If ``inputs`` is not specified for any subsequent non-reader stages, the
  previous stage in the array will be used as input.

* A ``tag`` mentioned in another stage's ``inputs``  must have been previously
  defined in the ``pipeline`` array.

* A reader or writer stage object may have a member with the name ``type`` whose
  value is a string. The ``type`` must specify a valid PDAL reader or writer
  name.

* A filter stage object must have a member with the name ``type`` whose value is
  a string. The ``type`` must specify a valid PDAL filter name.

* A stage object may have additional members with names corresponding to
  stage-specific option names and their respective values.

Filename Globbing
-----------------

* A filename may contain the wildcard character ``*`` to match any string of
  characters. This can be useful if working with multiple input files in a
  directory (e.g., merging all files).

=================
Extended Examples
=================

BPF to LAS
----------

The following pipeline converts the input file from BPF to LAS, inferring both
the reader and writer type, and setting a number of options on the writer stage.

.. code-block:: json

  {
      "pipeline":[
          "utm15.bpf",
          {
              "filename":"out2.las",
              "scale_x":0.01,
              "offset_x":311898.23,
              "scale_y":0.01,
              "offset_y":4703909.84,
              "scale_z":0.01,
              "offset_z":7.385474
          }
      ]
  }

Python HAG
----------

In our next example, the reader and writer types are once again inferred. After
reading the input file, the ferry filter is used to copy the Z dimension into a
new height above ground (HAG) dimension. Next, the programmable filter is used
with a python script to compute height above ground values by comparing the Z
values to a surface model. These height above ground values are then written
back into the Z dimension for further analysis.

.. code-block:: json

  {
      "pipeline":[
          "autzen.las",
          {
              "type":"ferry",
              "dimensions":"Z=HAG"
          },
          {
              "type":"programmable",
              "script":"hag.py",
              "function":"filter",
              "module":"anything"
          },
          "autzen-hag.las"
      ]
  }

DTM
---

A common task is to create a digital terrain model (DTM) from the input point
cloud. This pipeline infers the reader type, applies an approximate ground
segmentation filter, and then creates the DTM using the Points2Grid writer with
only the ground returns.

.. code-block:: json

  {
      "pipeline":[
          "autzen-full.las",
          {
              "type":"ground",
              "approximate":true,
              "max_window_size":33,
              "slope":1.0,
              "max_distance":2.5,
              "initial_distance":0.15,
              "cell_size":1.0,
              "extract":true,
              "classify":false
          },
          {
              "type":"p2g",
              "filename":"autzen-surface.tif",
              "output_type":"min",
              "output_format":"tif",
              "grid_dist_x":1.0,
              "grid_dist_y":1.0
          }
      ]
  }

Decimate & Colorize
-------------------

This example still infers the reader and writer types while applying options on
both. The pipeline decimates the input LAS file by keeping every other point,
and then colorizes the points using the provided raster image. The output is
written as ASCII text.

.. code-block:: json

  {
      "pipeline":[
          {
              "filename":"1.2-with-color.las",
              "spatialreference":"EPSG:2993"
          },
          {
              "type":"decimation",
              "step":2,
              "offset":1
          },
          {
              "type":"colorization",
              "raster":"autzen.tif",
              "dimensions":"Red:1:1, Green:2:1, Blue:3:1"
          },
          {
              "filename":"junk.txt",
              "delimiter":",",
              "write_header":false
          }
      ]
  }

Merge & Reproject
-----------------

Our first example with multiple readers, this pipeline infers the reader types,
and assigns spatial reference information to each. Next, the merge filter merges
points from all previous readers, and the reprojection filter reprojects data to
the specified output spatial reference system.

.. code-block:: json

  {
      "pipeline":[
          {
              "filename":"1.2-with-color.las",
              "spatialreference":"EPSG:2027"
          },
          {
              "filename":"1.2-with-color.las",
              "spatialreference":"EPSG:2027"
          },
          {
              "type":"merge"
          },
          {
              "type":"reprojection",
              "out_srs":"EPSG:2028"
          }
      ]
  }

Globbed Inputs
--------------

Finally, we capture another merge pipeline demonstrating the ability to glob
multiple input LAS files from a given directory.

.. code-block:: json

  {
      "pipeline":[
          "/path/to/data/*.las",
          {
              "type":"merge"
          },
          "output.las"
      ]
  }

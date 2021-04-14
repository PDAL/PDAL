.. _writers.draco:

writers.draco
=============

`Draco`_ is a library for compressing and decompressing 3D geometric meshes and
point clouds and was designed and built for compression efficiency and speed.
The code supports compressing points, connectivity information, texture coordinates,
color information, normals, and any other generic attributes associated with geometry.

This writer aims to use the encoding feature of the Draco library to compress and
output Draco files.

Example
--------------------------------------------------------------------------------

This example will read in a las file and output a Draco encoded file, with options
to include PDAL dimensions X, Y, and Z as double, and explicitly setting quantization
levels of some of the Draco attributes.

.. code-block:: json

[
    {
        "type": "readers.las",
        "filename": "color.las"
    },
    {
        "type": "writers.draco",
        "filename": "draco.drc",
        "dimensions": {
            "X": "float",
            "Y": "float",
            "Z": "float"
        },
        "quantization": {
            "NORMAL": 8,
            "TEX_COORD": 7,
            "GENERIC": 6
        }
    }
]

Options
-------

filename
    Output file name. [Required]

dimensions
    A json map of PDAL dimensions to desired data types. Data types must be string
    and must be available in `PDAL's Type specification`_. Any dimension that
    combine to make one Draco dimension must all have the same type (eg. POSITION is
    made up of X, Y, and Z. X cannot by float while Y and Z are specified as double)

    This argument will filter the dimensions being written to only the dimensions
    that have been specified. If that dimension is part of a multi-dimensional
    draco attribute (POSITION=[X,Y,Z]), then any dimension not specified will be
    filled in with zeros.

quantization
    A json map of Draco attributes to desired quantization levels. These levels
    must be integers. Default quantization levels are below, and will be
    overridden by any values placed in the options.

..code-block:: json
{
    "POSITION": 11,
    "NORMAL": 7,
    "TEX_COORD": 10,
    "COLOR": 8,
    "GENERIC": 8
}

.. include:: writer_opts.rst

.. _PDAL's Type specification: https://github.com/PDAL/PDAL/blob/master/pdal/DimUtil.hpp
.. _Draco: https://github.com/google/draco

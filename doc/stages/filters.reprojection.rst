.. _filters.reprojection:

filters.reprojection
===========================

The reprojection filter converts the X, Y and/or Z dimensions to a new spatial
reference system. The old coordinates are replaced by the new ones,
if you want to preserve the old coordinates for future processing, use a
:ref:`filters.ferry` to create a new dimension and stuff them there.

.. note::

    X, Y, and Z dimensions in PDAL are carried as doubles, with their
    scale information applied. Set the output scale (`scale_x`, `scale_y`, or
    `scale_z`) on your writer to descale the data on the way out.

    Many LIDAR formats store coordinate information in 32-bit address spaces, and
    use scaling and offsetting to ensure that accuracy is not lost while fitting
    the information into a limited address space. When changing projections, the
    coordinate values will change, which may change the optimal scale and offset
    for storing the data.


Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "filename":"input.las",
          "type":"readers.las",
          "spatialreference":"EPSG:26916"
        },
        {
          "type":"filters.range",
          "limits":"Z[0:100],Classification[2:2]"
        },
        {
          "type":"filters.reprojection",
          "in_srs":"EPSG:26916",
          "out_srs":"EPSG:4326"
        },
        {
          "type":"writers.las",
          "scale_x":"0.0000001",
          "scale_y":"0.0000001",
          "scale_z":"0.01",
          "offset_x":"auto",
          "offset_y":"auto",
          "offset_z":"auto",
          "filename":"example-geog.las"
        }
      ]
    }

Options
-------

in_srs
  Spatial reference system of the input data. Express as an EPSG string (eg
  "EPSG:4326" for WGS86 geographic) or a well-known text string. [Required if
  input reader does not supply SRS information]

out_srs
  Spatial reference system of the output data. Express as an EPSG string (eg
  "EPSG:4326" for WGS86 geographic) or a well-known text string. [Required]


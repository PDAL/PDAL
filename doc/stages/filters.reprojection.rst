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

.. embed::

.. streamable::

Example 1
--------------------------------------------------------------------------------

This pipeline reprojecs terrain points with Z-values between 0 and 100 by first
applying a range filter and then specifing both the input and output spatial
reference as EPSG-codes. The X and Y dimensions are scaled to allow enough
precision in the output coordinates.

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

Example 2
--------------------------------------------------------------------------------

In some cases it is not possible to use a EPSG-code as a spatial reference.
Instead `Proj.4 <http:/proj4.org>`_ parameters can be used to define a spatial
reference.  In this example the vertical component of points in a laz file is
converted from geometric (ellipsoidal) heights to orthometric heights by using
the ``geoidgrids`` parameter from Proj.4.  Here we change the vertical datum
from the GRS80 ellipsoid to DVR90, the vertical datum in Denmark. In the
writing stage of the pipeline the spatial reference of the file is set to
EPSG:7416. The last step is needed since PDAL will otherwise reference the
vertical datum as "Unnamed Vertical Datum" in the spatial reference VLR.


.. code-block:: json
    :linenos:

    {
      "pipeline":[
        "./1km_6135_632.laz",
        {
            "type":"filters.reprojection",
            "in_srs":"EPSG:25832",
            "out_srs":"+init=epsg:25832 +geoidgrids=C:/data/geoids/dvr90.gtx"
        },
        {
          "type":"writers.las",
          "a_srs":"EPSG:7416",
          "filename":"1km_6135_632_DVR90.laz"
        }
      ]
    }

Options
-------

in_srs
  Spatial reference system of the input data. Express as an EPSG string (eg
  "EPSG:4326" for WGS84 geographic), Proj.4 string or a well-known text string. [Required if
  input reader does not supply SRS information]

out_srs
  Spatial reference system of the output data. Express as an EPSG string (eg
  "EPSG:4326" for WGS84 geographic), Proj.4 string or a well-known text string. [Required]


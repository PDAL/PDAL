.. _writers.derivative:

writers.derivative
==================

The **Derivative Writer** supports writing of primary topographic attributes.


.. note::
    This driver uses `GDAL`_ to write the data. Only the `GeoTIFF`_ driver
    is supported at this time.

.. _`GDAL`: http://gdal.org
.. _`GeoTiff`: http://www.gdal.org/frmt_gtiff.html

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "inputfile.las",
        {
          "type":"writers.derivative",
          "filename":"outputfile.tiff",
          "primitive_type":"slope_d8"
        }
      ]
    }


Options
-------

filename
  `GeoTiff`_ file to write.  [Required]

primitive_type
  Topographic attribute to compute.  [Default: slope_d8]

  * slope_d8
  * slope_fd
  * aspect_d8
  * aspect_fd
  * contour_curvature
  * profile_curvature
  * tangential_curvature
  * total_curvature
  * hillshade
  * catchment_area

grid_dist_x, grid_dist_y
  Size of grid cell in X and Y dimensions using native units of the input point
  cloud.  [Default: 15.0]

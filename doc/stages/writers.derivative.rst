.. _writers.derivative:

writers.derivative
==================

The **Derivative Writer** supports writing of primary topographic attributes.


.. note::
    This driver uses `GDAL`_ to write the data. Only the `GeoTIFF`_ driver
    is supported at this time.

.. _`GDAL`: http://gdal.org
.. _`GeoTiff`: http://www.gdal.org/frmt_gtiff.html

Example #1
----------

Create a single GeoTIFF with slope values calculated using the D8 method.

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
    
Example #2
----------

Create multiple GeoTIFFs containing slope, hillshade, and contour curvature
values.

.. code-block:: json

    {
      "pipeline":[
        "inputfile.las",
        {
          "type":"writers.derivative",
          "filename":"outputfile_#.tiff",
          "primitive_type":"slope_d8,hillshade,contour_curvature"
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

edge_length
  Size of grid cell in X and Y dimensions using native units of the input point
  cloud.  [Default: 15.0]

altitude
  Illumination altitude in degrees (hillshade only). [Default: 45.0]

azimuth
  Illumination azimuth in degrees (hillshade only). [Default: 315.0]

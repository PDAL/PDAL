.. _writers.derivative:

writers.derivative
==================

The **Derivative Writer** supports writing of primary topographic attributes.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.derivative">
      <Option name="filename">
        outputfile.tiff
      </Option>
      <Option name="primitive_type">
        slope_d8
      </Option>
      <Reader type="readers.las">
        <Option name="filename">
          inputfile.las
        </Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  GeoTiff file to write.  [Required]

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

.. _filters.faceraster:

filters.faceraster
================================================================================

The **FaceRaster filter** creates a raster from a point cloud using an
algorithm based on an existing triangulation.  Each raster cell
is given a value that is an interpolation of the known values of containing
triangle.  If the raster cell center is outside of the triangulation, it is
assigned the nodata_ value.  Use `writers.raster` to write the output.

The extent of the raster can be defined by using the origin_x_, origin_y, width_ and
height_ options. If these options aren't provided the raster is sized to contain the
input data.

.. embed::


Basic Example
--------------------------------------------------------------------------------

This  pipeline reads the file autzen_trim.las and creates a raster based on a
Delaunay trianguation of the points. It then creates a raster, interpolating values
based on the vertices of the triangle that contains each raster cell center.

.. code-block:: json

  [
      "pdal/test/data/las/autzen_trim.las",
      {
          "type": "filters.delaunay"
      },
      {
          "type": "filters.faceraster",
          "resolution": 2,
          "width": 500,
          "height": 500,
          "origin_x": 636000,
          "origin_y": 849000
      }
  ]


Options
--------------------------------------------------------------------------------

.. _resolution:

resolution
    Length of raster cell edges in X/Y units.  [Required]

_`nodata`
    The value to use for a raster cell if no data exists in the input data
    with which to compute an output cell value. Note that this value may be
    different from the value used for nodata when the raster is written.
    [Default: NaN]

mesh
  Name of the triangulation to use for interpolation.  If not provided, the first
  triangulation associated with the input points will be used. [Default: None]
 
_`origin_x`
  X origin (lower left corner) of the grid. [Default: None]

_`origin_y`
  Y origin (lower left corner) of the grid. [Default: None]

_`width`
  Number of cells in the X direction. [Default: None]

_`height`
  Number of cells in the Y direction. [Default: None]

.. include:: filter_opts.rst


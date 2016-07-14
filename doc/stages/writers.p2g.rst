.. _writers.p2g:

writers.p2g
===========

The **points to grid writer** takes in a stream of point data and writes out
gridded summaries of the stream. Each cell in the output grids can give one of
the: minimum value, maximum value, average value, average value, inverse
distance weighted interpolation (for sparse points), or density. The points to
grid writer supports creating multiple output grids simultaneously, so it is
possible to generate all grid variants in one pass.


.. note::

    A project called `lidar2dems`_ by `Applied GeoSolutions`_ integrates the P2G
    writer and other PDAL components into a series of scripts and utilities that
    make it more convenient to do DEM production with PDAL.

.. _`lidar2dems`: https://github.com/Applied-GeoSolutions/lidar2dems
.. _`Applied GeoSolutions`: http://www.appliedgeosolutions.com/

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.las",
          "filename":"inputfile.las"
        },
        {
          "type":"writers.p2g",
          "grid_dist_x":"6.0",
          "grid_dist_y":"6.0",
          "radius":"8.4852813742385713",
          "filename":"autzen_grid",
          "output_type":"min",
          "output_type":"max",
          "output_type":"mean",
          "output_type":"idw",
          "output_type":"den",
          "output_format":"asc",
        }
      ]
    }

Options
-------

grid_dist_x
  Size of grid cell in x dimension [Default: **6**]

grid_dist_y
  Size of grid cell in y dimension. [Default: **6**]

radius
  ??? [Default: **8.48528**]

filename
  Base file name for output files. [Required]

output_type
  One or many options, specifying "min", "max", "mean", "idw" (inverse distance weighted), "den" (density), or "all" to get all variants with just one option. [Default: **all**]

output_format
  File output format to use, one of "grid", "tif", or "asc". [Default: **grid**]

z
  Name of the 'z' dimension to use. [Default: 'Z']

bounds
  Custom bounds for output raster(s).
  If not provided, bounds will be calculated from the bounds of the input data.
  [Default: **none**]

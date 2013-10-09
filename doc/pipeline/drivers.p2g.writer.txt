.. _drivers.p2g.writer:

drivers.p2g.writer
==================

The **points to grid writer** takes in a stream of point data and writes out gridded summaries of the stream. Each cell in the output grids can give one of the: minimum value, maximum value, average value, average value, inverse distance weighted interpolation (for sparse points), or density. The points to grid writer supports creating multiple output grids simultaneously, so it is possible to generate all grid variants in one pass.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.p2g.writer">
      <Option name="grid_dist_x">
        6.0
      </Option>
      <Option name="grid_dist_y">
        6.0
      </Option>
      <Option name="radius">
        8.4852813742385713
      </Option>
      <Option name="filename">
        autzen_grid
      </Option>
      <Option name="output_type">
        min
      </Option>
      <Option name="output_type">
        max
      </Option>
      <Option name="output_type">
        mean
      </Option>
      <Option name="output_type">
        idw
      </Option>
      <Option name="output_type">
        den
      </Option>
      <Option name="output_format">
        grid
      </Option>
        <Filter type="filters.inplacereprojection">
          <Option name="out_srs">
            EPSG:26910
          </Option>
          <Option name="scale_x">
            0.01
          </Option>
          <Option name="scale_y">
            0.01
          </Option>
          <Reader type="drivers.las.reader">
            <Option name="filename">
              ../1.2-with-color.las
            </Option>
            <Option name="spatialreference">
              ../1.2-with-color.las.wkt
            </Option>
          </Reader>
      </Filter>
    </Writer>
  </Pipeline>

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
  File output format to use, one of "grid" or "asc". [Default: **grid**]
  

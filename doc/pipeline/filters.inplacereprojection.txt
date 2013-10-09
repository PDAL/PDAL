.. _filters.inplacereprojection:

filters.inplacereprojection
===========================

"In place" reprojection converts the X, Y and/or Z dimensions to a new spatial reference system, and optionally applies a new scaling and offset to the coordinates in the process. The old coordinates are replaced by the new ones, and the old dimensions replaced in the schema by the new ones.

When reprojecting, coordinate values are scaled and offset into their true values in double precision floating point format. The coordinate reference system transformation is applied. Then the values are un-offset and un-scaled into their new dimensions.

Many LIDAR formats store coordinate information in 32-bit address spaces, and use scaling and offsetting to ensure that accuracy is not lost while fitting the information into a limited address space. When changing projections, the coordinate values will change, which may change the optimal scale and offset for storing the data.

For example, data in UTM might have a scaling of 0.01 (for centimeter accuracy) and offsets of 200000 in X and 4000000 in Y. When converted to a geographic coordinate reference, a scaling of 0.0000001 is required to maintain centimeter accuracy, and the offsets should be close to the lower left of the data extent to help ensure small positive values.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
    <Option name="filename">example-geog.las</Option>
    <Option name="spatialreference">EPSG:4326</Option>
      <Filter type="filters.inplacereprojection">
        <Option name="in_srs">EPSG:26916</Option>
        <Option name="out_srs">EPSG:4326</Option>
        <Option name="offset_x">-100</Option>
        <Option name="offset_y">40</Option>
        <Option name="offset_z">0</Option>
        <Option name="scale_x">0.0000001</Option>
        <Option name="scale_y">0.0000001</Option>
        <Option name="scale_z">0.1</Option>
        <Reader type="drivers.las.reader">
          <Option name="filename">example-utm.las</Option>
          <Option name="spatialreference">EPSG:26916</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

Options
-------

in_srs
  Spatial reference system of the input data. Express as an EPSG string (eg "EPSG:4326" for WGS86 geographic) or a well-known text string. [Required if input reader does not supply SRS information]

out_srs
  Spatial reference system of the output data. Express as an EPSG string (eg "EPSG:4326" for WGS86 geographic) or a well-known text string. [Required]

x_dim
  Specify the name of the dimension to use as the "X" dimension in the coordinate transformation. [Default: **X**]

y_dim
  Specify the name of the dimension to use as the "Y" dimension in the coordinate transformation. [Default: **Y**]

z_dim
  Specify the name of the dimension to use as the "Z" dimension in the coordinate transformation. [Default: **Z**]
  
offset_x, offset_y, offset_z
  Output offsets of the X, Y and Z dimensions respectively. When changing from projected to geographic, or vice versa, remember to set an output offset appropriate to the final coordinate range. [Default: Offset of the input dimension]

scale_x, scale_y, scale_z
  Output scales of the X, Y and Z dimensions respectively. When changing from projected to geographic, or vice versa, remember to set an output scale appropriate to the final coordinate magnitude. [Default: Scale of the input dimension]

ignore_old_dimensions
  Marking original dimensions as ignored will cause them to not be written out by the writer at the end of the chain. [Default: **true**]
  
do_offset_z
  TDB [Default: **false**]

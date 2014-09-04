.. _filters.crop:

filters.crop
============

The cropping filter takes a stream of points and removes points that fall outside the cropping bounds, or an optional cropping polygon. The cropping filter requires a bounds or a polygon to crop with.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">
        file-cropped.las
      </Option>
      <Filter type="filters.crop">
        <Option name="bounds">
          ([0,1000000],[0,1000000],[0,1000000])
        </Option>
        <Reader type="drivers.las.reader">
          <Option name="filename">
            file-input.las
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>


Options
-------

bounds
  The extent of the clipping rectangle, expressed in a string, eg: *([xmin, xmax], [ymin, ymax], [zmin, zmax])*
  
polygon
  The clipping polygon, expressed in a well-known text string, eg: *POLYGON((0 0, 5000 10000, 10000 0, 0 0))* 
  
x_dim
  The name of the dimension to use as the X coordinate in the cropping process. [Default: **X**]
  
y_dim
  The name of the dimension to use as the Y coordinate in the cropping process. [Default: **Y**]

z_dim
  The name of the dimension to use as the Z coordinate in the cropping process. [Default: **Z**]

outside
  Invert the cropping logic and only take points **outside** the cropping bounds or polygon. [Default: **false**]
  

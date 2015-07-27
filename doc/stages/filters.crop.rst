.. _filters.crop:

filters.crop
============

The crop filter removes points that fall outside or inside a cropping bounding
box (2D)
or polygon.  If more than one bounding region is specified, the filter will
pass all input points through each bounding region, creating an output point
set for each input crop region.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        file-cropped.las
      </Option>
      <Filter type="filters.crop">
        <Option name="bounds">
          ([0,1000000],[0,1000000])
        </Option>
        <Reader type="readers.las">
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
  The extent of the clipping rectangle, expressed in a string, eg: *([xmin, xmax], [ymin, ymax])*
  
polygon
  The clipping polygon, expressed in a well-known text string, eg: *POLYGON((0 0, 5000 10000, 10000 0, 0 0))* 
  
outside
  Invert the cropping logic and only take points **outside** the cropping bounds or polygon. [Default: **false**]
  

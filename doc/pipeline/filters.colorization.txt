.. _filters.colorization:

filters.colorization
====================

The colorization filter populates dimensions in the point buffer using input values read from a raster file. Commonly this is used to add Red/Green/Blue values to points from an aerial photograph of an area. However, any band can be read from the raster and applied to any dimension name desired.

.. figure:: filters.colorization.img1.jpg
    :scale: 50 %
    :alt: Points after colorization

    After colorization, points take on the colors provided by the input image

The bands of the raster to apply to each are selected using the "band" option, and the values of the band may be scaled before being written to the dimension. If the band range is 0-1, for example, it might make sense to scale by 256 to fit into a traditional 1-byte color value range.

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">colorized.las</Option>
      <Filter type="filters.colorization">
        <Option name="dimension">
          Red
          <Options>
            <Option name="band">1</Option>
            <Option name="scale">1.0</Option>
          </Options>
        </Option>
        <Option name="dimension">
          Green
          <Options>
              <Option name="band">2</Option>
              <Option name="scale">1.0</Option>
          </Options>
        </Option>
        <Option name="dimension">
          Blue
          <Options>
            <Option name="band">3</Option>
            <Option name="scale">256</Option>
          </Options>
        </Option>
        <Option name="raster">aerial.tif</Option>
        <Reader type="drivers.las.reader">
          <Option name="filename">uncolored.las</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>



Options
-------

raster
  The raster file to read the band from. Any format supported by `GDAL`_ may be read.

dimension
  A dimension to populate with values from the raster file. There may be multiple dimension options declared. The dimension name should be supplied, and an options list indicating the raster band to read from and the scaling to apply.
  
  band
    The raster band from which to read values for this dimension.
  
  scale
    The scaling factor to apply to the value read from the raster. [Default: **1.0**]

x_dim
  The point dimension to use for the x dimension [Default: **X**]
  
y_dim
  The point dimension to use for the y dimension [Default: **Y**]


.. _GDAL: http://gdal.org

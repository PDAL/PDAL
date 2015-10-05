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
    <Writer type="writers.las">
      <Option name="filename">colorized.las</Option>
      <Filter type="filters.colorization">
        <Option name="dimensions">
          Red:1:1.0, Blue, Green::256.0
        </Option>
        <Option name="raster">aerial.tif</Option>
        <Reader type="readers.las">
          <Option name="filename">uncolored.las</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>


Options
-------

raster
  The raster file to read the band from. Any format supported by `GDAL`_ may be read.

dimensions
  A comma separated list of dimensions to populate with values from the raster
  file. The format of each dimension is <name>:<band_number>:<scale_factor>.
  Either or both of band number and scale factor may be omitted as may ':'
  separators if the data is not ambiguous.  If not supplied, band numbers
  begin at 1 and increment from the band number of the previous dimension.
  If not supplied, the scaling factor is 1.0.
  [Default: "Red:1:1.0, Green:2:1.0, Blue:3:1.0"]
  
.. _GDAL: http://gdal.org

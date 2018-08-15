.. _filters.dem:

filters.dem
===============================================================================


:ref:`filters.dem` uses a raster to keep data within a range of the raster cell. For
example, atmospheric or MTA noise in a scene can be quickly removed by keeping all
data within 100m above and 20m below a pre-existing elevation model.

.. embed::

Example #1
----------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"filters.dem",
          "raster":"dem.tif",
          "limits":"Z[20:100]"
        }
      ]
    }

Options
-------------------------------------------------------------------------------

limits
  A :ref:`Range <ranges>` that defines the dimension and the magnitude above
  and below the value of the given dimension to filter.

  The following example would keep all ``Z`` point cloud values that are
  within 100 units above and 20 units below the elevation model value at the
  given ``X`` and ``Y`` value.

  ::

      Z[20:100]

raster
  `GDAL readable raster`_ data to use for filtering.

band
  GDAL Band number to read (count from 1) [Default: **1**]

.. _`GDAL`: http://gdal.org
.. _`GDAL readable raster`: http://www.gdal.org/formats_list.html

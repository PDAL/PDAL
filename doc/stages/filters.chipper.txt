.. _filters.chipper:

filters.chipper
===============

The chipper filter takes a single large point cloud and converts it into a set of smaller clouds, or chips. The chips are all spatially contiguous and non-overlapping, so the result is a an irregular tiling of the input data.

.. figure:: filters.chipper.img1.png
    :scale: 100 %
    :alt: Points before chipping

    Before chipping, the points are all in one collection.


.. figure:: filters.chipper.img2.png
    :scale: 100 %
    :alt: Points after chipping

    After chipping, the points are tiled into smaller contiguous chips.
   
Chipping is usually applied to data read from files (which produce one large stream of points) before the points are written to a database (which prefer data segmented into smaller blocks). The chipper filter is often used in conjunction with :ref:`filters.cache` to improve chipping performance when there is sufficient memory available.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.pgpointcloud.writer">
      <Option name="connection">dbname='lidar' user='user'</Option>
      <Filter type="filters.chipper">
        <Option name="capacity">400</Option>
        <Reader type="drivers.las.reader">
            <Option name="filename">example.las</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

Options
-------

capacity
  How many points to fit into each chip. The number of points in each chip will not exceed this value, and will sometimes be less than it. [Default: **5000**]
  

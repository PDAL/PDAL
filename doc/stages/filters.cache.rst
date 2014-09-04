.. _filters.cache:

filters.cache
=============

The cache filter holds the results of all previous filters and readers in memory, allowing filters further downstream to randomly access points in the buffer much more efficiently than would otherwise be possible.

The most frequent use of the :ref:`filters.cache` is in front of :ref:`filters.chipper`, because the chipping process requires a great deal of random access to the point buffer. When using the filter, it is best to allocate use a single cache block, and a `cache_block_size` large enough to contain the entire input file.

Loading an entire file into memory can make performance worse, in cases where file size is not much smaller than physical memory size, so be careful using this option. Where it works, however, it can speed the chipping process considerably.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.pgpointcloud.writer">
      <Option name="connection">
        host='localhost' dbname='lidar' user='lidaruser'
      </Option>
      <Option name="table">example</Option>
      <Filter type="filters.chipper">
        <Option name="capacity">400</Option>
        <Filter type="filters.cache">
          <Option name="max_cache_blocks">1</Option>
          <Option name="cache_block_size">1028000000</Option>
  	      <Reader type="drivers.las.reader">
  	        <Option name="filename">example.las</Option>
  	        <Option name="spatialreference">EPSG:26916</Option>
  	      </Reader>
  	    </Filter>
      </Filter>
    </Writer>
  </Pipeline>


Options
-------

max_cache_blocks
  How many blocks to allocate to the cache. [Default: **1**]
  
cache_block_size
  How big should the cache block be? Generally should be slightly larger than the input file for maximum effectiveness. [Default: **32768**]


.. _filters.hexbin:

filters.hexbin
==============

A common questions for users of point clouds is what the spatial extent of a point cloud collection is. Files generally provide only rectangular bounds, but often the points inside the files only fill up a small percentage of the area within the bounds.

.. figure:: filters.hexbin.img1.jpg
    :scale: 50 %
    :alt: Hexbin derive from input point buffer

    Hexbin output shows boundary of actual points in point buffer, not just rectangular extents.

The hexbin filter reads a point stream and writes out a metadata record that contains a much tighter data bound, expressed as a well-known text polygon. In order to write out the metadata record, the `pdal` pipeline command must be invoked using the `--pipeline-serialization` option:

::

    $ pdal hexbin-pipeline.xml --pipeline-serialization hexbin-out.xml

After running with the pipeline serialization option, the output file looks like this:

.. code-block:: xml

  <PointBuffer>
    <Metadata name="pointbuffer" type="blank">
      <Metadata name="filters.hexbin" type="blank">
        <Metadata name="edge_size" type="double">
          0.002218331916495204
        </Metadata>
        <Metadata name="threshold" type="nonNegativeInteger">
          15
        </Metadata>
        <Metadata name="sample_size" type="nonNegativeInteger">
          5000
        </Metadata>
        <Metadata name="boundary" type="string">
          MULTIPOLYGON (((
          -80.8466 35.2183, -80.8460 35.2194, 
          -80.8447 35.2194, -80.8441 35.2205, 
          -80.8428 35.2205, -80.8421 35.2216,
          ...
          -80.8498 35.2194, -80.8486 35.2194, 
          -80.8479 35.2183, -80.8466 35.2183
          )))
        </Metadata>
      </Metadata>
    </Metadata>
  </PointBuffer>

In addition, if you have defined a writer you will have the usual point data output file.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">
        file-output.las
      </Option>
      <Filter type="filters.hexbin">
        <Option name="x_dim">drivers.las.reader.X</Option>
        <Option name="y_dim">drivers.las.reader.Y</Option>
        <Option name="threshold">10</Option>
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

x_dim
  The name of the dimension to use as the X coordinate in the cropping process. [Default: **X**]
  
y_dim
  The name of the dimension to use as the Y coordinate in the cropping process. [Default: **Y**]

edge_size
  If not set, the hexbin filter will estimate a hex size based on a sample of the data. If set, hexbin will use the provided size in constructing the hexbins to test.

sample_size
  How many points to sample when automatically calculating the edge size? [Default: **5000**]

threshold
  Number of points that have to fall within a hexbin before it is considered "in" the data set. [Default: **15**] 
  
precision
  Coordinate precision to use in writing out the well-known text of the boundary polygon. [Default: **8**]
  
  










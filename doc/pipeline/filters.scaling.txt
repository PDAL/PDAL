.. _filters.scaling:

filters.scaling
===============

The scaling filter allows you to alter the data type and range of dimensions by applying scaling factors and offsets.

Data are read from the input schema, and de-normalized (internal scaling and offsets removed), then written to the output schema with the new requested scaling and offsets applied.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">
        file-scaled.las
      </Option>
      <Filter type="filters.scaling">
        <Option name="dimension">
          X
          <Options>
            <Option name="scale">1.0</Option>
            <Option name="offset">100000</Option>
            <Option name="type">SignedInteger</Option>
            <Option name="size">4</Option>
          </Options>
        </Option>
        <Option name="dimension">
          Y
          <Options>
            <Option name="scale">0.1</Option>
            <Option name="offset">0</Option>
            <Option name="type">SignedInteger</Option>
            <Option name="size">4</Option>
          </Options>
        </Option>
        <Option name="dimension">
          Z
          <Options>
            <Option name="scale">10</Option>
            <Option name="offset">0</Option>
            <Option name="type">UnsignedInteger</Option>
            <Option name="size">2</Option>
          </Options>
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

dimension
  The dimension to apply the new scalings to.
    
  scale
    The scaling to apply when storing values in this dimension.
  
  offset
    The offset to remove when storing values in this dimension.
  
  type
    The data type of this output dimension. [Default: **SignedInteger**] [Legal values: SignedInteger, UnsignedInteger, SignedByte, UnsignedByte, RawByte, Float]

  size
    The number of bytes to use in storing values for this dimension. Must be a multiple of 2. [Default: **4**]

ignore_old_dimensions
  A boolean value. Discard the input dimensions from the output stream if true. [Default: **true**]
  
  

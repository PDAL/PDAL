.. _filters.selector:

filters.selector
================

The selector filter allows you to add and remove attributes from a stream of points, or mark attributes to be ignored by the final writing stage.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Filter type="filters.selector">
      <Option name="overwite_existing_dimensions">false</Option>
      <Option name="keep">
        <Options>
          <Option name="dimension">X</Option>
          <Option name="dimension">Y</Option>
        </Options>
      </Option>
      <Option name="ignore">
        <Options>
          <Option name="dimension">Red</Option>
          <Option name="dimension">Green</Option>
          <Option name="dimension">Blue</Option>
        </Options>
      </Option>
      <Option name="create">
        <Options>
          <Option name="dimension">
            Greenish
            <Options>
              <Option name="type">
                uint16_t
              </Option>
              <Option name="size">
                2
              </Option>
              <Option name="description">
                This is the dimensions description
              </Option>
              <Option name="endianness">
                big
              </Option>
              <Option name="uuid">
                1f528897-a723-4a9c-8e5a-cae9b071d0b5
              </Option>
              <Option name="parent_uuid">
                7752759d-5713-48cd-9842-51db350cc979
              </Option>
              <Option name="scale">
                1e-7
              </Option>
              <Option name="offset">
                10.6
              </Option>
            </Options>
          </Option>
          <Option name="dimension">
            Green
            <Options>
              <Option name="type">
                uint16_t
              </Option>
              <Option name="size">
                2
              </Option>
              <Option name="description">
                This is the dimensions description
              </Option>
              <Option name="endianness">
                big
              </Option>
              <Option name="uuid">
                aabbc982-17c3-4c18-8c76-e3a9b344af86
              </Option>
              <Option name="parent_uuid">
                7752759d-5713-48cd-9842-51db350cc979
              </Option>
              <Option name="scale">
                1e-7
              </Option>
              <Option name="offset">
                10.6
              </Option>
            </Options>
          </Option>                
        </Options>
      </Option>        
      <Reader type="drivers.las.reader">
        <Option name="filename">
          ../1.2-with-color.las
        </Option>
      </Reader>
    </Filter>
  </Pipeline>

Options
-------

keep
  A list of dimensions to retain in the point stream, expressed as an options list.
  
  dimension
    The name of dimension to retain in the point stream. There can be multiple `dimension` options under a `keep` option.
  
ignore
  A list of dimensions to mark as ignored in the point stream, expressed as an options list.

  dimension
    The name of dimension to mark as ignored in the point stream. There can be multiple `dimension` options under an `ignore` option.
  
create
  A list of dimensions to create in the point stream, expressed as an options list.
  
  dimension
    The names the dimensions to create in the point stream. There can be multiple `dimension` options under an `ignore` option. [Required]
    
    type
      The data type of the dimension. One of: int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t, float_t, double_t. [Required]
    size
      The width of the dimension, in bytes (should match the declared type) [Optional]
    description
      This is the dimensions description [Optional]
    endianness
      "big" or "small" [Optional]
    uuid
      A unique value for the dimension (eg: aabbc982-17c3-4c18-8c76-e3a9b344af86) [Optional]
    parent_uuid
      If the dimension is derived from another dimension, the unique value for that parent. [Optional]
    scale
      The scaling to apply to the dimension values. [Required]
    offset
      The offset to apply to the dimension values. [Default: 0.0]
    
overwite_existing_dimensions
  A boolean options. When creating new dimensions allow the new dimensions to overwrite existing dimensions with the same name? [Default: **false**]
 
 

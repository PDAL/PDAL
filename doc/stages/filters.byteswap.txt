.. _filters.byteswap:

filters.byteswap
================

The byteswap filter takes a stream of points and reverses the byte order of all the dimensions. For example, if input points have an X dimension stored as a 32-bit integer in little endian order (0x01020304) byteswap will convert it to big-endian representation (0x04030201).

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">
        file-cropped.las
      </Option>
      <Filter type="filters.filters.byteswap">
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

The byteswap filter takes no options.

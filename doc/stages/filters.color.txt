.. _filters.color:

filters.color
=============

The color filters adds populates the Red, Green and Blue dimensions on points based on the Z dimension, using a fixed strategy to map Z values to colors based on their position in the range (ZMIN, ZMAX).

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">colored.las</Option>
      <Filter type="filters.color">
        <Reader type="drivers.las.reader">
            <Option name="filename">uncolored.las</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>


Options
-------

The color filter has no options.

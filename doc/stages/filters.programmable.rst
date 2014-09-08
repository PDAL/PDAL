.. _filters.programmable:

filters.programmable
====================

The programmable filter takes a stream of points and applies a `Python`_ function to each point in the stream.

The function must have two `NumPy`_ arrays as arguments, `ins` and `outs`. The `ins` array represents input points, the `outs` array represents output points. Each array contains all the dimensions of the point schema, for a number of points (depending on how large a point buffer the pipeline is processing at the time, a run-time consideration). Individual arrays for each dimension can be read from the input point and written to the output point.

This example scales the Z value by a factor of ten (of course, you could also use :ref:`filters.scaling` to achieve the same result.

.. code-block:: python

  import numpy as np

  def multiply_z(ins,outs):
      Z = ins['Z']
      Z = Z * 10.0
      outs['Z'] = Z
      return True

Note that the function always returns `True`. If the function returned `False`, an error would be thrown and the translation shut down.

To filter points based on a `Python`_ function, use the :ref:`filters.predicate` filter.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">
        file-filtered.las
      </Option>
      <Filter type="filters.programmable">
        <Option name="script">multiply_z.py</Option>
        <Option name="function">multiply_z</Option>
        <Option name="module">anything</Option>  
        <Reader type="drivers.las.reader">
          <Option name="filename">
            file-input.las
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

The XML pipeline file referenced the external `multiply_z.py` `Python`_ script, which scales up the Z coordinate by a factor of 10 (you could also do this with :ref:`filters.scaling`).

.. code-block:: python

  import numpy as np

  def multiply_z(ins,outs):
      Z = ins['Z']
      Z = Z * 10.0
      outs['Z'] = Z
      return True


Options
-------

script
  When reading a function from a separate `Python`_ file, the file name to read from. [Example: functions.py]

module
  The Python module that is holding the function to run. [Required]

function
  The function to call.

source
  The literal `Python`_ code to execute, when the script option is not being used.


.. _Python: http://python.org/
.. _NumPy: http://www.numpy.org/

.. _filters.matlab:

filters.matlab
====================

The **Matlab Filter** allows `Matlab`_ software to be embedded in a
:ref:`pipeline` that interacts with a struct array of the data and allows
you to modify those points. Additionally, some global :ref:`metadata` is also
available that Matlab functions can interact with.

The Matlab interpreter must exit and always set "ans==true" upon success. If
"ans==false", an error would be thrown and the :ref:`pipeline` exited.

.. seealso::
    :ref:`writers.matlab` can be used to write ``.mat`` files.


.. note::
    :ref:`filters.matlab` embeds the entire Matlab interpreter, and it
    will require a fully licensed version of Matlab to execute your script.

.. plugin::

Example
-------

.. code-block:: json

  [
      {
          "filename": "test\/data\/las\/1.2-with-color.las",
          "type": "readers.las"

      },
      {
          "type": "filters.matlab",
          "script": "matlab.m"

      },
      {
          "filename": "out.las",
          "type": "writers.las"
      }
  ]

Options
-------

script
  When reading a function from a separate `Matlab`_ file, the file name to read
  from. [Example: "functions.m"]

source
  The literal `Matlab`_ code to execute, when the script option is not
  being used.

add_dimension
  The name of a dimension to add to the pipeline that does not already exist.

struct
  Array structure name to read [Default: "PDAL"]

.. include:: filter_opts.rst

.. _Matlab: https://www.mathworks.com/products/matlab.html

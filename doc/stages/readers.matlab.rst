.. _readers.matlab:

readers.matlab
==============

The **Matlab Reader** supports readers Matlab ``.mat`` files. Data
must be in a `Matlab struct`_, with field names that correspond to
:ref:`dimensions` names. No ability to provide a name map is yet
provided.

Additionally, each array in the struct should ideally have the
same number of points. The reader takes its number of points
from the first array in the struct. If the array has fewer
elements than the first array in the struct, the point's field
beyond that number is set to zero.

.. _`Matlab struct`: https://www.mathworks.com/help/matlab/ref/struct.html

.. note::

    The Matlab reader requires the Mat-File API from MathWorks, and it must be
    explicitly enabled at compile time with the ``BUILD_PLUGIN_MATLAB=ON``
    variable


.. plugin::

.. streamable::

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.matlab",
          "struct":"PDAL",
          "filename":"autzen.mat"
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }

Options
-------

filename
  Output file name [REQUIRED]

struct
  Array structure name to read [OPTIONAL, defaults ``PDAL``]

.. _filters.python:

filters.python
====================

The ``filters.python`` filter allows `Python`_ software to be embedded in a
:ref:`pipeline` that interacts with a `NumPy`_ array of the data and allows
you to modify those points. Additionally, some global :ref:`metadata` is also
available that Python functions can interact with.

The function must have two `NumPy`_ arrays as arguments, ``ins`` and ``outs``.
The ``ins`` array represents the points before the ``filters.python``
filter and the ``outs`` array represents the points after filtering.

.. warning::
    :ref:`filters.python` was called ``filters.programmable`` and
    ``filters.predicate`` before PDAL 1.6. The functionality of
    ``filters.programmable`` and ``filters.predicate`` was rolled into
    :ref:`filters.python`, and the filter was renamed for clarity.


.. warning::

    Each array contains all the :ref:`dimensions` of the incoming ``ins`` point schema.
    Each array in the ``outs`` list match `NumPy`_ array of the
    same type as provided as ``ins`` for shape and type.

.. plugin::

.. code-block:: python

  import numpy as np

  def multiply_z(ins,outs):
      Z = ins['Z']
      Z = Z * 10.0
      outs['Z'] = Z
      return True



1) The function must always return `True` upon success. If the function returned `False`,
   an error would be thrown and the :ref:`pipeline` exited.



2) If you want write a dimension that might not be available, you can specify
   it with the ``add_dimension`` option. 

    .. code-block:: json

        "add_dimension": "NewDimensionOne"

   To create more than one dimension, this option also accepts an array.

    .. code-block:: json

        "add_dimension": [ "NewDimensionOne", "NewDimensionTwo", "NewDimensionThree" ]


.. note::

    To filter points based on a `Python`_ function, use the
    :ref:`filters.python` filter.

Modification Example
--------------------------------------------------------------------------------


.. code-block:: json

    {
      "pipeline":[
        "file-input.las",
        {
          "type":"filters.smrf"
        },
        {
          "type":"filters.python",
          "script":"multiply_z.py",
          "function":"multiply_z",
          "module":"anything"
        },
        {
          "type":"writers.las",
          "filename":"file-filtered.las"
        }
      ]
    }

The JSON pipeline file referenced the external `multiply_z.py` `Python`_ script,
which scales up the Z coordinate by a factor of 10.

.. code-block:: python

  import numpy as np

  def multiply_z(ins,outs):
      Z = ins['Z']
      Z = Z * 10.0
      outs['Z'] = Z
      return True

Predicates
--------------------------------------------------------------------------------

Points can be retained/removed from the stream by setting true/false values
into a special "Mask" dimension in the output point array.

The example above sets the "mask" to true for points that are in
classifications 1 or 2 and to false otherwise, causing points that are not
classified 1 or 2 to be dropped from the point stream.

.. code-block:: python

  import numpy as np

  def filter(ins,outs):
     cls = ins['Classification']

     keep_classes = [1,2]

     # Use the first test for our base array.
     keep = np.equal(cls, keep_classes[0])

     # For 1:n, test each predicate and join back
     # to our existing predicate array
     for k in range(1,len(keep_classes)):
         t = np.equal(cls, keep_classes[k])
         keep = keep + t

     outs['Mask'] = keep
     return True


.. note::

    :ref:`filters.range` is a specialized filter that implements the exact
    functionality described in this Python operation. It is likely to be much
    faster than Python, but not as flexible. :ref:`filters.python` is the tool
    you can use for prototyping point stream processing operations.

.. seealso::

    If you want to just read a :ref:`pipeline` of operations into a numpy
    array, the PDAL Python extension might be what you want. See it at
    https://pypi.python.org/pypi/PDAL

Example :ref:`pipeline`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: json

    {
      "pipeline":[
        "file-input.las",
        {
          "type":"filters.smrf"
        },
        {
          "type":"filters.python",
          "script":"filter_pdal.py",
          "function":"filter",
          "module":"anything"
        },
        {
          "type":"writers.las",
          "filename":"file-filtered.las"
        }
      ]
    }





Module Globals
--------------------------------------------------------------------------------

Three global variables are added to the Python module as it is run to allow
you to get :ref:`dimensions`, :ref:`metadata`, and coordinate system information.
Additionally, the ``metadata`` object can be set by the function to modify metadata
for the in-scope :ref:`filters.python` :cpp:class:`pdal::Stage`.

.. code-block:: python

   def myfunc(ins,outs):
       print ('schema: ', schema)
       print ('srs: ', spatialreference)
       print ('metadata: ', metadata)
       outs = ins
       return True

Updating metadata
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The filter can update the global ``metadata`` dictionary as needed, define it as a
**global** Python variable for the function's scope, and the updates will be
reflected back into the pipeline from that stage forward.

.. code-block:: python

   def myfunc(ins,outs):
     global metadata
     metadata = {'name': 'root', 'value': 'a string', 'type': 'string', 'description': 'a description', 'children': [{'name': 'filters.python', 'value': 52, 'type': 'integer', 'description': 'a filter description', 'children': []}, {'name': 'readers.faux', 'value': 'another string', 'type': 'string', 'description': 'a reader description', 'children': []}]}
     return True

Passing Python objects
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As of PDAL 1.5, it is possible to pass an option to :ref:`filters.python` and
:ref:`filters.python` of JSON representing a Python dictionary containing objects
you want to use in your function. This feature is useful in situations where you
wish to call :ref:`pipeline_command` with substitutions.

If we needed to be able to provide the Z scaling factor of `Example Pipeline`_ with a
Python argument, we can place that in a dictionary and pass that to the filter
as a separate argument. This feature allows us to be able easily reuse the same
basic Python function while substituting values as necessary.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.python",
          "module":"anything",
          "function":"filter",
          "source":"arguments.py",
          "pdalargs":"{\"factor\":0.3048,\"an_argument\":42, \"another\": \"a string\"}"
        },
        "output.las"
      ]
    }

With that option set, you can now fetch the ``pdalargs`` dictionary in your
Python script and use it:

.. code-block:: python

  import numpy as np

  def multiply_z(ins,outs):
      Z = ins['Z']
      Z = Z * float(pdalargs['factor'])
      outs['Z'] = Z
      return True




Standard output and error
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A ``redirector`` module is available for scripts to output to PDAL's log stream
explicitly. The module handles redirecting ``sys.stderr`` and ``sys.stdout`` for you
transparently, but it can be used directly by scripts. See the PDAL source
code for more details.


Options
--------------------------------------------------------------------------------

script
  When reading a function from a separate `Python`_ file, the file name to read
  from. [Example: functions.py]

module
  The Python module that is holding the function to run. [Required]

function
  The function to call.

source
  The literal `Python`_ code to execute, when the script option is not being used.

add_dimension
  A dimension name or an array of dimension names to add to the pipeline that do not already exist.

pdalargs
  A JSON dictionary of items you wish to pass into the modules globals as the
  ``pdalargs`` object.

.. _Python: http://python.org/
.. _NumPy: http://www.numpy.org/

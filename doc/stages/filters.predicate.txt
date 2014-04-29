.. _filters.predicate:

filters.predicate
=================

Like the ref:`filters.programmable` filter, the predicate filter applies a `Python`_ function to a stream of points. Points can be retained/removed from the stream by setting true/false values into a special "Mask" dimension in the output point array.

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

The example above sets the "mask" to true for points that are in classifications 1 or 2 and to false otherwise, causing points that are not classified 1 or 2 to be dropped from the point stream.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">
        file-cropped.las
      </Option>
      <Filter type="filters.predicate">
      <Option name="function">filter</Option>
      <Option name="module">anything</Option>        
      <Option name="source">

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

script
  When reading a function from a separate `Python`_ file, the file name to read from. [Example: functions.py]

module
  The Python module that is holding the function to run. [Required]

function
  The function to call.



.. _Python: http://python.org
.. _NumPy: http://www.numpy.org/

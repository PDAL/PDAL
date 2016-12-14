.. _writers.matlab:

writers.matlab
==============

The **Matlab Writer** supports writing Matlab `.mat` files.

The produced files have two variables, `Dimensions` and `Points`.  `Dimensions`
is a comma-delimited list of dimension names, and `Points` is a double array of
all dimensions of every points.  This output array can get very large
very quickly.


.. note::

    The Matlab writer requires the Mat-File API from MathWorks.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.las",
          "filename":"inputfile.las"
        },
        {
          "type":"writers.matlab",
          "output_dims":"X,Y,Z,Intensity",
          "filename":"outputfile.mat"
        }
      ]
    }

Options
-------

filename
  Output file name [REQUIRED]

output_dims
  Dimensions to include in the output file [OPTIONAL, defaults to all available dimensions]

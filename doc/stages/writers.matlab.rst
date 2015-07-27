.. _writers.matlab:

writers.matlab
==============

The **Matlab Writer** supports writing Matlab `.mat` files.

The produced files have two variables, `Dimensions` and `Points`.
`Dimensions` is a comma-delimited list of dimension names, and `Points` is a double array of all dimensions of every points.
Note that this output array can get very large very quickly.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.matlab">
      <Option name="filename">
        outputfile.mat
      </Option>
      <Option name="output_dims">
        X,Y,Z,Intensity
      </Option>
      <Reader type="readers.las">
        <Option name="filename">
          inputfile.las
        </Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  Output file name [REQUIRED]

output_dims
  Dimensions to include in the output file [OPTIONAL, defaults to all available dimensions]

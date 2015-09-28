.. _readers.ilvis2:

readers.ilvis2
===============

The **ILVIS2 reader** read from files in the ILVIS2 format. See http://nsidc.org/data/docs/daac/icebridge/ilvis2/index.html
for more information


.. figure:: readers.ilvis2.metadata.png

    Dimensions provided by the ILVIS2 reader

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">output.las</Option>
      <Reader type="readers.ilvis2">
        <Option name="filename">
          ILVIS2_GL2009_0414_R1401_042504.TXT
        </Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  File to read from [Required]

mapping
  Which ILVIS2 field type to map to X, Y, Z dimensions
  'LOW', 'CENTROID', or 'HIGH' ['CENTROID']

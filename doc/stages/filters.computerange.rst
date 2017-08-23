.. _filters.computerange:

===============================================================================
filters.computerange
===============================================================================

The Compute Range filter computes the range from the sensor to each of the
detected returns.

.. note::

    The Compute Range filter is specific to raw data from a particular data
    provider, where the sensor coordinates for each frame are encoded as regular
    points, and are identified by the pixel number -5.

.. embed::

Example
-------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "input.bpf",
        {
          "type":"filters.computerange"
        },
        {
          "type":"writers.bpf",
          "filename":"output.bpf",
          "output_dims":"X,Y,Z,Range"
        }
      ]
    }


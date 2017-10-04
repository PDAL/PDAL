.. _writers.null:

writers.null
============

The **null writer** discards its input.  No point output is produced when using
a **null writer**.

.. embed::

.. streamable::

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
          "type":"filters.hexbin"
        },
        {
          "type":"writers.null",
        }
      ]
    }

When used with an option that forces metadata output, like
--pipeline-serialization, this pipeline will create a hex boundary for
the input file, but no output point data file will be produced.

Options
-------

The **null writer** discards all passed options.


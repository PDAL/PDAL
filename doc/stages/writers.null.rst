.. _writers.text:

writers.null
============

The **null writer** discards its input.  No point output is produced when using
a **null writer**.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.null">
      <Filter type="filters.hexbin">
        <Reader type="readers.las">
          <Option name="filename">simple.las</Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>

When used with an option that forces metadata output, like
--pipeline-serialization, this pipeline will create a hex boundary for
the input file, but no output point data file will be produced.

Options
-------

The **null writer** discards all passed options.


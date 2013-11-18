.. _drivers.faux.writer:

drivers.faux.writer
===================

The "**faux writer**" is used for testing pipelines. It does not write to a file or database, but consumes input points and quietly does nothing.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.faux.writer">
      <Reader type="drivers.faux.reader">
        <Option name="bounds">([0,1000000],[0,1000000],[0,100])</Option>
        <Option name="num_points">10000</Option>
        <Option name="mode">random</Option>
      </Reader>
    </Writer>
  </Pipeline>

This pipeline file will generate ten thousand random points from nowhere, and then do nothing with them as a the faux writer consumes them quietly.

Options
-------

The faux writer has no options.

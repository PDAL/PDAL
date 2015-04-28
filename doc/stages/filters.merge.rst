.. _filters.merge:

filters.merge
============

The merge filter combines input from multiple sources into a single output.
No checks are made to ensure that points from the various sources have similar
dimensions or are generally compatible.  Notably, dimensions are not
initialized when points merged from various sources do not have dimensions in
common. 

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        merged.las
      </Option>
      <Filter type="filters.merge">
        <Reader type="readers.las">
          <Option name="filename">
            file1.las
          </Option>
        </Reader>
        <Reader type="readers.las">
          <Option name="filename">
            file2.las
          </Option>
        </Reader>
        <Reader type="readers.bpf">
          <Option name="filename">
            file3.bpf
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>


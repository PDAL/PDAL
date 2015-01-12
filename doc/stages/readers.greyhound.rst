.. _readers.greyhound:

readers.greyhound
=================

The **Greyhound Reader** allows you to read point data from a `Greyhound`_ server.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.text">
      <Option name="filename">output.txt</Option>
      <Option name="spatialreference">EPSG:26910</Option>
      <Reader type="readers.greyhound">
        <Option name="url">greyhound.organization.com:8080</Option>
        <Option name="pipelineId">a87d0a50e03a880c75e9f872c925f984</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

url
  Greyhound server URL string. [Required]

pipelineId
  Greyhound pipelineId to read. [Required]


.. _Greyhound: https://github.com/hobu/greyhound

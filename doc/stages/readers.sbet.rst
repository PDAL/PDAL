.. _readers.sbet:

readers.sbet
============

The **SBET reader** read from files in the SBET format, used for exchange data from interital measurement units (IMUs).


Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">output.las</Option>
      <Reader type="readers.sbet">
        <Option name="filename">
          sbetfile.sbet
        </Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  File to read from [Required]

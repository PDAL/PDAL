:orphan:

.. _filters.shell:

filters.shell
===================

The shell filter allows you to run shell operations in-line
with PDAL pipeline tasks. This can be especially useful for
follow-on items or orchestration of complex workflows.

.. embed::

.. warning::

    To use :ref:`filters.shell`, you must set ``PDAL_ALLOW_SHELL=1``
    PDAL's execution environment. Without the environment variable
    set, every attempt at execution will result in the following
    error:

        PDAL_ALLOW_SHELL environment variable not set, shell access is not allowed

Example
---------

GDAL processing operations applied to raster output from :ref:`writers.gdal`
are a common task. Applying these within the PDAL execution environment
can provide some convenience and allow downstream consumers to have deterministic
completion status of the task. The following task writes multiple elevation
models to disk and then uses the `gdaladdo <https://gdal.org/gdaladdo.html>`__
command to construct overview bands for the data using average interpolation.

.. code-block:: json

    {
      "pipeline":[
        "autzen.las",
        {
          "type":"writers.gdal",
          "filename" : "output-1m.tif",
          "resolution" : "1.0"
        },
        {
          "type":"writers.gdal",
          "filename" : "output-2m.tif",
          "resolution" : "2.0"
        },
        {
          "type":"writers.gdal",
          "filename" : "output-5m.tif",
          "resolution" : "5.0"
        },
        {
          "type":"filters.shell",
          "command" : "gdaladdo -r average output-1m.tif 2 4 8 16"
        },
        {
          "type":"filters.shell",
          "command" : "gdaladdo -r average output-2m.tif 2 4 8 16"
        },
        {
          "type":"filters.shell",
          "command" : "gdaladdo -r average output-5m.tif 2 4 8 16"
        }
        ]
    }


Options
-------

command
  The shell command to run. It is run in relation to the current
  working directory of the pipeline executing it.

.. include:: filter_opts.rst


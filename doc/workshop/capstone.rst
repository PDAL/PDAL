.. _capstone:

Final Project
================================================================================

.. include:: ./includes/substitutions.rst

.. index:: capstone, project

The final project brings together a number of PDAL processing workflow
operations into a single effort It builds upon the exercises to enable
you to use the capabilities of PDAL in a coherent processing strategy, and it
will give you ideas about how to orchestrate PDAL in the context of larger data
processing scenarios.

Given the following pipeline for fetching the data, complete the rest of the tasks:


.. code-block:: json

    {
        "pipeline": [
            {
                "type": "readers.ept",
                "filename":"http://na-c.entwine.io/dublin/",
                "bounds":"([-697041.0, -696241.0], [7045398.0, 7046086.0],[-40, 400])"

            },
            {
                "type": "writers.las",
                "compression": "true",
                "minor_version": "2",
                "dataformat_id": "0",
                "filename":"st-stephens.laz"
            }
        ]
    }


* Read data from an EPT resource using :ref:`readers.ept` (See :ref:`workshop-entwine`)
* Thin it by 1.0 meter spacing using :ref:`filters.sample` (See :ref:`workshop-thinning`)
* Filter out noise using :ref:`filters.outlier` (See :ref:`workshop-denoising`)
* Classify ground points using :ref:`filters.smrf` (See :ref:`workshop-ground`)
* Compute height above ground using :ref:`filters.hag_nn`
* Generate a digital terrain model (DTM) using :ref:`writers.gdal` (See :ref:`workshop-dtm`)
* Generate a average vegetative height model using :ref:`writers.gdal`

.. note::

    You should review specific :ref:`exercises` for specifics how to
    achieve each task.

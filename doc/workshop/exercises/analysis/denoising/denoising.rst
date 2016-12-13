.. _denoising:

Removing noise
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: Denoising, Filtering

This exercise uses PDAL to remove unwanted noise in an ALS collection.

.. warning::

    Our default :ref:`docker` machine instance is probably going to run out of
    memory for this operation (it only has 1gb). We may need to recreate it with
    the following commands to increase the available memory:

    1. Remove the existing machine instances

        .. literalinclude:: ./denoising-docker-machine-delete.txt

    2. Create a new one with 2gb of memory

        .. literalinclude:: ./denoising-docker-machine-delete.txt



Exercise
--------------------------------------------------------------------------------

PDAL provides a :ref:`filter <filters>` through |PCL| to apply a statistical
filter to data.

Because this operation is somewhat complex, we are going to use a pipeline to
define it.

.. include:: ./denoise.json
    :literal:

.. note::

    This pipeline is available in your workshop materials in the
    ``./exercises/analysis/denoising/denoise.json`` file.

Pipeline breakdown
................................................................................

1. Reader
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After our pipeline errata, the first item we define in the pipeline is the
point cloud file we're going to read.

::

    "/data/exercises/analysis/denoising/18TWK820985.laz",

2. :ref:`filters.outlier`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The :ref:`filters.outlier` PDAL filter does most of the work for this operation.

::

    {
        "type": "filters.outlier",
        "method": "statistical",
        "extract": "true",
        "multiplier": 3,
        "mean_k": 8
    },



3. :ref:`filters.range`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Even with the :ref:`filters.outlier` operation, there is still a cluster of
points with extremely negative ``Z`` values. These are some artifact or
miscomputation of processing, and we don't want these points. We are going to
use ::ref:`filters.range` to keep only points that are within the range
``-100 <= Z <= 3000``.

::

    {
        "type": "filters.range",
        "limits": "Z[-100:3000]"
    },

4. :ref:`writers.las`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We could just define the ``clean.laz`` filename, but we want to
add a few options to have finer control over what is written. These include:

::

    {
        "type": "writers.las",
        "compression": "true",
        "minor_version": "2",
        "dataformat_id": "0",
        "filename":"/data/exercises/analysis/denoising/clean.laz"
    }


1. ``compression``: |LASzip| data is ~6x smaller than ASPRS LAS.
2. ``minor_version``: We want to make sure to output LAS 1.2, which will
   provide the widest compatibility with other softwares that can
   consume LAS.
3. ``dataformat_id``: Format 3 supports both time and color information

.. note::

    :ref:`writers.las` provides a number of possible options to control
    how your LAS files are written.

Execution
................................................................................

Invoke the following command, substituting accordingly, in your `Docker
Quickstart Terminal`:

.. literalinclude:: ./denoising-run-command.txt

.. image:: ../../../images/denoise-run-command.png

Visualization
................................................................................

Use one of the point cloud visualization tools you installed to take a look at
your ``clean.laz`` output. In the example below, we simply
opened the file using the `Fugro Viewer`_

.. image:: ../../../images/denoise-fugro.png

.. _`Fugro Viewer`: http://www.fugroviewer.com/


Notes
--------------------------------------------------------------------------------

1. Control the aggressiveness of the algorithm with the ``mean_k`` parameter.

2. :ref:`filters.outlier` requires the entire set in memory to
   process. If you have really large files, you are going to need to
   :ref:`split <filters.splitter>` them in some way.

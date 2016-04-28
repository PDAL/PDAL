.. _ground:

Identifying ground
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: ground, classification, filtering

This exercise uses PDAL to classify ground returns using the `Progressive
Morphological Filter (PMF)` technique.

.. note::

    This excerise is an adaptation of the :ref:`pcl_ground` tutorial on the
    PDAL website by Brad Chambers. You can find more detail and example
    invocations there.

Exercise
--------------------------------------------------------------------------------

The primary input for `Digital Terrain Model`_ generation is a point cloud with
ground vs. not-ground classifications. In this example, we will use an
algorithm provided by PDAL, the `Progressive Morphological Filter` technique to
generate a ground surface.

.. seealso::

    PMF is implemented in PCL. PCL is then linked to PDAL. You can read more
    about the specifics of the algorithm from the `paper
    <http://users.cis.fiu.edu/~chens/PDF/TGRS.pdf>`__, and you can read more
    about the PCL implementation in the source code on `github
    <https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/morphological_filter.h>`__.

.. _`Digital Terrain Model`: https://en.wikipedia.org/wiki/Digital_elevation_model

Command
................................................................................

Invoke the following command, substituting accordingly, in your `Docker
Quickstart Terminal`:

.. literalinclude:: ./ground-run-no-filter.txt
    :linenos:

.. image:: ../../../images/ground-run-command.png

As we can see, the algorithm does a great job of discriminating the points, but
there's a few issues.

.. image:: ../../../images/ground-classified-included.png


There's noise underneath the main surface that will cause us trouble when we
generate a terrain surface.

.. image:: ../../../images/ground-classified-included-side.png

Filtering
................................................................................

We do not yet have a satisfactory surface for generating a DTM.  When we
visualize the output of this ground operation, we notice there's still some
noise. PCL also has its own :ref:`pipeline` concept, and we can stack the
call to PMF with a call to a the `filters.statisticaloutlier` technique we
learned about in :ref:`denoising`.

1. Let us start by removing the non-ground data:

.. literalinclude:: ./ground-run-ground-only.txt
    :linenos:
    :emphasize-lines: 6

.. note::

    The ``filters.ground.extract=true`` item causes all data except
    ground-classified points to be removed from the set.

Buildings and other non-ground points are removed with the ``extract`` option
of :ref:`filters.ground`

.. image:: ../../../images/ground-ground-only-view.png


2. Now we will remove the noise. PDAL has the :ref:`pcl_command` to allow you
   to pass |PCL| pipelines for processing. We will use this to combine
   the PMF and StatisticalOutlierRemoval filters into a single operation.

.. literalinclude:: ./filter.json
    :linenos:

.. note::

    This pipeline is available in your workshop materials in the
        ``./exercises/analysis/ground/filter.json`` file.


.. literalinclude:: ./ground-run-pcl-filter.txt
    :linenos:

The :ref:`pcl_command` allows you to use :ref:`pcl_json_specification` operations in
succession over data.

.. image:: ../../../images/ground-filtered.png

.. note::

    This pipeline is available in your workshop materials in the
    ``./exercises/analysis/ground/filter.json`` file.




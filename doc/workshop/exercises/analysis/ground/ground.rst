.. _workshop-ground:

Identifying ground
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: ground, classification, filtering

This exercise uses PDAL to classify ground returns using the `Simple
Morphological Filter (SMRF)` technique.

.. note::

    This exercise is an adaptation of the :ref:`pcl_ground` tutorial on the
    PDAL website by Brad Chambers. You can find more detail and example
    invocations there.

Exercise
--------------------------------------------------------------------------------

The primary input for `Digital Terrain Model`_ generation is a point cloud with
ground vs. not-ground classifications. In this example, we will use an
algorithm provided by PDAL, the `Simple Morphological Filter` technique to
generate a ground surface.

.. seealso::

    You can read more about the specifics of the SMRF algorithm from `[Pingle2013]_`

.. _`Digital Terrain Model`: https://en.wikipedia.org/wiki/Digital_elevation_model

Command
................................................................................

Invoke the following command, substituting accordingly, in your `Conda Shell`:

.. literalinclude:: ./ground-run-no-filter.txt
    :linenos:

.. literalinclude:: ./ground-run-no-filter-win.txt
    :linenos:

.. image:: ../../../images/ground-run-command.png
    :target: ../../../../_images/ground-run-command.png


As we can see, the algorithm does a great job of discriminating the points, but
there's a few issues.

.. image:: ../../../images/ground-classified-included.png
    :target: ../../../../_images/ground-classified-included.png


There's noise underneath the main surface that will cause us trouble when we
generate a terrain surface.

.. image:: ../../../images/ground-classified-included-side.png
    :target: ../../../../_images/ground-classified-included-side.png

Filtering
................................................................................

We do not yet have a satisfactory surface for generating a DTM.  When we
visualize the output of this ground operation, we notice there's still some
noise. We can stack the call to SMRF with a call to a the `filters.outlier`
technique we learned about in :ref:`denoising`.

1. Let us start by removing the non-ground data to just view the ground data:

.. literalinclude:: ./ground-run-ground-only.txt
    :linenos:
    :emphasize-lines: 5
    :language: console

.. literalinclude:: ./ground-run-ground-only-win.txt
    :linenos:
    :emphasize-lines: 5
    :language: bat

.. image:: ../../../images/ground-ground-only-view.png
    :target: ../../../../_images/ground-ground-only-view.png


2. Now we will instead use the :ref:`translate_command` command to stack the
:ref:`filters.outlier` and :ref:`filters.smrf` stages:

.. literalinclude:: ./translate-run-ground-only.txt
   :linenos:

.. literalinclude:: ./translate-run-ground-only-win.txt
   :linenos:

In this invocation, we have more control over the process. First the outlier
filter merely classifies outliers with a ``Classification`` value of 7. These
outliers are then ignored during SMRF processing with the ``ignore`` option.
Finally, we add a range filter to extract only the ground returns (i.e.,
``Classification`` value of 2).

The result is a more accurate representation of the ground returns.

.. image:: ../../../images/ground-filtered.png

.. _workshop-ground:

Identifying ground
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: ground, classification, filtering

This exercise uses PDAL to classify ground returns using the `Simple
Morphological Filter (SMRF)` technique.

.. note::

    This exercise is an adaptation of the :ref:`ground-filters` tutorial on the
    PDAL website by Brad Chambers. You can find more detail and example
    invocations there.

Exercise
--------------------------------------------------------------------------------

The primary input for `Digital Terrain Model`_ generation is a point cloud with
ground vs. not-ground classifications. In this example, we will use an
algorithm provided by PDAL, the `Simple Morphological Filter` technique to
generate a ground surface.

.. seealso::

    You can read more about the specifics of the :ref:`Simple Morphological
    Filter (SMRF) <filters.smrf>`

.. _`Digital Terrain Model`: https://en.wikipedia.org/wiki/Digital_elevation_model

Command
................................................................................

Invoke the following command, substituting accordingly, in your `Conda Shell`:

.. code-block:: console

    $ pdal translate ./exercises/analysis/ground/CSite1_orig-utm.laz \
    -o ./exercises/analysis/ground/ground.laz \
    smrf \
    -v 4

.. code-block:: doscon

    > pdal translate ./exercises/analysis/ground/CSite1_orig-utm.laz ^
    -o ./exercises/analysis/ground/ground.laz ^
    smrf ^
    -v 4

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
technique we learned about in :ref:`workshop-denoising`.

1. Let us start by removing the non-ground data to just view the ground data:

    .. code-block:: console
        :emphasize-lines: 5

        $ pdal translate \
        ./exercises/analysis/ground/CSite1_orig-utm.laz \
        -o ./exercises/analysis/ground/ground.laz \
        smrf range \
        --filters.range.limits="Classification[2:2]" \
        -v 4

    .. code-block:: doscon
        :emphasize-lines: 5

        > pdal translate ^
        ./exercises/analysis/ground/CSite1_orig-utm.laz ^
        -o ./exercises/analysis/ground/ground.laz ^
        smrf range ^
        --filters.range.limits="Classification[2:2]" ^
        -v 4


    .. image:: ../../../images/ground-ground-only-view.png
        :target: ../../../../_images/ground-ground-only-view.png


2. Now we will instead use the :ref:`translate_command` command to stack the
:ref:`filters.outlier` and :ref:`filters.smrf` stages:

    .. code-block:: console

        $ pdal translate ./exercises/analysis/ground/CSite1_orig-utm.laz \
        -o ./exercises/analysis/ground/denoised-ground-only.laz \
        outlier smrf range  \
        --filters.outlier.method="statistical" \
        --filters.outlier.mean_k=8 --filters.outlier.multiplier=3.0 \
        --filters.smrf.ignore="Classification[7:7]"  \
        --filters.range.limits="Classification[2:2]" \
        --writers.las.compression=true \
        --verbose 4

    .. code-block:: doscon

        > pdal translate ./exercises/analysis/ground/CSite1_orig-utm.laz ^
        -o ./exercises/analysis/ground/denoised-ground-only.laz ^
        outlier smrf range  ^
        --filters.outlier.method="statistical" ^
        --filters.outlier.mean_k=8 --filters.outlier.multiplier=3.0 ^
        --filters.smrf.ignore="Classification[7:7]"  ^
        --filters.range.limits="Classification[2:2]" ^
        --writers.las.compression=true ^
        --verbose 4

In this invocation, we have more control over the process. First the outlier
filter merely classifies outliers with a ``Classification`` value of 7. These
outliers are then ignored during SMRF processing with the ``ignore`` option.
Finally, we add a range filter to extract only the ground returns (i.e.,
``Classification`` value of 2).

The result is a more accurate representation of the ground returns.

.. image:: ../../../images/ground-filtered.png

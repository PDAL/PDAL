.. _workshop-meshing:

Creating surface meshes
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: ground, classification, filtering

This exercise uses PDAL to create surface meshes. PDAL is able to use a number of
meshing filters: https://pdal.io/stages/filters.html#mesh. Two of these are 'in
the box', without needing plugins compiled. These are 2D Delaunay triangulation
and Poisson surface meshing.

.. note::



Exercise
--------------------------------------------------------------------------------

two parts: first to mesh interesting terrain, second to mesh a building


.. seealso::


Command
................................................................................

Invoke the following command, substituting accordingly, in your `Conda Shell`:

.. literalinclude:: ./meshing-terrain.txt
    :linenos:

.. image:: ../../../images/meshing-terrain.png
    :target: ../../../../_images/meshing-terrain.png


Filtering
................................................................................

Add clipping a building in this section (filter by class = 6 OR exact clip from
building polygon)

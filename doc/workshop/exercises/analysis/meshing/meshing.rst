.. _workshop-meshing:

Creating surface meshes
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: ground, classification, filtering

This exercise uses PDAL to create surface meshes. PDAL is able to use a number of
meshing filters: https://pdal.io/stages/filters.html#mesh. Three of these are 'in
the box', without needing plugins compiled. These are 2D Delaunay triangulation,
Greedy projection meshing and Poisson surface meshing.

2D Delaunay triangulation is best suited for...

The Greedyprojection filter creates a triangle mesh attempting to reconstruct a
surface from a set of points.

Poisson meshing creates a watertight isosurface using an entirely new point set imputed
from the source data. The 

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

If we want to just mesh a building, we can apply a `range` filter based on point
classification. These data have ground labelled as class 2, and buildings as 6.

In this exercise we will create a poisson mesh surface of a building and the ground
surrounding it.

Command
................................................................................

Invoke the following command, substituting accordingly, in your `Conda Shell`:

.. literalinclude:: ./meshing-buildings.txt
    :linenos:

.. image:: ../../../images/meshing-buildings.png
    :target: ../../../../_images/meshing-buildings.png

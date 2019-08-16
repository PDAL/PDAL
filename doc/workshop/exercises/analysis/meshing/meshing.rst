.. _workshop-meshing:

Creating surface meshes
================================================================================

.. include:: ../../../includes/substitutions.rst

.. index:: ground, classification, filtering

This exercise uses PDAL to create surface meshes. PDAL is able to use a number of
meshing filters: https://pdal.io/stages/filters.html#mesh. Three of these are 'in
the box', without needing plugins compiled. These are 2D Delaunay triangulation,
Greedy projection meshing and Poisson surface meshing.

In this exercise we'll create a Poisson surface mesh - a watertight isosurface -
from our input point cloud.

Exercise
--------------------------------------------------------------------------------

We will create mesh models of a building and its surrounds using an entwine data
input source.

After running each command, the output `.ply` file can be viewed in Meshlab or
CloudCompare.

.. seealso::

  PDAL implements Mischa Kazhdan's Poisson surface reconstruction algorithm. For
  details see `[Kazhdan2006]_`

Command
................................................................................

Invoke the following command, substituting accordingly, in your `Conda Shell`:

.. literalinclude:: ./meshing.txt
    :linenos:

.. image:: ../../../images/meshing.png
    :target: ../../../../_images/meshing-terrain.png


Filtering
................................................................................

If we want to just mesh a building, or just terrain, or both we can apply a `range`
filter based on point classification. These data have ground labelled as class 2,
and buildings as 6.

In this exercise we will create a poisson mesh surface of a building and the ground
surrounding it, using the same data subset as above.

Command
................................................................................

Invoke the following command, substituting accordingly, in your `Conda Shell`:

.. literalinclude:: ./meshing-buildings.txt
    :linenos:

.. image:: ../../../images/meshing-buildings.png
    :target: ../../../../_images/meshing-buildings.png

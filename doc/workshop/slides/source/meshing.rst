.. _meshing:

Creating surface meshes
================================================================================

.. include:: ../../includes/substitutions.rst

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


Command
................................................................................

Invoke the following command, substituting accordingly, in your `Conda Shell`:

.. literalinclude:: ../../exercises/analysis/meshing/meshing.txt
    :linenos:

.. image:: ../../images/meshing.png

You can view the mesh in Cloud Compare, you should see something similar to

.. image:: ../../images/first-mesh.png



Filtering
................................................................................

If we want to just mesh a building, or just terrain, or both we can apply a `range`
filter based on point classification. These data have ground labelled as class 2,
and buildings as 6.

Command
................................................................................

Invoke the following command, substituting accordingly, in your `Conda Shell`:

.. literalinclude:: ../../exercises/analysis/meshing/meshing-buildings.txt
    :linenos:

.. literalinclude:: ../../exercises/analysis//meshing/meshing-buildings-win.txt
    :linenos:

.. image:: ../../images/meshing-buildings.png


.. _clipping:

Cliping data with polygons
================================================================================

.. include:: ../../includes/substitutions.rst



Purpose:
    * Subset data with a polygon
    * Assign attribute data
    * Use PDAL pipeline

:ref:`Clipping Workshop Materials <pdal:workshop-clipping>`

Clipping (Autzen)
================================================================================

.. image:: ../../images/clipping-autzen-view.png

Clipping (setup)
================================================================================

* Polygons as GeoJSON
* Data in two coordinate system
* First use of PDAL Pipeline

Clipping (polygon)
================================================================================

.. image:: ../../images/clipping-view-polygons.png

Clipping (reproject)
================================================================================

* GeoJSON in Geographic
* LiDAR in State Plane
* Use OGR VRT

Clipping (vrt)
================================================================================

.. literalinclude:: ../../exercises/analysis/clipping/attributes.vrt
    :linenos:

Clipping (pipeline)
================================================================================

* ``translate`` not expressive enough
* Complex operations (merge, filter, write)
* Repeatable operations with substitution

Clipping (pipeline)
================================================================================

.. literalinclude:: ../../exercises/analysis/clipping/clipping.json
    :linenos:

.. _pipeline_breakdown:

Pipeline
================================================================================

1. :ref:`readers.las`
2. :ref:`filters.overlay`
3. :ref:`filters.range`
4. :ref:`writers.las`

Attribute Filter
================================================================================

* Assign point values

  * Polygon
  * Single value

Range Filter
================================================================================

* Keep or reject values

Pipeline Strategy
================================================================================

1. Assign with `filters.overlay`
2. Filter with `filters.range`


Clipping (command)
================================================================================

.. literalinclude:: ../../exercises/analysis/clipping/clipping-run-command.txt
    :linenos:

Clipping (verify)
================================================================================

.. image:: ../../images/clipping-stadium-clipped.png


Other ways to clip
================================================================================

* Clip using multiple :ref:`filters.range`
* :ref:`filters.divider` or :ref:`filters.chipper`

    .. image:: ../../../stages/filters.chipper.img2.png



Next
================================================================================

On to :ref:`colorization`

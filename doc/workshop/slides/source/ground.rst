.. _ground:

Identifying ground
================================================================================

Purpose:
    * Filter and classify ground points


:ref:`Ground Workshop Materials <unavco:ground>`


filters.ground
================================================================================

* Progressive Morphological Filter [Zhang2003]_.
* Exact (slow) and Approximate (fast)
* Ground only, no other types

.. [Zhang2003] Zhang, Keqi, et al. "A progressive morphological filter for removing nonground measurements from airborne LIDAR data." Geoscience and Remote Sensing, IEEE Transactions on 41.4 (2003): 872-882.

Ground (execution)
================================================================================

.. literalinclude:: ../../exercises/analysis/ground/ground-run-no-filter.txt
    :linenos:

Ground (view)
================================================================================

.. image:: ../../images/ground-classified-included.png

Ground (noise)
================================================================================

Noise!

.. image:: ../../images/ground-classified-included-side.png

Ground (ground only)
================================================================================

.. literalinclude:: ../../exercises/analysis/ground/ground-run-ground-only.txt
    :linenos:
    :emphasize-lines: 6

Ground (PCL pipeline)
================================================================================

.. literalinclude:: ../../exercises/analysis/ground/filter.json
    :linenos:

Ground (PCL pipeline)
================================================================================

.. literalinclude:: ../../exercises/analysis/ground/ground-run-pcl-filter.txt
    :linenos:
    :emphasize-lines: 5

Ground (view)
================================================================================

.. image:: ../../images/ground-filtered.png

Next
================================================================================

On to :ref:`dtm`

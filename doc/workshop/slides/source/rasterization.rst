.. _rasterization:

Rasterizing Attributes
================================================================================


.. include:: ../../includes/substitutions.rst



Purpose:
    * Utilize :ref:`readers.greyhound`
    * Interpolate data (Intensity, Classification)
    * Use `gdaldem`_ to apply color ramps
    * Explore :ref:`writers.gdal` options


:ref:`Rasterization Workshop Materials <pdal:workshop-rasterize>`

.. _`gdaldem`: http://www.gdal.org/gdaldem.html



:ref:`readers.greyhound`
================================================================================

  .. literalinclude:: ../../exercises/analysis/rasterize/classification.json


:ref:`pdal pipeline <pdal:pipeline>`
================================================================================

.. literalinclude:: ../../exercises/analysis/rasterize/classify-command.txt


Yuck
================================================================================

.. image:: ../../images/rasterization-denmark-no-ramp.png
    :scale: 200%


Color Ramp
================================================================================

.. literalinclude:: ../../exercises/analysis/rasterize/ramp.txt


`gdaldem`_
================================================================================

.. literalinclude:: ../../exercises/analysis/rasterize/gdaldem-run-command.txt

Better
================================================================================

.. image:: ../../images/rasterization-colored-classification.png
    :scale: 200%

Intensity
================================================================================

.. literalinclude:: ../../exercises/analysis/rasterize/intensity-run-command.txt
    :linenos:

Intensity Image
================================================================================

.. image:: ../../images/rasterization-colored-intensity.png
    :scale: 200%


Home
================================================================================
:ref:`Home <home>`


================================================================================
PDAL
================================================================================

The PDAL Python extension allows you to process data with PDAL into `Numpy`_
arrays. Additionally, you can use it to fetch `schema`_ and `metadata`_ from
PDAL operations.

Usage
--------------------------------------------------------------------------------

Given the following pipeline, which simply reads an `ASPRS LAS`_ file and
sorts it by the ``X`` dimension:

.. _`ASPRS LAS`: https://www.asprs.org/committee-general/laser-las-file-format-exchange-activities.html

.. code-block:: python


    json = """
    {
      "pipeline": [
        "1.2-with-color.las",
        {
            "type": "filters.sort",
            "dimension": "X"
        }
      ]
    }"""

    import pdal
    pipeline = pdal.Pipeline(json)
    pipeline.validate() # check if our JSON and options were good
    pipeline.loglevel = 9 #really noisy
    count = pipeline.execute()
    arrays = pipeline.arrays
    metadata = pipeline.metadata
    log = pipeline.log


.. _`Numpy`: http://www.numpy.org/
.. _`schema`: http://www.pdal.io/dimensions.html
.. _`metadata`: http://www.pdal.io/development/metadata.html

Requirements
================================================================================

* PDAL 1.4+
* Python >=2.7 (including Python 3.x)


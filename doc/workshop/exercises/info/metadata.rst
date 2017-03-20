.. _workshop-metadata:

Printing file metadata
================================================================================

.. include:: ../../includes/substitutions.rst

.. index:: metadata, coordinate system, spatial reference system

Exercise
--------------------------------------------------------------------------------

This exercise uses PDAL to print metadata information. Issue the
following command in your `Docker Quickstart Terminal`.


.. literalinclude:: ./metadata-command.txt
    :linenos:


.. image:: ../../images/info-interesting-metadata.png

.. note::

    PDAL :ref:`metadata <metadata>` is returned a in a tree
    structure corresponding to processing pipeline that produced
    it.

.. seealso::

    Use the `JSON`_ processing capabilities of your favorite processing
    software to selectively access and manipulate values.

    * `Python JSON library`_
    * `jsawk`_ (like ``awk`` but for JSON data)
    * `jq`_ (command line processor for JSON)
    * `Ruby JSON library`_

.. _`Python JSON library`: https://docs.python.org/2/library/json.html
.. _`jsawk`: https://github.com/micha/jsawk
.. _`jq`: https://stedolan.github.io/jq/
.. _`Ruby JSON library`: http://ruby-doc.org/stdlib-2.0.0/libdoc/json/rdoc/JSON.html


Notes
--------------------------------------------------------------------------------

1. PDAL uses `JSON`_ as the exchange format when printing information from :ref:`info_command`.
   JSON is a structured, human-readable format that is much simpler than its `XML`_ cousin.

2. The PDAL :ref:`metadata document <metadata>` contains background and
   information about specific metadata entries and what they mean.

3. Metadata available for a given file depends on the stage that produces the data.
   :ref:`Readers <readers>` produce same-named values where possible, but it is
   common that variables are different. :ref:`Filters <filters>` and even
   :ref:`writers <writers>` can also produce metadata entries.

4. Spatial reference system or coordinate system information is a kind of
   special metadata, and is treated as something primary to a :ref:`Stage <stage_index>`
   in PDAL.


.. _`CSV`: https://en.wikipedia.org/wiki/Comma-separated_values
.. _`JSON`: https://en.wikipedia.org/wiki/JSON
.. _`XML`: https://en.wikipedia.org/wiki/XML

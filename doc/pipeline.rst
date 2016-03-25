.. _pipeline:

******************************************************************************
Pipeline
******************************************************************************

:Author: Howard Butler
:Contact: howard@hobu.co
:Date: 03/22/2016

Pipelines are the operative construct in PDAL -- you must construct pipelines
to perform data translation operations, filter data, or simply to provide data
to an endpoint. This document describes some features and background of PDAL
Pipelines and give some examples you can use to execute translation
operations.

.. seealso::

    :ref:`pipeline_command` is used to invoke JSON pipeline operations
    via the command line.


.. warning::

    As of PDAL 1.2, `JSON`_ is now the preferred specification language
    for PDAL pipelines. XML read support is still available at 1.2, but
    JSON is preferred.

.. _`JSON`: http://www.json.org/

Composing pipelines
------------------------------------------------------------------------------

A `Pipeline` is composed as an array of :cpp:class:`pdal::Stage` , with the
first stage at the beginning and the last at the end.  There are two primary
building blocks in PDAL, :cpp:class:`pdal::Stage` and
:cpp:class:`pdal::PointView`. :cpp:class:`pdal::Reader`,
:cpp:class:`pdal::Writer`, and :cpp:class:`pdal::Filter` are all subclasses of
:cpp:class:`pdal::Stage`.

:cpp:class:`pdal::PointView` is the substrate that flows between stages in a
pipeline and transfers the actual data as it moves through the pipeline. A
:cpp:class:`pdal::PointView` contains a :cpp:class:`pdal::PointTablePtr`, which
itself contains a list of :cpp:class:`pdal::Dimension` objects that define the
actual channels that are stored in the :cpp:class:`pdal::PointView`.

PDAL provides four types of stages -- :cpp:class:`pdal::Reader`,
:cpp:class:`pdal::Writer`, :cpp:class:`pdal::Filter`, and
:cpp:class:`pdal::MultiFilter` -- with the latter being hardly used (just
:ref:`filters.merge`) at this point. A Reader is a producer of data, a Writer
is a consumer of data, and a Filter is an actor on data.

.. note::

   As a C++ API consumer, you are generally not supposed to worry about the underlying
   storage of the PointView, but there might be times when you simply just
   "want the data." In those situations, you can use the
   :cpp:func:`pdal::PointView::getBytes` method to stream out the raw storage.


Usage
------------------------------------------------------------------------------

While pipeline objects are manipulable through C++ objects, the other, more
convenient way is through an JSON syntax. The JSON syntax mirrors the
arrangement of the Pipeline, with options and auxiliary metadata added on a
per-stage basis.

We have two use cases specifically in mind:

* a :ref:`command-line <pipeline_command>` application that reads an JSON
  file to allow a user to easily construct arbitrary writer pipelines, as
  opposed to having to build applications custom to individual needs with
  arbitrary options, filters, etc.

* a user can provide JSON for a reader pipeline, construct it via a simple call
  to the PipelineManager API, and then use the :cpp:func:`pdal::Stage::read()`
  function to perform the read and then do any processing of the points.  This
  style of operation is very appropriate for using PDAL from within
  environments like Python where the focus is on just getting the points, as
  opposed to complex pipeline construction.


.. code-block:: json

    {
      "pipeline":[
        "/path/to/my/file/input.las",
        "output.las"
      ]
    }


.. note::

    https://github.com/PDAL/PDAL/blob/master/test/data/pipeline/ contains
    test suite pipeline files that provide an excellent example of the
    currently possible operations.


Stage Types
..............................................................................


:cpp:class:`pdal::Reader`, :cpp:class:`pdal::Writer`, and
:cpp:class:`pdal::Filter` are the C++ classes that define the stage types in
PDAL. Readers follow the pattern of :ref:`readers.las` or
:ref:`readers.oci`, Writers follow the pattern of :ref:`writers.las` or
:ref:`readers.oci`, with Filters using :ref:`filters.reprojection` or
:ref:`filters.crop`.

.. note::

    :ref:`stage_index` contains a full listing of possible stages and
    descriptions of their options.

.. note::

    Issuing the command ``pdal info --options`` will list all available
    stages and their options. See :ref:`info_command` for more.

Options
------------------------------------------------------------------------------

Options are the mechanism that PDAL uses to inform :cpp:class:`pdal::Stage`
entities how to process data. The following example sorts the data using a
`Morton ordering`_ using :ref:`filters.mortonorder` and writes out a `LASzip`_
file as the result. We use options to define the ``compression`` function
for the :ref:`writers.las` :cpp:class:`pdal::Stage`.

.. _`LASzip`: http://www.laszip.org
.. _`Morton ordering`: http://en.wikipedia.org/wiki/Z-order_curve

.. code-block:: json

    {
      "pipeline":[
        "uncompressed.las",
        {
          "type":"filters.mortonorder"
        }
        {
          "type":"writers.las",
          "filename":"compressed.laz",
          "compression":"true"
        }
      ]
    }



Syntax Specification
------------------------------------------------------------------------------

See :ref:`json_pipeline_specification` for more detail

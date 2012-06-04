.. _pipeline:

******************************************************************************
Pipeline
******************************************************************************

:Author: Howard Butler
:Contact: hobu.inc at gmail dot com
:Date: 5/31/2012

Pipelines are the operative construct in PDAL -- you must construct pipelines
to perform data translation operations, filter data, or simply to provide data
to an endpoint. This document describes some features and background of PDAL
Pipelines and give some examples you can use to execute translation
operations.

Playing Dolls
------------------------------------------------------------------------------

A `Pipeline` is composed as a set of `Matryoshka dolls`_, with the outside 
or largest doll being the final endpoint of the pipeline, and the inner-most 
doll being the starting point. Each "doll" in the set represents a 
:cpp:class:`pdal::Stage` with its subsequent operations happening at that 
point as data are pulled through the pipeline from the starting point to 
end point.  

There are two primary building blocks in PDAL, :cpp:class:`pdal::Stage` and
:cpp:class:`pdal::PointBuffer`. :cpp:class:`pdal::Reader`,
:cpp:class:`pdal::Writer`, and :cpp:class:`pdal::Filter` are all subclasses of
:cpp:class:`pdal::Stage` that provide the concrete dolls for the Matryoshka
set. 

:cpp:class:`pdal::PointBuffer` is the substrate that flows between stages in a 
pipeline and transfers the actual data as it moves through the pipeline. A 
:cpp:class:`pdal::PointBuffer` contains a :cpp:class:`pdal::Schema`, which 
itself contains a list of :cpp:class:`pdal::Dimension` objects that define the 
actual channels that are stored in the :cpp:class:`pdal::PointBuffer`.  

PDAL provides four types of stages -- :cpp:class:`pdal::Reader`,
:cpp:class:`pdal::Writer`, :cpp:class:`pdal::Filter`, and
:cpp:class:`pdal::MultiFilter` -- with the latter being hardly used at this
point. A Reader is a producer of data, a Writer is a consumer of data, and a
Filter is an actor on data.

.. note::
    The current implementation of PointBuffer stores the point data in an 
    interleaved fashion. It is probable that a subclass of PointBuffer will 
    come along that stores the data non-interleaved for situations where that 
    data arrangement is well-suited. Patches graciously accepted :)
    
A diversion about :cpp:class:`pdal::Schema` and :cpp:class:`pdal::Dimension`
..............................................................................

The :cpp:class:`pdal::PointBuffer` is composed of a :cpp:class:`pdal::Schema`
that itself contains a list of :cpp:class:`pdal::Dimension` objects. A
:cpp:class:`pdal::Dimension` defines the properties of the data channel like
its size in bytes, its name, and its interpretation. 


.. warning::

    As of this writing, :cpp:class:`pdal::Stage` also has a
    :cpp:class:`pdal::Schema` instance, but this merely exists to collect the
    :cpp:class:`pdal::Dimension` that a :cpp:class:`pdal::Stage` is currently
    capable of providing. The model is likely to change to simply providing a
    list of :cpp:class:`pdal::Dimension` instances that are available, and the
    person composing the :cpp:class:`pdal::Schema` will worry about their order
    and other schema properties.

.. _`Matryoshka dolls`: http://en.wikipedia.org/wiki/Matryoshka_doll

A Basic Example
------------------------------------------------------------------------------

While pipeline objects are manipulable through C++ objects, the other, more
convenient way is through an XML syntax. The XML syntax mirrors the Matryoshka
arrangement of the Pipeline, with options and auxiliary metadata added on a
per-stage basis.

We have two use cases specifically in mind:

    * a command-line application that reads an XML file to allow a user to
      easily construct arbitrary writer pipelines, as opposed to having to
      build applications custom to individual needs with arbitrary options,
      filters, etc.
      
    * a user can provide XML for a reader pipeline, construct it via a simple
      call to the PipelineManager API, and then use the Stage read() function
      to perform the read and then do any processing of the points.  This style
      of operation is very appropriate for using PDAL from within environments
      like Python where the focus is on just getting the points, as opposed to
      complex pipeline construction.

Let's start with the following basic example:

::

    <?xml version="1.0" encoding="utf-8"?>
    <Pipeline version="1.0">
        <Writer type="drivers.las.writer">
            <Option name="filename">
                output.las
            </Option>
            <Reader type="drivers.las.reader">
                <Option name="filename">
                    ./path/to/my/file.las
                </Option>
            </Reader>
        </Writer>
    </Pipeline>

.. note::

    Because the Pipeline XML has been in flux throughout development, this 
    version number has not been incremented yet. It will be frozen at 1.0 
    until there is a PDAL 1.0 release. After the 1.0 release, PDAL will 
    provide a modicum of backward-compatibility using the version number 
    provided in the Pipeline element of the XML.

.. note::

    https://github.com/PDAL/PDAL/blob/master/test/data/pipeline/ contains 
    test suite pipeline files that provide an excellent example of the 
    currently possible operations.

Pipeline textual description
..............................................................................
    
The first element, Pipeline, tells us what we are and gives a version number. 
Next, we have the Writer element, and provides the `type` of writer stage 
that we are going to have as the endpoint of our pipeline (see more about this 
in `Stage Types`_). After that, we have an Option element, which provides the 
filename that the Writer is going to use when outputting the file. Then we 
have the final doll in our Matryoshka set, the Reader, which has a stage 
type of `drivers.las.reader` with its own Option giving a filename to use 
to read the file.


Stage Types
..............................................................................


:cpp:class:`pdal::Reader`, :cpp:class:`pdal::Writer`, and
:cpp:class:`pdal::Filter` are the C++ classes that define the stage types in
PDAL. Readers follow the pattern of ``drivers.las.reader`` or
``drivers.oci.reader``, Writers follow the pattern of ``drivers.las.writer`` or
``drivers.oci.reader``, with Filters using ``filters.inplacereprojection`` or
``filters.crop``.

.. note::
    
    A "Stage registry" is planned to be provided through PDAL's
    GlobalEnvironment that will be available to applications to determine which
    stages are available to an application at runtime. This is not currently in
    place, however, and to determine the stage types that are available to you,
    you must look through the headers in ``./include/pdal/drivers`` and
    ``./include/pdal/filters``

Options
------------------------------------------------------------------------------

Options are the mechanism that PDAL uses to inform :cpp:class:`pdal::Stage` 
entities how to process data. A stage option can itself have options, going 
on ad infinitum. The following example is one that describes how the 
:cpp:class:`pdal::filters::Scaling` filter defines its options to denote which 
dimensions it must operate on.

::

    <?xml version="1.0" encoding="utf-8"?>
    <Pipeline version="1.0">
        <Filter type="filters.scaling">
            <Option name="dimension">
                X
                <Options>
                    <Option name="scale">0.1</Option>
                    <Option name="offset">1.0</Option>
                </Options>
            </Option>
            <Option name="dimension">
                Y
                <Options>
                    <Option name="scale">0.1</Option>
                    <Option name="offset">2.0</Option>
                </Options>
            </Option>
            <!-- <Option name="debug">true</Option>
            <Option name="verbose">7</Option>    -->  
            <Reader type="drivers.las.reader">
                <Option name="filename">
                    ../1.2-with-color.las
                </Option>
            </Reader>
        </Filter>
    </Pipeline>


Options need not be uniquely named. In this example, we have the dimension 
option added multiple times for both the ``X`` and ``Y`` dimensions. In this 
way, it is possible to define an option type called "dimension" that applies 
for both. Stages can then ask an option instance for options with the name 
``dimension``, and do with it what is necessary beyond that point.

.. note::

    :ref:`metadata` is quite similar to `Options`_ except that Option/Options
    instances are used to denote an action to be done during a stage rather
    than simply a data value to be set or fetched.  PDAL is indeed still hazy
    here and there is ample room to clean this up, but the general gestalt is
    that you use Options to tell a Stage what to do, and you use Metadata to
    set a value or data member for something related to a stage.

Syntax Specification
------------------------------------------------------------------------------


* <Pipeline> 
    * this is the root element for all pipeline xml
    * mandatory
    * child elements:
        * exactly one of the following four:
            * <Writer> element, for writer pipelines
            * <Reader> or <Filter> or <MultiFilter> element, for reader
              pipelines
    * attributes:
        * the "version" attribute must appear exactly once; the value of this
          attribute shall be the string "1.0"

* <Writer> :cpp:class:`pdal::Writer`
    * indicates a writer stage
    * child elements:
        * zero or more <Option> elements
        * exactly one <Reader> or <Filter> or <MultiFilter> element
    * attributes:
        * the "type" attribute must appear exactly once; the value of this
          attribute shall be the string corresponding to the name of
          the Writer stage type, e.g. "drivers.las.writer"

* <Reader> :cpp:class:`pdal::Reader`
    * indicates a reader stage
    * child elements:
        * zero or more <Option> elements
    * attributes:
        * the "type" attribute must appear exactly once; the value of this
          attribute shall be the string corresponding to the name of
          the Reader stage type, e.g. "drivers.las.reader"

* <Filter> :cpp:class:`pdal::Filter`
    * indicates a filter stage
    * child elements:
        * zero or more <Option> elements
        * exactly one <Reader> or <Filter> or <MultiFilter> element
    * attributes:
        * the "type" attribute must appear exactly once; the value of this
          attribute shall be the string corresponding to the name of
          the filter stage type, e.g. "filters.crop"

* <MultiFilter> :cpp:class:`pdal::MultiFilter`
    * indicates a multifilter stage (filter than takes >1 input stage)
    * child elements:
        * zero or more <Option> elements
        * one or more <Reader> or <Filter> or <MultiFilter> elements

    * attributes:
        * the "type" attribute must appear exactly once; the value of this
          attribute shall be the string corresponding to the name of
          the MultiFilter stage type, e.g. "filter.mosaic"

* <Option> :cpp:class:`pdal::Option`
    * indicates an option parameter to the pipeline stage
    * may only be a child of a <Reader>, <Writer>, <Filter>, or <MultiFilter>
      element
    * attributes:
        * the "name" attribute must appear exactly once; the value of this
          attribute shall be the name of the option, e.g. "filename"
    * any number of recursively-defined <Option> elements
    * content:
        * the content of the element shall be the string representing the
          value of the option, e.g. "input.las"


.. note::

    In the implementation, boost::property_tree are used to read and write the
    pipelines.  This means less parsing hassle for us, but also means we can't
    produce decent error messages (esp. because we don't have line numbers).

.. note::

    The schema is intended to be something that can be validated via XSD,
    although we don't do that today.  (There is an XSD file in the schemas
    directory.)



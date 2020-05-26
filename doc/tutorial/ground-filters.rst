.. _ground-filters:

================================================================================
Ground Filter Tutorial
================================================================================

:Author: Bradley Chambers
:Contact: brad.chambers@gmail.com
:Date: 04/17/2017

Background
--------------------------------------------------------------------------------

In previous tutorials we introduced our implmentation of the
:ref:`Progressive Morphological Filter (PMF)
<filters.pmf>`, a :ref:`ground kernel <ground_command>` to simplify command-line
access to PMF, and a filter for :ref:`removing outliers <filters.outlier>`.

This tutorial will highlight some recent enhancements to the PDAL library, in
the context of a ground segmentation workflow. Specifically, we will discuss:

* Constructing and executing a "filters-only" pipeline
* Resetting existing classifications prior to processing
* Using Extended Local Minimum (ELM) to identify low outliers
* Using Simple Morphological Filter (SMRF) as an alternative to PMF
* Ignoring outliers during ground segmentation
* Considering only last returns during ground segmentation
* Extracting ground returns as a post-processing step

.. note::
  
    The pipeline discussed in this tutoral requires `PDAL v1.5
    <https://github.com/PDAL/PDAL/releases/tag/1.5.0>`_.

The Pipeline
--------------------------------------------------------------------------------

Begin by creating a new file called ``pipeline.json`` with the following
contents.

.. literalinclude:: ground-filters-pipeline.json
    :language: json
    :linenos:

.. note::
  
    For users familiar with PDAL pipelines, this example may seem to be missing
    a couple of very important stages, namely the reader and writer! A new
    feature of PDAL is the ability to provide a PDAL pipeline with no reader or
    writer stages to the :ref:`translate_command` command. The input and output
    filenames can be specified on the command line and will be automatically
    inserted into the pipeline by the application.

The Explanation
--------------------------------------------------------------------------------

We continue by explaining the various stages of the pipeline in order.

Reprojecting Data
................................................................................

Many of PDAL's default parameters are specified in meters, and individual filter
stages typically assume that units are at least uniform in X, Y, and Z. Because
data will not always be provided in this way, PDAL pipelines should account for
any data reprojections and parameter scaling that are required from one dataset
to the next.

.. literalinclude:: ground-filters-pipeline.json
    :linenos:
    :lines: 3-6
    :lineno-start: 3

In this example, we show data being reprojected to ``EPSG:32632`` with X, Y, and
Z in meters.

Assigning Classification Values
................................................................................

Let's assume that you have been given an LAS file that contains per point
classifications, but you'd like to start with a clean slate and derive your own
classifications with your PDAL pipeline.

PDAL's :ref:`assign filter <filters.assign>` has been added to assign values to
a given dimension. In our example, a single option has been provided that
specifies the dimension, range, and value to assign. In this case, we are
stating that we would like to apply a value of 0 to the ``Classification``
dimension for every point.

.. literalinclude:: ground-filters-pipeline.json
    :linenos:
    :lineno-start: 7
    :lines: 7-10

.. note::
  
    Previously, you could do the same thing (with a slightly different syntax)
    using ``filters.attribute``, but this filter has been deprecated and split
    into :ref:`filters.assign` and :ref:`filters.overlay`.
    
Extended Local Minimum
................................................................................

The :ref:`Extended Local Minimum (ELM) method <filters.elm>` helps to identify
low noise points that can adversely affect ground segmentation algorithms. ELM
was first published in [Chen2012]_ as part of the upward-fusion method of DTM
generation. Noise points are **classified** with a value of ``7`` in keeping
with the LAS specification.

.. literalinclude:: ground-filters-pipeline.json
    :linenos:
    :lineno-start: 11
    :lines: 11-13

Outliers
................................................................................

PDAL's :ref:`outlier filter <filters.outlier>` provides two methods of outlier
detection at the moment: ``radius`` and ``statistical``. Both aim to identify
points that are isolated and likely arise from noise sources. Noise points are
**classified** with a value of ``7`` in keeping with the LAS specification.

.. literalinclude:: ground-filters-pipeline.json
    :linenos:
    :lineno-start: 14
    :lines: 14-16

Ground Segmentation
................................................................................

The :ref:`Simple Morphological Filter (SMRF) <filters.smrf>` [Pingel2013]_ is a
newer addition to PDAL that has quietly existed in an alpha state since v1.3.
With the release of PDAL v1.5, our SMRF implementation is much more complete,
although it only implements nearest neighbor void filling and not the authors'
preferred "Springs" algorithm.

The changes to SMRF between PDAL v1.3 and v1.5 are substantial. The original
version had actually drifted quite far from the authors' published approach,
namely in the area of filling voids. We have reverted the code to match the
published work, but for now are only using the nearest neighbors approach to
filling voids. The morphological operations are also accelerated by moving to an
iterative approach and using a diamond struturing element.

.. literalinclude:: ground-filters-pipeline.json
    :linenos:
    :lineno-start: 17
    :lines: 17-25
    
In addition to specifying some of the SMRF-specific arguments, our example also
demonstrates the use of two optional pre-filtering capabilities: ``ignore`` and
``last``.

The ``ignore`` option accepts a :ref:`range <ranges>`, here indicating that we
have points marked as noise (i.e., ``Classification`` of 7) that should be
excluded from ground segmentation, but are kept as part of the output dataset.

The ``last`` option, when set to ``true`` indicates that we would like to only
consider last returns for ground segmentation when return information is
available. Again, returns that are not "last returns" are still retained in the
output dataset - they are simply ignored for the purposes of ground
segmentation.

.. note::

    Many lidar systems provide return information. This includes the number of
    returns per pulse and the order of a particular return within the pulse.
    Where the return number and number of returns are equal, we call this a last
    return.

    Last returns are not by definition ground returns. In fact, the first and
    only return from surfaces such as rooftops will also be last returns, and
    last returns within dense foliage may not ever make it all the way to
    ground. Still, whenever there are multiple returns within a pulse, it stands
    to reason that anything before the last return would not be from the ground.

    Some bare earth algorithms explicitly operate on last returns only. In this
    case, this logic will presumably be implemented within the filter stage
    itself. That being said, it stands to reason that any ground segmentation
    approach could be improved by excluding all returns but the so-called last
    returns. Neither PMF nor SMRF make this assertion, but our implementations
    still consider only last returns by default. This behavior can be changed by
    setting ``last=false``.
    
    For an example of how to filter on last returns outside the context of SMRF
    and PMF, see `this
    <https://github.com/PDAL/PDAL/blob/master/test/data/pipeline/predicate-keep-last-return.json.in>`_
    within PDAL's source tree.
    
.. note::
  
    SMRF is not intended to be a replacement for the :ref:`Progressive
    Morphological Filter (PMF) <filters.pmf>` [Zhang2003]_. Rather, it is
    offered as an alternative. PMF has been a part of PDAL since v1.0, first as
    part of the PCL plugin and now as ``filters.pmf``. Since PDAL v1.4, we have
    fixed a number of bugs, and have accelerated the approximate mode by
    implementing iterative morphological operations and using a diamond
    structuring element.

Extracting Ground Returns
................................................................................

Any time we have points classified as ground, we may wish to extract just these
points, e.g., to create a *digital terrain model* (DTM). In this case, we use a
:ref:`range filter <filters.range>` as shown.

.. literalinclude:: ground-filters-pipeline.json
    :language: json
    :linenos:
    :lineno-start: 26
    :lines: 26-29
    
The :ref:`range filter <filters.range>` accepts a ``limits`` option that
identifies the dimension(s) on which to filter and the :ref:`range <ranges>` of
values to passthrough. In this case, we are indicating that the filter should
only pass points whose ``Classification`` value is equal to 2.

.. note::
  
    The default behavior of both :ref:`PMF <filters.pmf>` and :ref:`SMRF
    <filters.smrf>` is to classify points, which has not changed from previous
    versions of PDAL. The ``extract`` and ``classify`` options have been removed
    in PDAL v1.5. These filters now **only** classify points, such that ground
    points can be identified and filtered downstream, as we have shown with the
    range filter above.

Running the Pipeline
--------------------------------------------------------------------------------

Now let's run our ``pipeline.json`` example, using it to
:ref:`translate_command` ``input.las`` to ``output.las``.

::
  
    $ pdal translate input.las output.las --json pipeline.json

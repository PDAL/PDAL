.. python_filtering:

================================================================================
Filtering Data with Python
================================================================================

.. include:: ../workshop/includes/substitutions.rst

:Author: Howard Butler
:Contact: howard@hobu.co
:Date: 5/12/2017

.. contents:: Table of Contents
   :depth: 2


This tutorial will describe using :ref:`filters.programmable` to identify
outlier points in an LAS file.

Introduction
-------------------------------------------------------------------------------

Noise filtering is a primary challenge in point cloud processing. There are
many stock options available to you in PDAL to achieve it. These include:

* :ref:`filters.iqr`
* :ref:`filters.elm`
* :ref:`filters.outlier`

These filters remove points that are deemed outliers in different ways.
The don't, however, identify specific points. Python plus |NumPy| would
be a very quick way to prototype a tool that identified specific points
we would like to filter.

PDAL has three different ways to manipulate data with Python. The first is
:ref:`filters.programmable`, which we will be using in this tutorial. The
second is :ref:`filters.predicate`, which allows you to keep or remove points
given a Python filtering operation. The third is the Python extension at
https://pypi.python.org/pypi/PDAL that allows you to utilize PDAL processing
operations in your own Python programs.

.. seealso::

    :ref:`python` describes PDAL's Python story in more detail.


The Challenge
-------------------------------------------------------------------------------

We have a |ASPRSLAS| file that has some points that that are completely out of
range and have bad ``Y`` values. We don't know how those points got in the
file, but we want to identify **which** points they were, because it might be
some indication about a failure point in our processing operation. We could
simply use :ref:`filters.mad` to remove these with stock PDAL operations,
but that filter doesn't tell us which points it removed. Our own custom Python
filter can do this for us.

Processing Stategy
................................................................................

The technique that :ref:`filters.mad` uses will suit our problem. In short,
we want to identify which points have Y values that are more than three standard
deviations away from the median. We are going to print a |JSON| object that will
contain our information, which we can then use in some downstream processing
operation (as filtered with the `jq`_ command line utility).

We also want to use :ref:`docker` to process our data with a |Bash| script. We want to know
for sure that we're using PDAL's standard release (``1.5`` in this case), and
we'd like for our operation to be agnostic to the platform on which it is
running.


.. _`jq`: https://stedolan.github.io/jq/

Python Filter
-------------------------------------------------------------------------------

Through the use of the :ref:`filters.programmable` and :ref:`filters.predicate`
filters, PDAL allows the use of Python and |NumPy| to process point cloud
data. This can be very useful in prototyping situations, where PDAL can provide
convenient data access and the processing logic of software that is still
taking shape can be constructed with the rapid prototyping tools that Python
can provide.


.. code-block:: python
    :linenos:

    import numpy as np

    def mad(ins, outs):

        # Fetch the Y dimension, which has our outliers
        y = ins["Y"]

        # Let numpy compute median
        median = np.median(y)

        # Let numpy compute stddev
        stddev = np.std(y)

        # Identify points that are > 3 stddev
        indexes = np.where( abs(y-median) > 3*stddev)[0]

        # Stuff our output in a dictionary
        output = {}
        output['median'] = median
        output['stddev'] = stddev
        output['indexes'] = list([v for v in indexes])
        output['values'] = [y[i] for i in indexes]

        # Print our dict to stdout
        print output

        # filters.predicate must return True to tell
        # PDAL it successfully completed
        return True


PDAL Pipeline
--------------------------------------------------------------------------------

Our pipeline to apply the filter is straightforward. Because we are going to be
processing the data with a Bash script, we'll use some substitution to push
our filename (taken from an argument on the command line) and our script itself.
There is nothing much interesting except for the fact that we substitute our
filename in its own entry so PDAL can figure out which driver to read it with.
This will allow our script to work with any format PDAL can identify.

.. code-block:: json
    :linenos:

    {
      "pipeline":[
        "/data/$filename",
        {
        "type" : "filters.programmable",
        "function":"mad",
        "module":"anything",
        "source":"$script"
        }  ]
    }


Docker
--------------------------------------------------------------------------------

As I mentioned before, we want to run this script using Docker so we have
consistent versioning and so we can use the script in the same way across
a bunch of machines. To run the command in Docker we need to do two things:

1. Start a Docker container with the PDAL Docker container tag (``pdal/pdal:1.5`` in this case)

2. Run our command on that image once it is running.


``docker run`` outputs the SHA of the container instance it is running to STDIN. We
will capture that and then run our ``pdal pipeline`` command on it.

.. code-block:: shell


    # Start the container
    container=$(docker run -it -d -v `pwd`:/data pdal/pdal:1.5)

    # Echo our pipeline into it and run pdal pipeline on it
    output=$(echo $pipeline | docker exec -i $container pdal pipeline -i STDIN)

    # redirect stderr and stdout to null. We don't want to know which ID
    # was killed.
    docker kill $container &> /dev/null


Final Script
--------------------------------------------------------------------------------

Here's our final script. The junk from lines 53-66 are to get bash to do variable
substitution into our :ref:`pipeline` |JSON| without breaking its syntax doing double
quote and newline substitution.

Using our script, we can now identify which points have ``Y`` dimensions greater than
three standard deviations:

::

    $ ./run.sh bad-y-values.las
    {'values': [22940882.882926211, 18747910.56323766], 'median': 4322652.908885533, 'stddev': 8292.74344366484, 'indexes': [1375302, 1376129]}

.. code-block:: shell
    :linenos:

    #!/bin/bash

    filename="$1"

    read -d '' script <<"EOF"

    import numpy as np

    def mad(ins, outs):

        # Fetch the Y dimension, which has our outliers
        y = ins["Y"]

        # Let numpy compute median
        median = np.median(y)

        # Let numpy compute stddev
        stddev = np.std(y)

        # Identify points that are > 3 stddev
        indexes = np.where( abs(y-median) > 3*stddev)[0]

        # Stuff our output in a dictionary
        output = {}
        output['median'] = median
        output['stddev'] = stddev
        output['indexes'] = list([v for v in indexes])
        output['values'] = [y[i] for i in indexes]

        # Print our dict to stdout
        print output

        # filters.predicate must return True to tell
        # PDAL it successfully completed
        return True

    EOF

    read -d '' pipeline <<"EOF"
    {
      "pipeline":[
        "/data/$filename",
        {
        "type" : "filters.programmable",
        "function":"mad",
        "module":"anything",
        "source":"$script"
        }  ]
    }
    EOF


    # Do a bunch of bash vomit to properly substitute
    # in our $filename and $script variables into the pipeline
    # variable
    IFS="%"

    # echo newlines
    script=$(awk '{printf "%s\\n", $0}' <<< "$script")

    # replace " with \"
    script=$(sed  's/["]/\\&/g' <<< "$script")

    pipeline=$(awk '{printf "%s", $0}' <<< "$pipeline")
    pipeline=$(sed  's/["]/\\&/g' <<< "$pipeline")
    pipeline=$(eval echo $pipeline)


    # Start the container
    container=$(docker run -it -d -v `pwd`:/data pdal/pdal:1.5)

    # Echo our pipeline into it and run pdal pipeline on it

    output=$(echo $pipeline | docker exec -i $container pdal pipeline -i STDIN)

    # redirect stderr and stdout to null. We don't want to know which ID
    # was killed.
    docker kill $container &> /dev/null


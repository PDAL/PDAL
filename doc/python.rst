.. _python:

********************************************************************
Python
********************************************************************

.. index:: Numpy, Python


PDAL provides Python support in two significant ways. First it `embeds`_ Python
to allow you to write Python programs that interact with data using
:ref:`filters.python` filter. Second, it `extends`_ Python by providing an
extension that Python programmers can use to leverage PDAL capabilities in
their own applications.

.. _`embeds`: https://docs.python.org/3/extending/embedding.html
.. _`extends`: https://docs.python.org/3/extending/extending.html

.. note::

    PDAL's Python story always revolves around `Numpy`_ support. PDAL's
    data is provided to both the filters ands the extension as
    Numpy arrays.

.. _NumPy: http://www.numpy.org/

Versions
--------------------------------------------------------------------------------

PDAL supports both Python 3.5+. :ref:`integration` tests Python
Linux, OSX, and Windows.

Embed
--------------------------------------------------------------------------------

.. index:: Embed, Python

PDAL allows users to embed Python functions inline with other :ref:`pipeline`
processing operations. The purpose of this capability is to allow users to
write small programs that implement interesting actions without requiring a
full C++ development activity of building a PDAL stage to implement it. A
Python filter is an opportunity to interactively and iteratively prototype a
data operation without strong considerations of performance or generality.  If
something works well enough, maybe one takes on the effort to formalize it, but
that isn't necessary. PDAL's embed of Python allows you to be as grimy as you
need to get the job done.

.. figure:: ./images/python-pdal-pipeline.png

    Embedding a Python function to take Z values read from a
    :ref:`readers.las` and then output them to a :ref:`writers.bpf`.

Extend
--------------------------------------------------------------------------------

.. index:: Extension, Python

PDAL provides a Python `extension <https://pypi.python.org/pypi/PDAL>`_
that gives users access to executing
:ref:`pipeline <pipeline>` instantiations and capturing the results
as `Numpy`_ arrays.
This mode of operation is useful if you are looking to have PDAL simply act as
your data format and processing handler.

Python extension users are expected to construct their own PDAL
:ref:`pipeline <pipeline>`
using Python's ``json`` library, or whatever other libraries they wish to
manipulate JSON. They then feed it into the extension and get back the
results as `Numpy`_ arrays:

.. code-block:: python


    json = """
    [
        "1.2-with-color.las",
        {
            "type": "filters.sort",
            "dimension": "X"
        }
    ]
    """

    import pdal
    pipeline = pdal.Pipeline(json)
    count = pipeline.execute()
    arrays = pipeline.arrays
    metadata = pipeline.metadata
    log = pipeline.log

Installation
................................................................................

.. index:: Install, Python

The PDAL Python extension requires a working
:ref:`PDAL installation <download>`.  Unless you choose the Conda installation
method, make sure that you a current, working version of PDAL before
installing the extension.

.. note::

    Previous to PDAL 2.1, Python support was spread across the embedded
    stages (:ref:`readers.numpy` and :ref:`filters.python`) which were installed
    by PDAL itself and the PDAL extension that was installed from PyPI.
    As of PDAL 2.1 and PDAL/python 2.3, both the embedded stages and the
    extension are installed from PyPI.

Installation Using pip
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. index:: Install, Python, pip

As administrator, you can install PDAL using pip:

::

    pip install PDAL

.. note::

    To install pip please read
    `here <https://pip.pypa.io/en/stable/installing/>`_

Installation from Source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. index:: Install, Python, Source

PDAL Python support is hosted in a separate repository than PDAL itself at
`GitHub <https://github.com/PDAL/python>`_.  If you have a working PDAL
installation and a working Python installation, you can install the extension
using the following procedure on Unix.  The procedure on Windows is similar ::

    $ git clone https://github.com/PDAL/python pdalextension
    $ cd pdalextension
    $ pip install .

Install using Conda
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. index:: Install, Python, Conda

The PDAL Python support can also be installed using the `Conda`_
package manager.  An advantage of using Conda to install the extension is
that Conda will install PDAL. We recommend installing PDAL and the PDAL
Python extension in an environment other than the base environment.  To
install in an existing environment, use the following ::

    conda install -n <environment name> -c conda-forge python-pdal

Use the following command to install PDAL and the PDAL Python extension
into a new environment and activate that environment ::

    conda create -n <environment name> -c conda-forge python-pdal
    conda activate <environment name>

.. note::

    The official ``pdal`` and ``python-pdal`` packages reside in the
    conda-forge channel, which can be added via ``conda config`` or manually
    specified with the ``-c`` option, as shown in the examples above.

.. _`pip`: https://pip.pypa.io/en/stable/
.. _`Conda`: https://conda.io/docs/

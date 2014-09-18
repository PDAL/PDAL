.. _development_docs:

====
Docs
====


Building the Documentation
==========================

To build the PDAL documentation yourself, you need to install sphinx_, breathe_, and other python dependencies, preferably with PyPI_.
If you have `pip installed <http://www.pip-installer.org/en/latest/installing.html>`_, install the documentation dependencies with:

.. code-block:: bash

    (sudo) pip install sphinx breathe rst2pdf

If you are installing these packages to a system-wide directory, you may need the **sudo** in front of the **pip**, though it might be better that instead you use `virtual environments`_ instead of installing the packages system-wide.

The PDAL documentation also depends on `doxygen`_, which can be installed from source or from binaries from the `doxygen website <http://www.stack.nl/~dimitri/doxygen/download.html>`_.
If you are on Max OS X and use `homebrew`_, you can install doxygen with a simple ``brew install doxygen``.

Once you have installed all the doc dependencies, you can then build the documentation itself.
The :file:`doc/` directory in the PDAL source tree contains a Makefile which can be used to build all documentation.
For a list of the output formats supported by Sphinx, simply type ``make``.
For example, to build html documentation:

.. code-block:: bash

    cd doc
    make doxygen html

The html docs will be placed in :file:`doc/build/html/`.
The ``make doxygen`` is necessary to re-generate the API documentation from the source code.

.. _sphinx: http://sphinx-doc.org/
.. _breathe: https://github.com/michaeljones/breathe
.. _virtual environments: https://pypi.python.org/pypi/virtualenv
.. _pypi: https://pypi.python.org/pypi
.. _doxygen: http://www.stack.nl/~dimitri/doxygen/
.. _homebrew: http://mxcl.github.io/homebrew/

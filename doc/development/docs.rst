.. _development_docs:

================================================================================
Docs
================================================================================


Requirements
================================================================================

To build the PDAL documentation yourself, you need to install the following
items:

* Sphinx_
* Breathe_
* `Doxygen`_
* `Latex`_
* `dvipng`_

.. _`dvipng`: https://en.wikipedia.org/wiki/Dvipng
.. _`Latex`: https://en.wikipedia.org/wiki/LaTeX
.. _`pdflatex`: https://www.tug.org/applications/pdftex/

.. note::

    For the website, PDAL builds the documentation using :ref:`travis` and :ref:`docker`.
    You can get a listing of the exact Ubuntu requirements in the `Dockerfile`_.
    See :ref:`integration` for more detail.

.. _`Dockerfile`: https://github.com/PDAL/PDAL/tree/master/scripts/docker/docbuild/Dockerfile

Sphinx_ and Breathe_
--------------------------------------------------------------------------------

Python dependencies should be installed from PyPI_ with ``pip`` or
``easy_install``.

.. code-block:: bash

    (sudo) pip install sphinx sphinxconfig-bibtex breathe

.. note::

    If you are installing these packages to a system-wide directory, you may need
    the **sudo** in front of the **pip**, though it might be better that instead
    you use `virtual environments`_ instead of installing the packages system-wide.

Doxygen
--------------------------------------------------------------------------------

The PDAL documentation also depends on `Doxygen`_, which can be installed from
source or from binaries from the `doxygen website
<http://www.stack.nl/~dimitri/doxygen/download.html>`_.  If you are on Max OS X
and use `homebrew`_, you can install doxygen with a simple ``brew install
doxygen``.

Latex
--------------------------------------------------------------------------------

`Latex`_ and `pdflatex`_ are used to generate the companion PDF of the website.

dvipng
--------------------------------------------------------------------------------

For math output, we depend on `dvipng`_ to turn `Latex`_ output into math PNGs.

Generation
================================================================================

Once you have installed all the doc dependencies, you can then build the
documentation itself.  The :file:`doc/` directory in the PDAL source tree
contains a Makefile which can be used to build all documentation.  For a list
of the output formats supported by Sphinx, simply type ``make``.  For example,
to build html documentation:

.. code-block:: bash

    cd doc
    make doxygen html

The html docs will be placed in :file:`doc/build/html/`.  The ``make doxygen``
is necessary to re-generate the API documentation from the source code using
`Breathe`_ and `Sphinx`_.


.. note::

    For a full build of the :ref:`cppapi` documentation, you need to
    ``make doxygen`` to have it build its XML output which is consumed
    by `Breathe`_ before ``make html`` can be issued.


Website
================================================================================

The http://pdal.io website is regenerated from the ``*-maintenance`` branch using
:ref:`travis`. It will be committed by the ``PDAL-docs`` `GitHub`_ user and pushed
to the https://github.com/PDAL/pdal.github.io repository. The website
is then served via `GitHub Pages`_.

.. note::

    The website is regenerated and pushed only on the ``after_success`` :ref:`travis`
    call. If the tests aren't passing, the website won't be updated.

Building With Docker
================================================================================

A :ref:`docker` image, ``pdal/docs`` contains the full compliment of requirements,
and it is used by PDAL's :ref:`travis` continuous integration to build and commit
new versions of the website. You can easily build the docs using Docker by
issuing the following command:

::

    docker run -v /path/to/pdal/root/tree:/data -w /data/doc pdal/docs make html

.. _`GitHub Pages`: https://pages.github.com/
.. _`GitHub`: http://github.com/PDAL/PDAL

.. _`Digital Ocean`: digitalocean.com

.. _Sphinx: http://sphinx-doc.org/
.. _Breathe: https://github.com/michaeljones/breathe
.. _virtual environments: https://pypi.python.org/pypi/virtualenv
.. _pypi: https://pypi.python.org/pypi
.. _Doxygen: http://www.stack.nl/~dimitri/doxygen/
.. _homebrew: http://mxcl.github.io/homebrew/

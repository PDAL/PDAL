.. _workshop-conda:


Conda
================================================================================

.. index:: Conda, software installation

.. include:: ./includes/substitutions.rst




What is Conda
--------------------------------------------------------------------------------

Conda is an open source package management system and environment management system that runs
on Windows, macOS and Linux. Conda quickly installs, runs and updates packages and their dependencies.
Conda easily creates, saves, loads and switches between environments on your local computer.
It was created for Python programs, but it can package and distribute software for any language..


How will we use Conda?
--------------------------------------------------------------------------------

PDAL stands on the shoulders of giants. It uses GDAL, GEOS, and
:ref:`many other dependencies <building>`. Because of this, it is very
challenging to build it yourself. We could easily burn an entire workshop
learning the esoteric build mysteries of PDAL and all of its dependencies.
Fortunately, Conda provides us a fully-featured known
configuration to run our examples and exercises without having to suffer
so much, and provides it for Windows, Linux, and macOS.


.. note::

    Not everyone uses Conda. Another alternative to get a known configuration
    is to go through the workshop using :ref:`docker` as your platform. A
    previous edition of the workshop was provided as Docker, but it was
    found to be a bit too difficult to follow.

Installing Conda
--------------------------------------------------------------------------------

1. Copy the entire contents of your workshop USB key to a ``PDAL`` directory in your
   home directory (something like ``C:\Users\hobu\PDAL``) or the equivalent for your OS.
   We will refer to this location for the rest of the workshop materials.


2. Download the Conda installer for your OS setup. https://docs.conda.io/en/latest/miniconda.html


3. After installing Conda, create an environment for PDAL with::

    conda create --name pdalworkshop


4. Then *activate* the new environment::

    conda activate pdalworkshop


5. Install PDAL, Entwine, and GDAL, and install it from **conda-forge**::

    conda install -c conda-forge pdal python-pdal gdal entwine matplotlib

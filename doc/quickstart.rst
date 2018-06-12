.. _quickstart:

******************************************************************************
Quickstart
******************************************************************************

.. index:: Conda, Quickstart

Introduction
------------------------------------------------------------------------------

It's a giant pain to build everything yourself. The quickest way to start using
PDAL is to leverage builds that were constructed by the PDAL development team
using `Conda`_.

Directly from the Conda front page,

    *Conda is an open source package management system and environment
    management system that runs on Windows, macOS and Linux. Conda quickly
    installs, runs and updates packages and their dependencies. Conda easily
    creates, saves, loads and switches between environments on your local
    computer.*

This exercise will print the first point of an :ref:`ASPRS LAS <readers.las>`
file. It will utilize the PDAL :ref:`command line application <apps>` to
inspect the file.

.. note::

    If you need to compile your own copy of PDAL, see :ref:`building` for
    more details.

.. _`Conda`: https://conda.io/docs/


Install Conda
------------------------------------------------------------------------------

Conda installation instructions can be found at the following links. Read
through them a bit for your platform so you have an idea what to expect.

* `Windows <https://conda.io/docs/user-guide/install/windows.html>`__
* `macOS <https://conda.io/docs/user-guide/install/macos.html>`__
* `Linux <https://conda.io/docs/user-guide/install/linux.html>`__

.. note::

    We will assume you are running on Windows, but the same commands should
    work in macOS or Linux too -- though definition of file paths might provide
    a significant difference.


Run Conda
................................................................................

On macOS and Linux, all Conda commands are typed into a terminal window. On
Windows, commands are typed into the Anaconda Prompt window. Instructions can
be found in the Conda `Getting Started`_ guide.

.. _`Getting Started`: https://conda.io/docs/user-guide/getting-started.html#starting-conda


Test Installation
................................................................................

To test your installation, simply run the command ``conda list`` from your
terminal window or the Anaconda Prompt. A list of installed packages should
appear.


Install the PDAL Package
................................................................................

A PDAL package based on the latest release, including all recent patches, is
pushed to the `conda-forge`_ channel on anaconda.org with every code change on
the PDAL maintenance branch.

.. warning::

    It is actually a very good idea to install PDAL in it's own environment (or
    add it to an existing one). You will **NOT** want to add it to your default
    environment named ``base``. Managing environments is beyond the scope of
    the quickstart, but you should read more about it over at
    https://conda.io/docs/user-guide/getting-started.html#managing-envs.

To install the PDAL package so that we can use it to run PDAL commands, we run
the following command to create an environment named ``myenv``, installing PDAL
from the ``conda-forge`` channel. ::

    conda create --name myenv --channel conda-forge pdal

.. image:: ./images/docker-quickstart-pull.png

.. note::

    PDAL's Python extension is managed separately from the PDAL package. To
    install it, replace ``pdal`` with ``python-pdal`` in any of the commands in
    this section. Seeing as how PDAL is a dependency of the Python extension,
    you will actually get two for the price of one!

To install PDAL to an existing environment names ``myenv``, we would run the
following command. ::

    conda install --name myenv --channel conda-forge pdal

Finally, to update PDAL to the latest version, run the following. ::

    conda update pdal

.. _`conda-forge`: https://anaconda.org/conda-forge/pdal


Fetch Sample Data
------------------------------------------------------------------------------

We need some sample data to play with, so we're going to download the
``autzen.laz`` file. Inside your terminal (assuming Windows), issue the
following command: ::

    explorer.exe http://www.liblas.org/samples/autzen/autzen.laz

In the download dialog, save the file to your ``Downloads`` folder, e.g.,
``C:/Users/hobu/Downloads``.


Print the first point
------------------------------------------------------------------------------

To print the first point only, issue the following command (replacing of course
``hobu`` with your user name, or another path altogether, depending on where
you saved the file).

::

    pdal info C:/Users/hobu/Downloads/autzen.laz -p 0

Here's a summary of what's going on with that command invocation

1. ``pdal``: We're going to run the ``pdal`` command.

2. ``info``: We want to run :ref:`info_command` on the data.

3. ``autzen.laz``: The ``autzen.laz`` file that we want information from.

.. image:: ./images/docker-print-one.png


What's next?
------------------------------------------------------------------------------

* Visit :ref:`apps` to find out how to utilize PDAL applications to process
  data on the command line yourself.
* Visit :ref:`development_index` to learn how to embed and use PDAL in your own
  applications.
* :ref:`readers` lists the formats that PDAL can read, :ref:`filters` lists the
  kinds of operations you can do with PDAL, and :ref:`writers` lists the
  formats PDAL can write.
* :ref:`tutorial` contains a number of walk-through tutorials for achieving
  many tasks with PDAL.
* :ref:`The PDAL workshop <workshop>` contains numerous hands-on examples with screenshots and
  example data of how to use PDAL :ref:`apps` to tackle point cloud data
  processing tasks.
* :ref:`python` describes how PDAL embeds and extends Python and
  how you can leverage these capabilities in your own programs.

.. seealso::

    :ref:`community` is a good source to reach out to when you're stuck.

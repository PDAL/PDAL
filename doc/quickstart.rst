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

    conda create --yes --name myenv --channel conda-forge pdal

Depending on what packages you may or may not have already installed, the
output should look something like: ::

    Solving environment: done

    ## Package Plan ##

      environment location: C:\Miniconda3\envs\myenv

      added / updated specs:
        - pdal


    The following packages will be downloaded:

        package                    |            build
        ---------------------------|-----------------
        pdal-1.7.2                 |   py35h33f895e_1         8.6 MB  conda-forge
        setuptools-39.2.0          |           py35_0         591 KB  conda-forge
        numpy-1.14.3               |   py35h9fa60d3_2          42 KB
        ------------------------------------------------------------
                                               Total:         9.2 MB

    The following NEW packages will be INSTALLED:

        boost:           1.66.0-py35_vc14_1    conda-forge [vc14]
        boost-cpp:       1.66.0-vc14_1         conda-forge [vc14]
        ca-certificates: 2018.4.16-0           conda-forge
        cairo:           1.14.10-vc14_0        conda-forge [vc14]
        certifi:         2018.4.16-py35_0      conda-forge
        curl:            7.60.0-vc14_0         conda-forge [vc14]
        expat:           2.2.5-vc14_0          conda-forge [vc14]
        flann:           1.9.1-h0953f56_2      conda-forge
        freexl:          1.0.5-vc14_0          conda-forge [vc14]
        geotiff:         1.4.2-vc14_1          conda-forge [vc14]
        hdf4:            4.2.13-vc14_0         conda-forge [vc14]
        hdf5:            1.10.1-vc14_2         conda-forge [vc14]
        hexer:           1.4.0-vc14_1          conda-forge [vc14]
        icc_rt:          2017.0.4-h97af966_0
        icu:             58.2-vc14_0           conda-forge [vc14]
        intel-openmp:    2018.0.3-0
        jpeg:            9b-vc14_2             conda-forge [vc14]
        jsoncpp:         1.8.1-vc14_0          conda-forge [vc14]
        kealib:          1.4.7-vc14_4          conda-forge [vc14]
        krb5:            1.14.6-vc14_0         conda-forge [vc14]
        laszip:          3.2.2-vc14_0          conda-forge [vc14]
        laz-perf:        1.2.0-vc14_1          conda-forge [vc14]
        libgdal:         2.2.4-vc14_4          conda-forge [vc14]
        libiconv:        1.15-vc14_0           conda-forge [vc14]
        libnetcdf:       4.6.1-vc14_2          conda-forge [vc14]
        libpng:          1.6.34-vc14_0         conda-forge [vc14]
        libpq:           9.6.3-vc14_0          conda-forge [vc14]
        libspatialite:   4.3.0a-vc14_19        conda-forge [vc14]
        libssh2:         1.8.0-vc14_2          conda-forge [vc14]
        libtiff:         4.0.9-vc14_0          conda-forge [vc14]
        libxml2:         2.9.8-vc14_0          conda-forge [vc14]
        libxslt:         1.1.32-vc14_0         conda-forge [vc14]
        mkl:             2018.0.3-1
        mkl_fft:         1.0.2-py35_0          conda-forge
        mkl_random:      1.0.1-py35_0          conda-forge
        nitro:           2.7.dev2-vc14_0       conda-forge [vc14]
        numpy:           1.14.3-py35h9fa60d3_2
        numpy-base:      1.14.3-py35h5c71026_0
        openjpeg:        2.3.0-vc14_2          conda-forge [vc14]
        openssl:         1.0.2o-vc14_0         conda-forge [vc14]
        pcl:             1.8.1-hd76163c_1      conda-forge
        pdal:            1.7.2-py35h33f895e_1  conda-forge
        pip:             9.0.3-py35_0          conda-forge
        pixman:          0.34.0-vc14_2         conda-forge [vc14]
        postgresql:      10.3-py35_vc14_0      conda-forge [vc14]
        proj4:           4.9.3-vc14_5          conda-forge [vc14]
        python:          3.5.5-1               conda-forge
        setuptools:      39.2.0-py35_0         conda-forge
        sqlite:          3.20.1-vc14_2         conda-forge [vc14]
        tiledb:          1.4.1                 conda-forge
        vc:              14-0                  conda-forge
        vs2015_runtime:  14.0.25420-0          conda-forge
        wheel:           0.31.0-py35_0         conda-forge
        wincertstore:    0.2-py35_0            conda-forge
        xerces-c:        3.2.0-vc14_0          conda-forge [vc14]
        xz:              5.2.3-0               conda-forge
        zlib:            1.2.11-vc14_0         conda-forge [vc14]

    Downloading and Extracting Packages
    pdal-1.7.2           |  8.6 MB | ###################################### | 100%
    setuptools-39.2.0    |  591 KB | ###################################### | 100%
    numpy-1.14.3         |   42 KB | ###################################### | 100%
    Preparing transaction: done
    Verifying transaction: done
    Executing transaction: done
    #
    # To activate this environment, use
    #
    #     $ conda activate myenv
    #
    # To deactivate an active environment, use
    #
    #     $ conda deactivate

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

    explorer.exe https://github.com/PDAL/data/raw/master/autzen/autzen.laz

In the download dialog, save the file to your ``Downloads`` folder, e.g.,
``C:\Users\hobu\Downloads``.


Print the first point
------------------------------------------------------------------------------

To print the first point only, issue the following command (replacing of course
``hobu`` with your user name, or another path altogether, depending on where
you saved the file).

::

    pdal info C:\Users\hobu\Downloads\autzen.laz -p 0

Here's a summary of what's going on with that command invocation

1. ``pdal``: We're going to run the ``pdal`` command.

2. ``info``: We want to run :ref:`info_command` on the data.

3. ``autzen.laz``: The ``autzen.laz`` file that we want information from.

::

    Warning 1: Cannot find datum.csv or gdal_datum.csv
    Warning 1: Cannot find ellipsoid.csv
    {
      "filename": "C:\\Users\\hobu\\Downloads\\autzen.laz",
      "pdal_version": "1.7.2 (git-version: Release)",
      "points":
      {
        "point":
        {
          "Blue": 93,
          "Classification": 1,
          "EdgeOfFlightLine": 0,
          "GpsTime": 245379.3984,
          "Green": 102,
          "Intensity": 4,
          "NumberOfReturns": 1,
          "PointId": 0,
          "PointSourceId": 7326,
          "Red": 84,
          "ReturnNumber": 1,
          "ScanAngleRank": -17,
          "ScanDirectionFlag": 0,
          "UserData": 128,
          "X": 637177.98,
          "Y": 849393.95,
          "Z": 411.19
        }
      }
    }


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

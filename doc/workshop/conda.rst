.. _workshop-conda:


Conda
================================================================================

.. index:: Conda, software installation

.. include:: ./includes/substitutions.rst




What is Conda
--------------------------------------------------------------------------------

Conda is an open source package management system and environment management system that runs
on Windows, macOS and Linux. Conda quickly installs, runs, and updates packages and their dependencies.
Conda easily creates, saves, loads and switches between environments on your local computer.
It was created for Python programs, but it can package and distribute software for any language.


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
    is to go through the workshop using :ref:`docker <development_docker>` as your platform. A
    previous edition of the workshop was provided using Docker, but it was
    found to be a bit too difficult to follow.

.. note::

    PDAL does not have a python wheel package and thus is distributed via ``conda-forge`` conda 
    channel. If you would like to know more about the limitations that prevent PDAL from being 
    distributed as a pip package, you can read about it at the 
    `pypackaging-native website`_


Installing Conda Environment (Workshop USB)
--------------------------------------------------------------------------------

#. From the installer directory, install both QGIS_, CloudCompare_, and mambaforge_
#. Once mambaforge_ is installed, open a terminal and navigate to the location of your USB drive
#. Uncompress the environment, activate it, and use the ``conda-unpack`` command
  
  * macOS Users 

     .. code-block:: bash

        $ mkdir -p "$HOME/mambaforge/envs/pdal-workshop"
        $ tar -xzf conda_environments/pdal-workshop_osx-arm64.tar.gz -C "$HOME/mambaforge/envs/pdal-workshop"
        $ source "$HOME/mambaforge/envs/pdal-workshop/bin/activate"
        (pdal-workshop) $ conda-unpack
  * Windows Users 

     .. code-block:: doscon
      
        > mkdir "%userprofile%\mambaforge\envs\pdal-workshop"
        > tar -xvf ./conda_environments/pdal-workshop-win64.zip -C "%userprofile%\mambaforge\envs\pdal-workshop"
        > call "%userprofile%\mambaforge\envs\pdal-workshop\scripts\activate"
        (pdal-workshop) > conda-unpack 


Installing Conda 
--------------------------------------------------------------------------------

1. Copy the entire contents of your workshop USB key to a ``PDAL`` directory in your
   home directory (something like ``C:\Users\hobu\PDAL``) or the equivalent for your OS.
   We will refer to this location for the rest of the workshop materials.

2. Download and install the mambaforge_ installer for your platform

3. After installing mambaforge create your workshop by running

   .. code-block:: doscon

      > conda create -c conda-forge -n pdal-workshop 


4. Then *activate* the new environment
    
   .. code-block:: doscon

      > conda activate pdal-workshop


5. Install PDAL, Entwine, and GDAL, and install it from **conda-forge**

   .. code-block:: doscon
    
      (pdal-workshop) > mamba install -c conda-forge python-pdal gdal entwine matplotlib


Alternatively use the following ``environment.yml`` file to create your environment

  .. literalinclude:: environment.yml
    :language: yaml


  .. note:: 

    The ``conda-pack`` package is used for packaging the pdal-workshop conda environment to go
    to the USB image, and is not needed otherwise

.. _mambaforge: https://github.com/conda-forge/miniforge#mambaforge
.. _pypackaging-native website: https://pypackaging-native.github.io/key-issues/native-dependencies/geospatial_stack/
.. _QGIS: https://www.qgis.org/en/site/
.. _cool-lidar: https://github.com/hobuinc/cool-lidar
.. _CloudCompare: https://www.danielgm.net/cc/

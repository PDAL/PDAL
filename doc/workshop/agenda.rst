.. _introduction:

Introduction
================================================================================
After the basic set up the workshop is divided into three 45 to 90 min sections with a final project at the end.

1. :ref:`lidar-introduction`

2. :ref:`Introduction to PDAL <about>`

3. :ref:`software`

4. :ref:`basic-info`

5. :ref:`manipulation`

6. :ref:`generation`

7. :ref:`capstone`



Materials
--------------------------------------------------------------------------------

USB Drive
................................................................................

A companion USB drive containing everything needed to do the workshop without
internet connectivity

.. image:: ./images/agenda-usb-drive.jpg
    :width: 400

.. note::

    A drive image is available for download at
    https://pdal.s3.amazonaws.com/workshop/PDAL-Workshop-complete.zip


Included in the USB driver are

* Installers for QGIS_ and mambaforge_.
* Conda environments for macOS (x86_64 and arm64) and Windows platforms with needed dependencies
* Copy of PDAL documentation
* Collection of cool-lidar_ datasets

Offline Installation Instructions
.................................


#. From the installer directory, install both QGIS_ and mambaforge_
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

#. Success


.. _QGIS: https://www.qgis.org/en/site/
.. _mambaforge: https://github.com/conda-forge/miniforge#mambaforge
.. _cool-lidar: https://github.com/hobuinc/cool-lidar
.. _vagrant:

******************************************************************************
Getting Started with PDAL using `Vagrant`_ and `VirtualBox`_
******************************************************************************


:Author: Howard Butler
:Contact: hobu.inc at gmail dot com
:Date: 11/04/2013

Introduction
------------------------------------------------------------------------------

Because of PDAL's relatively short lifespan, there are not so many 
distributions and packaging solutions that have it available. This means that 
prospective users of PDAL are pretty much on their own to build up PDAL 
and use it. Folks who've climbed this mountain with other well-entangled 
software like `GDAL`_, `MapServer`_, or `Mapnik`_ should be able to manage 
the compilation challenges listed in the full set of dependencies in :ref:`building_unix`.

For everyone else, suffering all of the compilation challenges, including 
at least three different build systems is simply too much. "Give me a 
binary and be done with it," they might ask, and for them, there is an 
answer. Surrender the requirement of running PDAL in your own customized 
system for the buildout of PDAL on a known quantity -- a pre-configured 
virtual machine.

Installation
------------------------------------------------------------------------------

.. note::

    This document assumes you are starting with a basic Macintosh system 
    because this is the only system the author assumes people could 
    ever want to run :). This document has not been tested on any other 
    platforms, but it is expected that things should work if you're 
    starting from Windows or some Linux.

`Vagrant`_ is configuration management for virtual machines. Its focus is 
on reproducibility of configuration. The idea is to codify the configuration 
using simple text files, press a button, and out spits a fully configured 
virtual machine with the specified options. And for the most part, this is 
what actually happens.

`Vagrant`_ is simply the configuration management software, however. `VirtualBox`_ 
is the virtual machine container. These two tools are used in concert, 
and both are required to complete this tutorial.

1) Download and install `VirtualBox <https://www.virtualbox.org/wiki/Downloads>`__.
2) Download and install `Vagrant <http://downloads.vagrantup.com/>`__.
3) Clone PDAL to your environment:
  
    ::
  
        $ git clone https://github.com/PDAL/PDAL.git

4) Execute the vagrant configuration:

    ::
    
        vagrant up

5) Wait ...
6) Wait ...
7) Wait ...
8) Look for the following when it's all done:

    ::
    
        vagrant@pdal-development:~$ pdal info --input readpgpointcloud.xml -p 0
        {
            "X": "560974.95000000007",
            "Y": "5115235.6900000004",
            "Z": "1881.71",
            "Intensity": "0",
            "ReturnNumber": "0",
            "NumberOfReturns": "0",
            "ScanDirectionFlag": "0",
            "EdgeOfFlightLine": "0",
            "Classification": "2",
            "ScanAngleRank": "0",
            "UserData": "0",
            "PointSourceId": "21",
            "Time": "0",
            "Red": "248",
            "Green": "250",
            "Blue": "246",
            "PointID": "772536",
            "BlockID": "0"
        }
        
9) Start playing around:

    ::
        
        $ vagrant ssh
        $ pdal
        
        ------------------------------------------------------------------------------------------
        pdal (PDAL 0.9.9 (b632df) with GeoTIFF 1.4.0 GDAL 1.11dev LASzip 2.1.0 System )
        ------------------------------------------------------------------------------------------
          available actions:
             - info
             - pipeline
             - query
             - translate
        
What you get
------------------------------------------------------------------------------

The `Vagrant`_ configuration that PDAL provides contains nearly 
every possible feature except for `Oracle Point Cloud`_ support. Things it 
includes are:

* Full `pgpointcloud`_ support including example database
* :ref:`filters.hexbin`
* :ref:`filters.inplacereprojection` 
* :ref:`drivers.nitf.reader` and :ref:`drivers.nitf.writer` 
* `LASzip`_ support in :ref:`drivers.las.reader` and :ref:`drivers.las.writer`
* Interpolation with `Points2Grid`_ using :ref:`drivers.p2g.writer`
* Python manipulation using :ref:`filters.predicate` and :ref:`filters.programmable`
* `PCL`_ support via :ref:`filters.pclblock`

Head to :ref:`pipeline` for more information on using PDAL pipelines. Two pipelines 
are provided in ``/home/vagrant`` that are used to load the ``st-helens-small.las`` 
file into `pgpointcloud`_.

.. _`Points2Grid`: https://github.com/CRREL/points2grid
.. _`Oracle Point Cloud`: http://docs.oracle.com/cd/B28359_01/appdev.111/b28400/sdo_pc_pkg_ref.htm
.. _`pgpointcloud`: https://github.com/pramsey/pointcloud

.. _`LASzip`: http://laszip.org
.. _`VirtualBox`: https://www.virtualbox.org/
.. _`GDAL`: http://gdal.org
.. _`MapServer`: http://mapserver.org
.. _`Mapnik`: http://mapnik.org
.. _`PCL`: http://www.pointclouds.org

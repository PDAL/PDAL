(download)=

# Download

## Current Release(s)

- **2024-09-05** [PDAL-2.8.0-src.tar.bz2] [Release Notes] ([md5])

## Past Releases

- **2024-03-27** [PDAL-2.7.1-src.tar.bz2]
- **2024-02-05** [PDAL-2.6.3-src.tar.bz2]
- **2023-08-18** [PDAL-2.5.6-src.tar.bz2]
- **2022-03-18** [PDAL-2.4.0-src.tar.bz2]

(source)=

## Development Source

The main repository for PDAL is located on github at
<https://github.com/PDAL/PDAL>.

You can obtain a copy of the active source code by issuing the following
command

```
git clone https://github.com/PDAL/PDAL.git
```

## Binaries

In this section we list a number of the binary distributions of PDAL. The table
below is intended to provide an overview of some of the differences between the
various distributions, as not all features can be enabled in every
distribution. This table only summarizes the differences between distributions,
and there are several plugins that are not built for any of the distributions.
These include Delaunay, MATLAB, MBIO, OpenSceneGraph, RDBLIB,
and RiVLib. To enable any of these plugins, the reader will need to install any
required dependencies and build PDAL from source.

```{eval-rst}
.. csv-table:: PDAL Distribution Feature Comparison
   :header: "", "Docker", "RPMs", "Alpine", ":ref:`Conda`"
   :widths: 20, 20, 20, 20, 20

   "Platform(s)", "linux", "linux",  "linux", "win64, mac, linux"
   "PDAL version", "2.8", "",  "2.6", "2.8"
   ":ref:`readers.arrow`, :ref:`writers.arrow`", "", "",  "", "X"
   ":ref:`filters.cpd`", "", "",  "X", "X"
   ":ref:`readers.draco`, :ref:`writers.draco`", "", "",  "", "X"
   ":ref:`readers.e57`, :ref:`writers.e57`", "X", "",  "", "X"
   ":ref:`readers.hdf`", "X", "",  "", "X"
   ":ref:`readers.i3s`, :ref:`readers.slpk`", "", "", "",  "X"
   ":ref:`readers.nitf`, :ref:`writers.nitf`", "X",  "",  "", "X"
   ":ref:`readers.pgpointcloud`, :ref:`writers.pgpointcloud`", "X",  "",  "X", "X"
   ":ref:`readers.tiledb`, :ref:`writers.tiledb`", "X", "", "", "X"
   ":ref:`filters.trajectory`", "", "",  "", "X"

```

### Windows

Windows builds are available via [Conda Forge] (64-bit only). See the
{ref}`conda` for more detailed information.

### RPMs

RPMs for PDAL are available at
<https://copr.fedorainfracloud.org/coprs/neteler/pdal/>.

### Alpine

[Alpine] is a linux distribution that is compact and frequently used with
Docker images. Alpine packages for PDAL are available at
<https://pkgs.alpinelinux.org/packages?name=*pdal*&branch=edge>.

Users have a choice of three separate packages.

1\. `pdal` will install the PDAL binaries only, and is suitable for users who
will be using the PDAL command line applications.

2\. `pdal-dev` will install development files which are required for users
building their own software that will link against PDAL.

3. `py-pdal` will install the PDAL Python extension.

Note that the PDAL package now resides in Alpine's `edge/community` repository,
which must be added to your Alpine repositories list. Information on adding and
updating repositories can be found in the Alpine documentation.

To install one or more packages on Alpine, use the following command.

```
apk add [package...]
```

For example, the following command will install both the PDAL application and
the Python extension.

```
apk add py-pdal pdal
```

(conda)=

### Conda

[Conda] can be used on multiple platforms (Windows, macOS, and Linux) to
install software packages and manage environments. Conda packages for PDAL are
available at <https://anaconda.org/conda-forge/pdal>.

Conda installation instructions can be found on the Conda website. The
instructions below assuming you have a working Conda installation on your
system.

Users have a choice of two separate packages.

1. `pdal` will install the PDAL binaries **and** development files.
2. `python-pdal` will install the PDAL Python extension.

To install one or more Conda packages, use the following command.

```
conda install [-c channel] [package...]
```

Because the PDAL package (and it's dependencies) live in the [Conda Forge]
channel, the command to install both the PDAL application and the Python
extension is

```
conda install -c conda-forge pdal python-pdal gdal
```

It is strongly recommended that you make use of Conda's environment management
system and install PDAL in a separate environment (i.e., not the base
environment). Instructions can be found on the Conda website.

[alpine linux]: https://www.alpinelinux.org/
[conda forge]: https://anaconda.org/conda-forge/pdal
[md5]: https://github.com/PDAL/PDAL/releases/download/2.8.0/PDAL-2.8.0-src.tar.bz2.md5
[pdal-1.9.1-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/1.9.1/PDAL-1.9.1-src.tar.bz2
[pdal-2.0.1-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.0.1/PDAL-2.0.1-src.tar.bz2
[pdal-2.1.0-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.1.0/PDAL-2.1.0-src.tar.bz2
[pdal-2.2.0-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.2.0/PDAL-2.2.0-src.tar.bz2
[pdal-2.3.0-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.3.0/PDAL-2.3.0-src.tar.bz2
[pdal-2.4.0-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.4.0/PDAL-2.4.0-src.tar.bz2
[pdal-2.4.2-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.4.2/PDAL-2.4.2-src.tar.bz2
[pdal-2.5.0-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.5.0/PDAL-2.5.0-src.tar.bz2
[pdal-2.5.1-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.5.1/PDAL-2.5.1-src.tar.bz2
[pdal-2.5.2-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.5.2/PDAL-2.5.2-src.tar.bz2
[pdal-2.5.3-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.5.3/PDAL-2.5.3-src.tar.bz2
[pdal-2.5.4-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.5.4/PDAL-2.5.4-src.tar.bz2
[pdal-2.5.5-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.5.5/PDAL-2.5.5-src.tar.bz2
[pdal-2.5.6-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.5.6/PDAL-2.5.6-src.tar.bz2
[pdal-2.6.0-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.6.0/PDAL-2.6.0-src.tar.bz2
[pdal-2.6.1-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.6.1/PDAL-2.6.1-src.tar.bz2
[pdal-2.6.2-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.6.2/PDAL-2.6.2-src.tar.bz2
[pdal-2.6.3-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.6.3/PDAL-2.6.3-src.tar.bz2
[pdal-2.7.0-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.7.0/PDAL-2.7.0-src.tar.bz2
[pdal-2.7.1-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.7.1/PDAL-2.7.1-src.tar.bz2
[pdal-2.8.0-src.tar.bz2]: https://github.com/PDAL/PDAL/releases/download/2.8.0/PDAL-2.8.0-src.tar.bz2
[release notes]: https://github.com/PDAL/PDAL/releases/tag/2.8.0

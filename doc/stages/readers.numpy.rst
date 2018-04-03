.. _readers.numpy:

readers.numpy
================================================================================

PDAL has support for processing data using :ref:`filters.python`, but it is also
convenient to read data from `Numpy`_ for processing in PDAL.

`Numpy`_ supports saving files with the ``save`` method, usually with the
extension ``.npy``. As of PDAL 1.7.0, ``.npz`` files were not yet supported.

.. warning::

    It is untested whether the version of Python PDAL was linked against and
    the version that saved the ``.npy`` files can be mixed.

Array Types
--------------------------------------------------------------------------------

:ref:`readers.numpy` supports reading data in two forms:

* Arrays as named fields all of the same shape (from `laspy`_ for example)
* 2-dimensional arrays


Named Field Arrays
................................................................................

`laspy`_ provides its ``.points`` Numpy array as a bunch of named fields:

::

    import laspy
    f = laspy.file.File('test/data/autzen/autzen.las')
    print (f.points[0:1])

::

    array([ ((63608330, 84939865, 40735, 65, 73, 1, -11, 126, 7326,  245385.60820904),)],
          dtype=[('point', [('X', '<i4'), ('Y', '<i4'), ('Z', '<i4'), ('intensity', '<u2'), ('flag_byte', 'u1'), ('raw_classification', 'u1'), ('scan_angle_rank', 'i1'), ('user_data', 'u1'), ('pt_src_id', '<u2'), ('gps_time', '<f8')])])

:ref:`readers.numpy` supports reading these Numpy arrays and mapping applicable
names to :ref:`dimensions` names. It will try to remove ``_``, ``-``, and ``space`` from
the field name and use that as a dimension name if it can match. Types are also
preserved when mapped to PDAL.


Two-dimensional Arrays
................................................................................

Typical two-dimensional `Numpy`_ arrays are also supported, with options to allow
you to map the values in the cells using the ``dimension`` option. Additionally,
you can override the `Z` value for the entire array by using the ``assign_z``
option to set a single `Z` value for the entire point cloud. Mapping the values to the
``Z`` dimension using the ``dimension`` option is also allowed.


::

    f = open('./perlin.npy', 'rb')
    data = np.load(f,)

    data.shape
    (100, 100)

    data.dtype
    dtype('float64')


In this case, the cell locations are mapped to X and Y dimensions, the cell
values are mapped to ``Intensity`` using the ``dimension`` option, and the Z
values are assigned to 4 using the ``assign_z`` option.

::

    pdal info perlin.npy --readers.numpy.dimension=Intensity --readers.numpy.assign_z=4

::

    {
      "filename": "perlin.npy",
      "pdal_version": "1.6.0 (git-version: 897afd)",
      "stats":
      {
            "statistic":
            [
              {
                "average": 49.995,
                "count": 10000,
                "kurtosis": -1.201226882,
                "maximum": 100,
                "minimum": 0,
                "name": "X",
                "position": 0,
                "skewness": -0.0001281084091,
                "stddev": 29.16793715,
                "variance": 850.7685575
              },
              {
                "average": 50,
                "count": 10000,
                "kurtosis": -1.1996846,
                "maximum": 100,
                "minimum": 0,
                "name": "Y",
                "position": 1,
                "skewness": -8.69273658e-05,
                "stddev": 28.87401021,
                "variance": 833.7084657
              },
              {
                "average": 4,
                "count": 10000,
                "kurtosis": 9997,
                "maximum": 4,
                "minimum": 4,
                "name": "Z",
                "position": 2,
                "skewness": 1.844674407e+21,
                "stddev": 0.04000200015,
                "variance": 0.001600160016
              },
              {
                "average": 0.01112664759,
                "count": 10000,
                "kurtosis": -0.5634013693,
                "maximum": 0.5189296418,
                "minimum": -0.5189296418,
                "name": "Intensity",
                "position": 3,
                "skewness": -0.1127124452,
                "stddev": 0.2024120437,
                "variance": 0.04097063545
              }
            ]
          }
        }

.. _`Numpy`: http://www.numpy.org/
.. _`laspy`: https://github.com/laspy/laspy

.. plugin::

.. streamable::

Options
-------

filename
  npy file to read [Required]

dimension
  Dimension name from :ref:`dimensions` to map raster values
x
  Dimension number (starting from 0) to map to the ``X`` PDAL :ref:`dimension <dimensions>`

y
  Dimension number (starting from 0) to map to the ``Y`` PDAL :ref:`dimension <dimensions>`

z
  Dimension number (starting from 0) to map to the ``Z`` PDAL :ref:`dimension <dimensions>`

assign_z
  A single value to override for ``Z`` values when ``dimension`` is used to assign the
  Numpy values to another dimension

.. _formatted: http://en.cppreference.com/w/cpp/string/basic_string/stof

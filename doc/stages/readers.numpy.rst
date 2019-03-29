.. _readers.numpy:

readers.numpy
=============

PDAL has support for processing data using :ref:`filters.python`, but it is also
convenient to read data from `Numpy`_ for processing in PDAL.

`Numpy`_ supports saving files with the ``save`` method, usually with the
extension ``.npy``. As of PDAL 1.7.0, ``.npz`` files were not yet supported.

.. warning::

    It is untested whether problems may occur if the versions of Python used
    in writing the file and for reading the file don't match.

Array Types
-----------

readers.numpy supports reading data in two forms:

* As a `structured array`_ with specified field names (from `laspy`_ for
  example)
* As a standard array that contains data of a single type.



Structured Arrays
.................

Numpy arrays can be created as structured data, where each entry is a set
of fields.  Each field has a name.  As an example, `laspy`_ provides its
``.points`` as an array of named fields:

::

    import laspy
    f = laspy.file.File('test/data/autzen/autzen.las')
    print (f.points[0:1])

::

    array([ ((63608330, 84939865, 40735, 65, 73, 1, -11, 126, 7326,  245385.60820904),)],
    dtype=[('point', [('X', '<i4'), ('Y', '<i4'), ('Z', '<i4'), ('intensity', '<u2'), ('flag_byte', 'u1'), ('raw_classification', 'u1'), ('scan_angle_rank', 'i1'), ('user_data', 'u1'), ('pt_src_id', '<u2'), ('gps_time', '<f8')])])

The numpy reader supports reading these Numpy arrays and mapping
field names to standard PDAL :ref:`dimension <dimensions>` names.
If that fails, the reader retries by removing ``_``, ``-``, or ``space``
in turn.  If that also fails, the array field names are used to create
custom PDAL dimensions.


Standard (non-structured) Arrays
................................

Arrays without field information contain a single datatype.  This datatype is
mapped to a dimension specified by the ``dimension`` option.

::

    f = open('./perlin.npy', 'rb')
    data = np.load(f,)

    data.shape
    (100, 100)

    data.dtype
    dtype('float64')

::

    pdal info perlin.npy --readers.numpy.dimension=Intensity --readers.numpy.assign_z=4

::

    {
      "filename": "..\/test\/data\/plang\/perlin.npy",
      "pdal_version": "1.7.1 (git-version: 399e19)",
      "stats":
      {
        "statistic":
        [
          {
            "average": 49.5,
            "count": 10000,
            "maximum": 99,
            "minimum": 0,
            "name": "X",
            "position": 0,
            "stddev": 28.86967866,
            "variance": 833.4583458
          },
          {
            "average": 49.5,
            "count": 10000,
            "maximum": 99,
            "minimum": 0,
            "name": "Y",
            "position": 1,
            "stddev": 28.87633116,
            "variance": 833.8425015
          },
          {
            "average": 0.01112664759,
            "count": 10000,
            "maximum": 0.5189296418,
            "minimum": -0.5189296418,
            "name": "Intensity",
            "position": 2,
            "stddev": 0.2024120437,
            "variance": 0.04097063545
          }
        ]
      }
    }


X, Y and Z Mapping
................................................................................
Unless the X, Y or Z dimension is specified as a field in a structured array,
the reader will create dimensions X, Y and Z as necessary and populate them
based on the position of each item of the array.  Although Numpy arrays always
contain contiguous, linear data, that data can be seen to be arranged in more
than one dimension.  A two-dimensional array will cause dimensions X and Y
to be populated.  A three dimensional array will cause X, Y and Z to be
populated.  An array of more than three dimensions will reuse the X, Y and Z
indices for each dimension over three.

When reading data, X Y and Z can be assigned using row-major (C) order or
column-major (Fortran) order by using the ``order`` option.


.. _`Numpy`: http://www.numpy.org/
.. _`laspy`: https://github.com/laspy/laspy
.. _`structured array`: https://docs.scipy.org/doc/numpy/user/basics.rec.html

.. plugin::

.. streamable::

Options
-------

filename
  npy file to read [Required]

.. include:: reader_opts.rst

dimension
  :ref:`Dimension <dimensions>` name to map raster values

order
  Either 'row' or 'column' to specify assigning the X,Y and Z values
  in a row-major or column-major order. [Default: matches the natural
  order of the array.]

.. note::
    The functionality of the 'assign_z' option in previous versions is
    provided with :ref:`filters.assign`

    The functionality of the 'x', 'y', and 'z' options in previous versions
    are generally handled with the current 'order' option.

.. _formatted: http://en.cppreference.com/w/cpp/string/basic_string/stof

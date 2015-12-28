.. _python_installation:

********************************************************************
Python installation
********************************************************************

Since PDAL 1.0.0 there is a Python extension to execute pipelines
and read its output as a numpy array.

To install it need to compile and install :ref:`PDAL <building>` and
then install the PDAL Python extension

Install from local
-------------------------
In the source code of PDAL there is a ``python`` folder, you have to enter
there and run ::

    python setup.py build
    # this should be run as administrator/super user
    python setup.py install

Install from repository
--------------------------
The second method to install the PDAL Python extension is to use `pip`_
or `easy_install`_, you have to run the command as administrator. ::

    pip install PDAL

.. note::

    To install pip please read
    `here <https://pip.pypa.io/en/stable/installing/>`_

.. _`pip`: https://pip.pypa.io/en/stable/
.. _`easy_install`: https://pypi.python.org/pypi/setuptools

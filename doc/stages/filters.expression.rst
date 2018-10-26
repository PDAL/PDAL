.. _filters.expression:

filters.expression
==================

.. contents::

The expression filter applies query logic to the input point cloud based on a
MongoDB-style query expression using the point cloud attributes.

.. embed::

.. streamable::

Pipeline Example
----------------

This example passes through only the points whose Classification is non-zero.

.. code-block:: json

    [
        "input.las",
        {
            "type": "filters.expression",
            "expression": {
                "Classification": { "$ne": 0 }
            }
        },
        "filtered.las"
    ]

Options
-------

expression
    A JSON query :ref:`expression` containing a combination of query comparisons
    and logical operators.


.. _expression:

Expression
--------------------------------------------------------------------------------

A query expression is a combination of comparison and logical operators that
define a query which can be used to select matching points by their attribute
values.

Comparison operators
................................................................................

There are 8 valid query comparison operators:
    - ``$eq``: Matches values equal to a specified value.
    - ``$gt``: Matches values greater than a specified value.
    - ``$gte``: Matches values greater than or equal to a specified value.
    - ``$lt``: Matches values less than a specified value.
    - ``$lte``: Matches values less than or equal to a specified value.
    - ``$ne``: Matches values not equal to a specified value.
    - ``$in``: Matches any of the values specified in the array.
    - ``$nin``: Matches none of the values specified in the array.

Comparison operators compare a point cloud attribute with a value or an array
of values.  For all comparison operators except for ``$in`` and ``$nin``, the
value must be a number.  For ``$in`` and ``$nin``, the value must be an array
of numbers.

Comparison operators are applied directly to attribute values, and thus must be
contained within an attribute selection by which an attribute is selected by its
name.  For example:

.. code-block:: json

    { "Classification": { "$eq": 2 } }

.. code-block:: json

    { "Intensity": { "$gt": 0 } }

.. code-block:: json

    { "Classification": { "$in": [2, 6, 9] } }

The ``$eq`` comparison operator may be implicitly invoked by setting an
attribute name directly to a value.

.. code-block:: json

    { "Classification": 2 }

Logical operators
................................................................................

There are 4 valid logical operators:
    - ``$and``: Applies a logical **and** on the expressions of the array and
      returns a match only if all expressions match.
    - ``$not``: Inverts the value of the single sub-expression.
    - ``$nor``: Applies a logical **nor** on the expressions of the array and
      returns a match only if all expressions fail to match.
    - ``$nor``: Applies a logical **or** on the expressions of the array and
      returns a match if any of the expressions match.

Logical operators are used to logically combine sub-expressions.  All logical
operators except for ``$not`` are applied to arrays of expressions.
``$not`` is applied to a single expression and negates its result.

Logical operators may be applied directly to comparison expressions or may
contain further nested logical operators.  For example:

.. code-block:: json

    { "$or": [
        { "Classification": 2 },
        { "Intensity": { "$gt": 0 } }
    ] }

.. code-block:: json

    { "$or": [
        { "Classification": 2 },
        { "$and": [
            { "ReturnNumber": { "$gt": 0 } },
            { "Z": { "$lte": 42 } }
        ] }
    ] }

.. code-block:: json

    { "$not": {
        "$or": [
            { "Classification": 2 },
            { "$and": [
                { "ReturnNumber": { "$gt": 0 } },
                { "Z": { "$lte": 42 } }
            ] }
        ] }
    }


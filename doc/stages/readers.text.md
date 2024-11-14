(readers.text)=

# readers.text

The **text reader** reads data from ASCII text files.  Each point is
represented in the file as a single line.  Each line is expected to be divided
into a number of fields by a separator.  Each field represents a value for
a point's dimension.  Each value needs to be [formatted] properly for
C++ language double-precision values.

The text reader expects a header line to indicate the dimensions are
in each subsequent line.  There are two types of header lines.

## Quoted dimension names

When the first character of the header is a double quote, each dimension name
is assumed to be surrounded by double quotes.  A single separator character
is expected between the dimension names (spaces are stripped).  If no separator
character is found, a space is assumed.  You can set the [separator] character
if it differs from that in the header.  Note that PDAL requires dimension
names that consist only of alphabetic characters and underscores.  Edit
the header line or use the [header] option to set the dimension names to
ones that PDAL understands.

## Unquoted dimension names

The first non alpha-numeric character encountered is treated as a separator
between dimension names.  The separator in the header line can be overridden
by the [separator] option.

Each line in the
file must contain the same number of fields as indicated by
dimension names in the header.  Spaces are generally ignored in the input
unless used as a separator.  When a space character is used as a separator,
any number of consecutive spaces are treated as single space and
leading/trailing spaces are ignored.

Blank lines are ignored after the header line is read.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Example Input File

This input file contains X, Y and Z value for 10 points.

```
X,Y,Z
289814.15,4320978.61,170.76
289814.64,4320978.84,170.76
289815.12,4320979.06,170.75
289815.60,4320979.28,170.74
289816.08,4320979.50,170.68
289816.56,4320979.71,170.66
289817.03,4320979.92,170.63
289817.53,4320980.16,170.62
289818.01,4320980.38,170.61
289818.50,4320980.59,170.58
```

## Example #1

```json
[
    {
        "type":"readers.text",
        "filename":"inputfile.txt"
    },
    {
        "type":"writers.text",
        "filename":"outputfile.txt"
    }
]
```

## Example #2

This reads the data in the input file as Red, Green and Blue instead of
as X, Y and Z.

```json
[
    {
        "type":"readers.text",
        "filename":"inputfile.txt",
        "header":"Red, Green, Blue",
        "skip":1
    },
    {
        "type":"writers.text",
        "filename":"outputfile.txt"
    }
]
```

## Options

filename

: text file to read, or “STDIN” to read from standard in \[Required\]

```{include} reader_opts.md
```

header

: String to use as the file header.  All lines in the file are assumed to be
  records containing point data unless skipped with the [skip] option.
  \[Default: None\]

separator

: Separator character to override that found in header line. \[Default: None\]

skip

: Number of lines to ignore at the beginning of the file. \[Default: 0\]

[formatted]: http://en.cppreference.com/w/cpp/string/basic_string/stof

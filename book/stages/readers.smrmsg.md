(readers-smrmsg)=

# readers.smrmsg

The **SMRMSG reader** read from POSPac MMS post-processed accuracy files, used to describes the accuracy of the post-processed solution (SBET file) and
contains the position, orientation and velocity RMS after smoothing. See {ref}`writers.sbet`.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Example

```json
[
    "smrmsg_xxx.out",
    "output.txt"
]
```

## Options

filename

: File to read from \[Required\]

```{eval-rst}
.. include:: reader_opts.rst
```

(development-docs)=

# Docs

## Requirements

To build the PDAL documentation yourself, you need to install the following
items:

- [Sphinx]
- [Breathe]
- [Doxygen]
- [Latex]
- [dvipng]

### [Sphinx] and [Breathe]

Python dependencies should be installed from [PyPI] with `pip` or
`easy_install`.

```bash
(sudo) pip install sphinx sphinxconfig-bibtex breathe
```

:::{note}
If you are installing these packages to a system-wide directory, you may need
the **sudo** in front of the **pip**, though it might be better that instead
you use [virtual environments] instead of installing the packages system-wide.
:::

### Doxygen

The PDAL documentation also depends on [Doxygen], which can be installed from
source or from binaries from the [doxygen website](http://www.stack.nl/~dimitri/doxygen/download.html).  If you are on Max OS X
and use [homebrew], you can install doxygen with a simple `brew install
doxygen`.

### Latex

[Latex] and [pdflatex] are used to generate the companion PDF of the website.

### dvipng

For math output, we depend on [dvipng] to turn [Latex] output into math PNGs.

## Generation

Once you have installed all the doc dependencies, you can then build the
documentation itself.  The {file}`doc/` directory in the PDAL source tree
contains a Makefile which can be used to build all documentation.  For a list
of the output formats supported by Sphinx, simply type `make`.  For example,
to build html documentation:

```bash
cd doc
make doxygen html
```

The html docs will be placed in {file}`doc/build/html/`.  The `make doxygen`
is necessary to re-generate the API documentation from the source code using
[Breathe] and [Sphinx].

:::{note}
For a full build of the {ref}`cppapi` documentation, you need to
`make doxygen` to have it build its XML output which is consumed
by [Breathe] before `make html` can be issued.
:::

## Website

The <http://pdal.io> website is regenerated from the `*-maintenance` branch using
Github Actions. It will be committed by the `PDAL-docs` [GitHub] user and pushed
to the <https://github.com/PDAL/pdal.github.io> repository. The website
is then served via [GitHub Pages].

[breathe]: https://github.com/michaeljones/breathe
[doxygen]: http://www.stack.nl/~dimitri/doxygen/
[dvipng]: https://en.wikipedia.org/wiki/Dvipng
[github]: http://github.com/PDAL/PDAL
[github pages]: https://pages.github.com/
[homebrew]: http://mxcl.github.io/homebrew/
[latex]: https://en.wikipedia.org/wiki/LaTeX
[pdflatex]: https://www.tug.org/applications/pdftex/
[pypi]: https://pypi.python.org/pypi
[sphinx]: http://sphinx-doc.org/
[virtual environments]: https://pypi.python.org/pypi/virtualenv

(development-docs)=

# Docs

We have recently converted our documentation to use [Jupyter Book], which [plays nicely with](https://jupyterbook.org/en/stable/publish/readthedocs.html) our documentation hosting service [Read the Docs]. A key motivation in doing so is to enable first class support for executable code cells and even full Jupyter notebooks!

[Jupyter Book] uses [MyST] (Markedley Structured Text) Markdown (see their [overview](https://jupyterbook.org/en/stable/content/myst.html) for details). [MyST] supports many of the same reStructuredText directives already in use within PDAL's documentation, though the look and feel of the raw docs will certainly have more of Markdown feel to them.

Rather than provide our own [Jupyter Book] and [MyST] markdown overview, we'll encourage developers to visit the respective projects to learn more. And be sure to browse our new docs to familiarize yourself with how we have adopted it.

Some key points to remember when writing documentation.

1. Start by creating a new `.md` file within the `doc` folder and populating it with your desired content.

2. Next (or first, it doesn't really matter!), take a look at `_toc.yml` and add a reference to your new file where you would like it to appear within the rendered docs. More on the TOC and structure of Jupyter Books [here](https://jupyterbook.org/en/stable/structure/toc.html).

3. If your documentation has any references cited, be sure to add the corresponding BibTex entry to `references.bib` at the root of the `doc` folder.

4. If you have added [executable content](https://jupyterbook.org/en/stable/content/executable/index.html) to your docs and it requires a new package, please add it to `environment.yml` at the root of the `doc` folder so that [Read the Docs] will be able to properly setup the environment.

5. Documentation is automatically built for each GitHub Pull Request commit. You can always preview the rendered documentation by following the links in the pull request.

## Generation

PDAL's docs were recently updated to build using the [Jupyter Book CLI](https://jupyterbook.org/en/stable/reference/cli.html). Instructions to set up the environment can be found below. Once your environment is setup and `jupyter-book` is installed, the simplest way to build documentation is `jb build doc`.

### Conda

Python dependencies for building documentation can be installed from **conda-forge**
using the `environment.yml` files in the `doc` directory.

```doscon
> mamba env create -n pdal-docs -f ./doc/environment.yml
> mamba env update -n pdal-docs -f ./doc/local-build-environment.yml
> mamba activate pdal-docs
```

### CMake

[CMake] support for building PDAL documentation has recently been added, allowing Build targets for all Jupyter Book's [supported builders](https://jupyterbook.org/en/stable/reference/cli.html#cmdoption-jupyter-book-build-builder) are created automatically; other [sphinx builders](https://www.sphinx-doc.org/en/master/usage/builders/index.html) can be added by specifying `cmake -DCUSTOM_SPHINX_BUILDER=<builder>`. With [Doxygen] and [Breathe] installed, API documentation can also be created using the `doxygen` target.

When compiling PDAL from source, the `BUILD_DOCS` CMake option controls the creation of targets:

```
> mkdir build
> cd build
> cmake -G Ninja -D WITH_DOCS=ON ..
> ninja html
```

By default, all Jupyter Book outputs are created in the {file}`doc/_build` directory. You can specify a new output location with `cmake -D JB_BUILD_LOCATION`, but keep in mind that Jupyter Book always creates a `_build` directory in the location you specify.

CMake can also be used to build standalone in the {file}`doc/` directory, which doesn't require PDAL's dependencies to be installed:

```
> mkdir doc/_build
> cd doc/_build
> cmake -G Ninja ..
> ninja doxygen
> ninja html
```

```{note}
For a full build of the {ref}`cppapi` documentation, you need to
`make doxygen` to have it build its XML output which is consumed
by [Breathe] before `html` can be built.
```

### Latex

[Latex] and [pdflatex] are used to generate the companion PDF of the website.

### dvipng

For math output, we depend on [dvipng] to turn [Latex] output into math PNGs.

## Pixi

PDAL has begun to adopt `pixi` for many tasks. This is awesome news for the documentation! Instead of following the steps outlined in the "Building Locally" section above, you can now run the following command from the root of the PDAL repository.

```
pixi run -e doc doxygen
pixi run -e doc build
pixi run -e doc preview
```

If adding dependencies required for building the documentation, as in step 4, the process is slightly different. While it is convenient to add a new dependency using

```
pixi add mynewdependency
```

this will add the dependency to the default environment, whereas we build in a custom `doc` environment. The current solution will be to add a new line under

```
[feature.doc.dependencies]
...existing dependencies...
mynewdependency = "*"
```

in the `pixi.toml` manifest. Of course, if specific versions are required, this can be specified using the [VersionSpec](https://pixi.sh/latest/reference/pixi_manifest/#the-dependencies-tables).


[Jupyter Book]: https://jupyterbook.org/en/stable/intro.html#\
[cmake]: http://www.cmake.org
[Read the Docs]: https://about.readthedocs.com/
[MyST]: https://mystmd.org/
[Mamba]: https://mamba.readthedocs.io/en/latest/index.html
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/
[Breathe]: https://github.com/michaeljones/breathe
[latex]: https://en.wikipedia.org/wiki/LaTeX
[pdflatex]: https://www.tug.org/applications/pdftex/
[dvipng]: https://en.wikipedia.org/wiki/Dvipng

(documentation-quickstart)=

# Documentation Quickstart

We have recently converted our documentation to use [Jupyter Book], which [plays nicely with](https://jupyterbook.org/en/stable/publish/readthedocs.html) our documentation hosting service [Read the Docs]. A key motivation in doing so is to enable first class support for executable code cells and even full Jupyter notebooks!

[Jupyter Book] uses [MyST] (Markedley Structured Text) Markdown (see their [overview](https://jupyterbook.org/en/stable/content/myst.html) for details). [MyST] supports many of the same reStructuredText directives already in use within PDAL's documentation, though the look and feel of the raw docs will certainly have more of Markdown feel to them.

Rather than provide our own [Jupyter Book] and [MyST] markdown overview, we'll encourage developers to visit the respective projects to learn more. And be sure to browse our new docs to familiarize yourself with how we have adopted it.

Some key points to remember when writing documentation.

1. Start by creating a new `.md` file within the `doc` folder and populating it with your desired content.

2. Next (or first, it doesn't really matter!), take a look at `_toc.yml` and add a reference to your new file where you would like it to appear within the rendered docs. More on the TOC and structure of Jupyter Books [here](https://jupyterbook.org/en/stable/structure/toc.html).

3. If your documentation has any references cited, be sure to add the corresponding BibTex entry to `references.bib` at the root of the `doc` folder.

4. If you have added [executable content](https://jupyterbook.org/en/stable/content/executable/index.html) to your docs and it requires a new package, please add it to `environment.yml` at the root of the `doc` folder so that [Read the Docs] will be able to properly setup the environment.

5. Documentation is automatically built for each GitHub Pull Request commit. You can always preview the rendered documentation by following the links in the pull request.

6. Documentation can be built locally as well using the [Jupyter Book CLI](https://jupyterbook.org/en/stable/reference/cli.html). I'm currently using the following Mamba environment to setup my environment. Once your environment is setup and `jupyter-book` is installed, you should be able to simply run `jb build doc`.

    ```
    name: jb-build-pdal-docs
    channels:
    - conda-forge
    dependencies:
    - python-pdal
    - jupyter-book
    - ca-certificates
    - certifi
    - openssl
    - sphinx-notfound-page
    - sphinxcontrib-jquery
    - sphinxcontrib-bibtex
    ```

[Jupyter Book]: https://jupyterbook.org/en/stable/intro.html#
[Read the Docs]: https://about.readthedocs.com/
[MyST]: https://mystmd.org/
[Mamba]: https://mamba.readthedocs.io/en/latest/index.html
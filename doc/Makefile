# Makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    = -E -W --keep-going
SPHINXBUILD   = sphinx-build
PAPER         =
BUILDDIR      = build
DATE=$(shell date +'%y.%m.%d %H:%M:%S')

# Internal variables.
PAPEROPT_a4     = -D latex_paper_size=a4
PAPEROPT_letter = -D latex_paper_size=letter
ALLSPHINXOPTS   = -d $(BUILDDIR)/doctrees $(PAPEROPT_$(PAPER)) $(SPHINXOPTS) .
WORKSHOPOPTS    = -d $(BUILDDIR)/doctrees $(PAPEROPT_$(PAPER)) -E -c . ./workshop

.PHONY: help clean html dirhtml singlehtml pickle json htmlhelp qthelp devhelp epub latex latexpdf text man changes linkcheck doctest jupyterbook

help:
	@echo "Please use \`make <target>' where <target> is one of"
	@echo "  html       to make standalone HTML files"


clean:
	-rm -rf $(BUILDDIR)/*
	-rm -rf doxygen/xml*;
	-rm -rf doxygen/html*;
	-rm conf.py

html: jupyterbook
	$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) $(BUILDDIR)/html
	@echo
	@echo "Build finished. The HTML pages are in $(BUILDDIR)/html."

pdf: jupyterbook
	$(SPHINXBUILD) -b latex $(ALLSPHINXOPTS) $(BUILDDIR)/pdf
	cd $(BUILDDIR)/pdf
	make
	cd build/pdf
	latexmk -r latexmkrc -pdf -f -dvi- -ps- -jobname=pdal -interaction=nonstopmode
	cd ..
	@echo
	@echo "Build finished. The PDF pages are in $(BUILDDIR)/pdf."

jupyterbook: ./doxygen/xml/index.xml _config.yml
	cd .. && jupyter-book config sphinx doc/ && cd doc
	sed -i "1i\import os, sys; sys.path.append(os.path.abspath(\"./_ext\"))" conf.py
	@echo
	@echo "Built jupyter-book"

./doxygen/xml/index.xml:
	cd .. && doxygen ./doc/doxygen/doxygen.conf 1> /dev/null && cd doc
	@echo
	@echo "Built doxygen"


FROM ubuntu:16.04

RUN apt-get -y update && apt-get install -y \
    python-dev python-pip g++ doxygen dvipng \
    cmake libjpeg8-dev zlib1g-dev texlive-latex-base \
    texlive-latex-extra git \
    graphviz python-matplotlib

RUN pip install Sphinx breathe \
    sphinx_bootstrap_theme awscli sphinxcontrib-bibtex

RUN pip install git+https://github.com/rtfd/sphinx_rtd_theme.git


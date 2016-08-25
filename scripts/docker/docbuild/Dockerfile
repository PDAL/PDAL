FROM ubuntu:15.10

RUN apt-get -y update && apt-get install -y \
    python-dev python-pip g++ doxygen dvipng \
    cmake libjpeg8-dev zlib1g-dev texlive-latex-base \
    texlive-latex-extra git \
    graphviz python-matplotlib

RUN pip install Sphinx==1.4 breathe \
    sphinx_bootstrap_theme awscli sphinxcontrib-bibtex \
    sphinx_rtd_theme


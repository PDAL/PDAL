FROM ubuntu:16.04

ADD http://www.timeapi.org/utc/now /tmp/bust-cache

RUN apt-get -y update && apt-get install -y \
    python-dev python-pip g++ doxygen dvipng \
    cmake libjpeg8-dev zlib1g-dev texlive-latex-base \
    texlive-latex-extra git \
    graphviz python-matplotlib

RUN pip install Sphinx breathe \
    sphinx_bootstrap_theme awscli sphinxcontrib-bibtex \
    sphinx_rtd_theme


FROM ubuntu:15.10

RUN apt-get -y update && apt-get install -y python-dev python-pip
RUN apt-get install -y g++
RUN apt-get install -y doxygen dvipng
RUN apt-get install -y cmake libjpeg8-dev zlib1g-dev texlive-latex-base texlive-latex-extra
RUN apt-get install -y git
RUN apt-get install -y graphviz

RUN pip install Sphinx rst2pdf breathe sphinx_bootstrap_theme awscli

RUN useradd pdal
RUN mkdir /home/pdal&& chown -R pdal: /home/pdal

RUN mkdir /root/.ssh
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts
WORKDIR /home/pdal
RUN git clone https://github.com/PDAL/PDAL.git pdal
RUN git config --global push.default matching
RUN git config --global user.email "pdal@hobu.net"
RUN git config --global user.name "PDAL Automated Dockerfile"

VOLUME /data

ADD run.sh /home/pdal/run.sh

# docker run --rm --volumes-from docbuild pdal-doc-build ls -al /data
# docker create -v $(pwd)/output:/data --name docbuild pdal-doc-build

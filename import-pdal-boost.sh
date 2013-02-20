#
# file: import-pdal-boost.sh
#
# How to update the embedded copy of boost
# ========================================
#
# Prerequisites:
#    1. current build of boost, including tools/bcp
#
# Instructions:
#    1. Remove the current embedded copy of boost.
#        $ cd $PDAL_ROOT
#        $ rm -r boost
#    2. Find current included set of boost headers.
#        $ export BOOST_HEADERS=\
#             `find src include test apps \
#                  \( -name '*.[cChH]' -o \
#                     -name '*.[cChH][pPxX][pPxX]' -o \
#                     -name '*.[cChH][cChH]' \) \
#                  -exec grep 'include.*boost' {} \; \
#              | grep '^#' \
#              | sed -e 's/.*boost/boost/' -e 's/>.*//' \
#              | sort -u`
#    2.5 Make bcp
#        ./b2 tools/bcp
#    3. Run bcp.
#        $ cd $BOOST_ROOT
#        $ ../boost/boost_1_48_0/dist/bin/bcp --namespace=pdalboost --namespace-alias $BOOST_HEADERS build boost --boost=/home/oracle/boost/boost_1_48_0/
#    4. Stage.
#        $ cd $PDAL_ROOT
#        $ git add boost
#        $ git status         # verify this looks sane
#    5. Fix files (bcp misses a couple of namespace issues)
#        $ vim boost/boost/property_tree/detail/xml_parser_read_rapidxml.hpp \
#            # "namespace rapidxml" -> "namespace pdalboostrapidxml"
#    5.25 ./bootstrap.sh
#    5.5 ./b2 stage variant=release link=static threading=multi cflags="-fPIC" cxxflags="-fPIC" linkflags="-fPIC" define=DBOOST_TEST_DYN_LINK=1
#    5.75 ./b2 stage variant=release link=static threading=multi cflags="-fPIC" cxxflags="-fPIC" linkflags="-fPIC" define=DBOOST_ALL_DYN_LINK=1
#    6. Test it.
#        $ cmake -DPDAL_EMBED_BOOST=ON .
#    7. Check it in.
#        $ git commit
#
#@echo "see comments for instructions"
#exit 1

export BOOST_HOME=/home/hobu/release/boost_1_53_0/
export TARGET=./boost
export BOOST_HEADERS=`find src include test apps \( -name '*.[cChH]' -o -name '*.[cChH][pPxX][pPxX]' -o -name '*.[cChH][cChH]' \) -exec grep 'include.*boost' {} \; | grep '^#' | sed -e 's/.*boost/boost/' -e 's/>.*//' | sort -u`
export BOOST_HEADERS="$BOOST_HEADERS boost/parameter/aux_/overloads.hpp"
export NAMESPACE=pdalboost

rm -rf boost/*
echo $BOOST_HEADERS
$BOOST_HOME/dist/bin/bcp --namespace=$NAMESPACE \
                         --namespace-alias \
                         $BOOST_HEADERS \
                         filesystem iostreams fusion date_time unordered geometry multiprecision \
                         build \
                         boost \
                         --boost=$BOOST_HOME

#put back our boost-specific patches
#git cherry-pick 285ac1bc226c90deae027ae548559b1f5870cad5
#git cherry-pick 22422680a000e4ff1534a3c5ff117f8f967b86f0
git checkout boost/CMakeLists.txt

#
# FauxPlugin CMake configuration
#

PDAL_ADD_PLUGIN(fauxplugin_kernel_libname kernel fauxplugin
    FILES
        kernel/FauxPluginKernel.cpp
)

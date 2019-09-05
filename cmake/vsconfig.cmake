#
# Extra configuration for Visual Studio Generator
#

# detect if we're setting up a visual studio solution
if(CMAKE_GENERATOR MATCHES "^Visual Studio.*$")
    set(VCPROJ ON)
endif()

# add user props to build directory to add to project settings
# for running targets in the IDE
if(VCPROJ)
    file(GENERATE
        OUTPUT "${CMAKE_BINARY_DIR}/user.props"
        CONTENT "<?xml version='1.0' encoding='utf-8'?>
<Project ToolsVersion='15.0' xmlns='http://schemas.microsoft.com/developer/msbuild/2003'>
  <PropertyGroup>
    <LocalDebuggerEnvironment>PATH=${PDAL_PLUGIN_INSTALL_PATH};%PATH%</LocalDebuggerEnvironment>
  </PropertyGroup>
</Project>
"
    )
endif()

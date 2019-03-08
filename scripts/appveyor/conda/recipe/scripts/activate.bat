@REM Store existing PDAL env vars and set to this conda env
@REM so other PDAL installs don't pollute the environment

@if defined PDAL_DRIVER_PATH (
    set "_CONDA_SET_PDAL_DRIVER_PATH=%PDAL_DRIVER_PATH%"
)

@REM Support plugins if the plugin directory exists
@REM i.e if it has been manually created by the user
@set "PDAL_DRIVER_PATH=%CONDA_PREFIX%\Library\bin"
@if not exist %PDAL_DRIVER_PATH% (
     set "PDAL_DRIVER_PATH="
)


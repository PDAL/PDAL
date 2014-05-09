#pragma once

namespace pcl
{

namespace console
{

enum VERBOSITY_LEVEL
{
    L_ALWAYS,
    L_ERROR,
    L_WARN,
    L_INFO,
    L_DEBUG,
    L_VERBOSE
};

inline void setVerbosityLevel(VERBOSITY_LEVEL l)
{
(void)l;
}

} //namespace console;

} //namespace pcl;

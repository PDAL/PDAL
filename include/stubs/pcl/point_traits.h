#pragma once

#include <string>

namespace pcl
{

namespace traits
{

template<typename T>
struct fieldList
{
    typedef T type;
};

} //namespace traits

template<typename inT, typename outT>
struct CopyIfFieldExists
{
    CopyIfFieldExists(const inT& pt, const std::string& field,
        bool& exists, outT& value)
    {
        (void)pt;
        (void)field;
        (void)exists;
        (void)value;
    }
};

template<typename inT, typename outT>
struct SetIfFieldExists
{
    SetIfFieldExists(const inT& pt, const std::string& field, outT& value)
    {
        (void)pt;
        (void)field;
        (void)value;
    }
};

} //namespace pcl

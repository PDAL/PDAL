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

template<typename PointInT, typename OutT>
struct CopyIfFieldExists
{
    CopyIfFieldExists(const PointInT& pt, const std::string& field,
        bool& exists, OutT& value)
    {
        (void)pt;
        (void)field;
        (void)exists;
        (void)value;
    }

    CopyIfFieldExists(const PointInT& pt, const std::string& field,
        OutT& value)
    {
        (void)pt;
        (void)field;
        (void)value;
    }
};

template<typename PointOutT, typename InT>
struct SetIfFieldExists
{
    SetIfFieldExists(PointOutT& pt, const std::string& field, const InT& value)
    {
        (void)pt;
        (void)field;
        (void)value;
    }
};

} //namespace pcl

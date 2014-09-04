
#include <pcl/point_types.h>

#pragma once

#define PCL_ADD_POINT4D \
  union { \
    struct { \
      float x; \
      float y; \
      float z; \
    }; \
  };

#define EIGEN_ALIGN16
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define POINT_CLOUD_REGISTER_POINT_STRUCT(a, b)

namespace pcl
{

class PCDReader
{
public:
    template<typename T>
    int read(const std::string& filename, PointCloud<T>& cloud,
        const int offset = 0)
    {
        (void)filename;
        (void)cloud;
        (void)offset;
        return 0;
    }
    int readHeader(const std::string& filename, PCLPointCloud2& cloud,
        const int offset = 0)
    {
        (void)filename;
        (void)cloud;
        (void)offset;
        return 0;
    }
};

class PCDWriter
{
public:
    template<typename T>
    int writeBinaryCompressed(const std::string& filename,
        const PointCloud<T>& cloud)
    {
        (void)filename;
        (void)cloud;
        return 0;
    }
    template<typename T>
    int writeASCII(const std::string& filename, const PointCloud<T>& cloud,
        int precision = 8)
    {
        (void)filename;
        (void)cloud;
        (void)precision;
        return 0;
    }
};

}

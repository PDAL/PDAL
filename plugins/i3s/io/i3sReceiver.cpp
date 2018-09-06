//i3sRecevier.cpp
#include "i3sReceiver.hpp"

namespace pdal
{
  std::vector<lepcc::Point3D> decompress(
          bool elevation, bool rgb, std::vector<char>* compData, int nodeNum)
  {
    unsigned char* c = new unsigned char [compData->size()];
    
    /*Make buffer*/

    std::copy(compData->begin(), compData->end(), c);

    const unsigned char* compressed = c;
    int numPoint = 0;
    int nInfo = lepcc_getBlobInfoSize();
    lepcc_ContextHdl ctx(nullptr);
    ctx = lepcc_createContext(); 
    lepcc_blobType bt;
    lepcc::uint32 blobSize = 0;
    
    lepcc::ErrCode errCode = (lepcc::ErrCode)lepcc_getBlobInfo(ctx, compressed, nInfo, &bt, &blobSize); 
    
    int nBytes = (errCode == lepcc::ErrCode::Ok) ? (int)blobSize : -1; 

    lepcc::Byte vec;  
    lepcc_status stat;
    std::vector<lepcc::Point3D> decVec;
    lepcc::uint32 nPts = 0;
    const lepcc::Byte* pByte = &compressed[0]; 
     
    if(elevation) 
    {
        if (nBytes > 0)
        {
            stat = lepcc_getPointCount(ctx, pByte, nBytes, &nPts); 
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_getPointCount() failed. \n";            
                return decVec;
            }
            std::cout << "Node Number: " << nodeNum << std::endl;
            std::cout << "Point Count: " << nPts << std::endl;
            decVec.resize(nPts);
            stat = lepcc_decodeXYZ(
                    ctx, &pByte, nBytes, &nPts, (double*)(&decVec[0]));
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_decodeXYZ() failed. \n";
                return decVec;
            }
            return decVec;
        }
    }
    return decVec;
  }
}

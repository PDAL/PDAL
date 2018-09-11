//i3sRecevier.cpg
#include "i3sReceiver.hpp"

namespace pdal
{
    std::vector<lepcc::Point3D> decompressXYZ(
            std::vector<char>* compData, int nodeNum)
    {
        unsigned char* c = new unsigned char [compData->size()];
        
        std::copy(compData->begin(), compData->end(), c);

        const unsigned char* compressed = c;
        int nInfo = lepcc_getBlobInfoSize();
        lepcc_ContextHdl ctx(nullptr);
        ctx = lepcc_createContext(); 
        lepcc_blobType bt;
        lepcc::uint32 blobSize = 0;
        

        lepcc::Byte vec;  
        lepcc_status stat;
        std::vector<lepcc::Point3D> decVec;
        lepcc::uint32 xyzPts = 0;

        lepcc::ErrCode errCode = 
            (lepcc::ErrCode)lepcc_getBlobInfo(
                    ctx, compressed, nInfo, &bt, &blobSize); 

        const lepcc::Byte* pByte = &compressed[0]; 
        int nBytes = (errCode == lepcc::ErrCode::Ok) ? (int)blobSize : -1; 
        if (nBytes > 0)
        {
            stat = lepcc_getPointCount(ctx, pByte, nBytes, &xyzPts); 
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_getPointCount() failed. \n";            
                return decVec;//TODO throw error here
            }
            decVec.resize(xyzPts);
            stat = lepcc_decodeXYZ(
                    ctx, &pByte, nBytes, &xyzPts, (double*)(&decVec[0]));
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_decodeXYZ() failed. \n";
                return decVec;//TODO throw error here
            }
        }
        return decVec;
    }


    std::vector<lepcc::RGB_t> decompressRGB(
            std::vector<char>* compData, int nodeNum)
    {
        unsigned char* c = new unsigned char [compData->size()];
        
        std::copy(compData->begin(), compData->end(), c);

        const unsigned char* compressed = c;
        int nInfo = lepcc_getBlobInfoSize();
        lepcc_ContextHdl ctx(nullptr);
        ctx = lepcc_createContext(); 
        lepcc_blobType bt;
        lepcc::uint32 blobSize = 0;

        lepcc_status stat;
        std::vector<lepcc::RGB_t> rgbVec;

        lepcc::uint32 nPts = 0;
        lepcc::ErrCode errCode = 
            (lepcc::ErrCode)lepcc_getBlobInfo(
                    ctx, compressed, nInfo, &bt, &blobSize); 

        int nBytes = (errCode == lepcc::ErrCode::Ok) ? (int)blobSize : -1; 
        const lepcc::Byte* pByte = &compressed[0]; 
        if (nBytes > 0)
        {
            stat = lepcc_getRGBCount(ctx, pByte, nBytes, &nPts); 
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << stat << std::endl;
                std::cout << "lepcc_getPointRGB() failed. \n"; 
                //TODO throw error here
            }
            rgbVec.resize(nPts);
            stat = lepcc_decodeRGB(
                    ctx, &pByte, nBytes, &nPts, (lepcc::Byte*)(&rgbVec[0]));
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_decodergb() failed. \n";
                return rgbVec;//TODO throw error here
            }
        }
        return rgbVec;
    }


    std::vector<uint16_t> decompressIntensity(std::vector<char>* compData,int nodenum)
    {
        
        unsigned char* c = new unsigned char [compData->size()];
        
        std::copy(compData->begin(), compData->end(), c);

        const unsigned char* compressed = c;
        int nInfo = lepcc_getBlobInfoSize();
        lepcc_ContextHdl ctx(nullptr);
        ctx = lepcc_createContext(); 
        lepcc_blobType bt;
        lepcc::uint32 blobSize = 0;

        lepcc_status stat;
        lepcc::uint32 nPts = 0;
        lepcc::ErrCode errCode = 
            (lepcc::ErrCode)lepcc_getBlobInfo(
                    ctx, compressed, nInfo, &bt, &blobSize); 

        int nBytes = (errCode == lepcc::ErrCode::Ok) ? (int)blobSize : -1; 
        const lepcc::Byte* pByte = &compressed[0]; 
        std::vector<uint16_t> intVec;
        if (nBytes > 0)
        {
            stat = lepcc_getIntensityCount(ctx, pByte, nBytes, &nPts); 
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << stat << std::endl;
                std::cout << "lepcc_getPointRGB() failed. \n"; 
                //TODO throw error here
            }
            intVec.resize(nPts);
            stat = lepcc_decodeIntensity(
                    ctx, &pByte, nBytes, &nPts, (unsigned short*)(&intVec[0]));
            if(stat != (lepcc_status) lepcc::ErrCode::Ok)
            {
                std::cout << "lepcc_decodergb() failed. \n";
                return intVec;//TODO throw error here
            }
            
            return intVec;
        }
        return intVec;
    }
}

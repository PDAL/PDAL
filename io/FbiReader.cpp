/******************************************************************************
* Copyright (c) 2021, Antoine Lavenant, antoine.lavenant@ign.fr
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include "FbiReader.hpp"

#include <pdal/PointView.hpp>
#include <pdal/util/Extractor.hpp>

#include <map>

using namespace pdal::fbi;

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.fbi",
    "Bbi Reader",
    "http://pdal.io/stages/readers.fbi.html",
    { "bin" }
};

CREATE_STATIC_STAGE(FbiReader, s_info)

void readFbiHeader(FbiHdr* hdr, std::ifstream* ifFbi)
{
    ifFbi->read(reinterpret_cast<char *>(&hdr->Signature), sizeof(hdr->Signature));
    assert(std::string(hdr->Signature)=="FASTBIN");
    
    ifFbi->read(reinterpret_cast<char *>(&hdr->Version), sizeof(hdr->Version));
    ifFbi->read(reinterpret_cast<char *>(&hdr->HdrSize), sizeof(hdr->HdrSize));
    ifFbi->read(reinterpret_cast<char *>(&hdr->TimeType), sizeof(hdr->TimeType));
    ifFbi->read(reinterpret_cast<char *>(&hdr->Order), sizeof(hdr->Order));
    ifFbi->read(reinterpret_cast<char *>(&hdr->Reserved1), sizeof(hdr->Reserved1));
    ifFbi->read(reinterpret_cast<char *>(&hdr->VlrCnt), sizeof(hdr->VlrCnt));
    ifFbi->read(reinterpret_cast<char *>(&hdr->VlrSize), sizeof(hdr->VlrSize));
    ifFbi->read(reinterpret_cast<char *>(&hdr->RecSize), sizeof(hdr->RecSize));
    
    ifFbi->read(reinterpret_cast<char *>(&hdr->FastCnt), sizeof(hdr->FastCnt));
    ifFbi->read(reinterpret_cast<char *>(&hdr->RecCnt), sizeof(hdr->RecCnt));
    
    ifFbi->read(reinterpret_cast<char *>(&hdr->UnitsXyz), sizeof(hdr->UnitsXyz));
    assert(hdr->UnitsXyz!=0.);
    
    ifFbi->read(reinterpret_cast<char *>(&hdr->UnitsDistance), sizeof(hdr->UnitsDistance));
    ifFbi->read(reinterpret_cast<char *>(&hdr->OrgX), sizeof(hdr->OrgX));
    ifFbi->read(reinterpret_cast<char *>(&hdr->OrgY), sizeof(hdr->OrgY));
    ifFbi->read(reinterpret_cast<char *>(&hdr->OrgZ), sizeof(hdr->OrgZ));
    ifFbi->read(reinterpret_cast<char *>(&hdr->MinX), sizeof(hdr->MinX));
    ifFbi->read(reinterpret_cast<char *>(&hdr->MaxX), sizeof(hdr->MaxX));
    ifFbi->read(reinterpret_cast<char *>(&hdr->MinY), sizeof(hdr->MinY));
    ifFbi->read(reinterpret_cast<char *>(&hdr->MaxY), sizeof(hdr->MaxY));
    ifFbi->read(reinterpret_cast<char *>(&hdr->MinZ), sizeof(hdr->MinZ));
    ifFbi->read(reinterpret_cast<char *>(&hdr->MaxZ), sizeof(hdr->MaxZ));
    ifFbi->read(reinterpret_cast<char *>(&hdr->System), sizeof(hdr->System));
    ifFbi->read(reinterpret_cast<char *>(&hdr->Software), sizeof(hdr->Software));
    ifFbi->read(reinterpret_cast<char *>(&hdr->Reserved2), sizeof(hdr->Reserved2));
    
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsX), sizeof(hdr->BitsX));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsY), sizeof(hdr->BitsY));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsZ), sizeof(hdr->BitsZ));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsTime), sizeof(hdr->BitsTime));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsDistance), sizeof(hdr->BitsDistance));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsGroup), sizeof(hdr->BitsGroup));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsNormal), sizeof(hdr->BitsNormal));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsColor), sizeof(hdr->BitsColor));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsIntensity), sizeof(hdr->BitsIntensity));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsLine), sizeof(hdr->BitsLine));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsEchoLen), sizeof(hdr->BitsEchoLen));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsAmplitude), sizeof(hdr->BitsAmplitude));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsScanner), sizeof(hdr->BitsScanner));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsEcho), sizeof(hdr->BitsEcho));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsAngle), sizeof(hdr->BitsAngle));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsEchoNorm), sizeof(hdr->BitsEchoNorm));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsClass), sizeof(hdr->BitsClass));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsEchoPos), sizeof(hdr->BitsEchoPos));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsImage), sizeof(hdr->BitsImage));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsReflect), sizeof(hdr->BitsReflect));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsDeviation), sizeof(hdr->BitsDeviation));
    ifFbi->read(reinterpret_cast<char *>(&hdr->BitsReliab), sizeof(hdr->BitsReliab));
    ifFbi->read(reinterpret_cast<char *>(&hdr->Reserved5), sizeof(hdr->Reserved5));
    
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosVlr), sizeof(hdr->PosVlr));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosXyz), sizeof(hdr->PosXyz));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosTime), sizeof(hdr->PosTime));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosDistance), sizeof(hdr->PosDistance));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosGroup), sizeof(hdr->PosGroup));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosNormal), sizeof(hdr->PosNormal));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosColor), sizeof(hdr->PosColor));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosIntensity), sizeof(hdr->PosIntensity));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosLine), sizeof(hdr->PosLine));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosEchoLen), sizeof(hdr->PosEchoLen));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosAmplitude), sizeof(hdr->PosAmplitude));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosScanner), sizeof(hdr->PosScanner));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosEcho), sizeof(hdr->PosEcho));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosAngle), sizeof(hdr->PosAngle));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosEchoNorm), sizeof(hdr->PosEchoNorm));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosClass), sizeof(hdr->PosClass));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosRecord), sizeof(hdr->PosRecord));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosEchoPos), sizeof(hdr->PosEchoPos));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosImage), sizeof(hdr->PosImage));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosReflect), sizeof(hdr->PosReflect));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosDeviation), sizeof(hdr->PosDeviation));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosReliab), sizeof(hdr->PosReliab));
    ifFbi->read(reinterpret_cast<char *>(&hdr->PosImgNbr), sizeof(hdr->PosImgNbr));
    ifFbi->read(reinterpret_cast<char *>(&hdr->ImgNbrCnt), sizeof(hdr->ImgNbrCnt));
    ifFbi->read(reinterpret_cast<char *>(&hdr->Reserved6), sizeof(hdr->Reserved6));
}

FbiReader::FbiReader():
pdal::Reader(),
hdr(new FbiHdr())
{
}

std::string FbiReader::getName() const { return s_info.name; }

void FbiReader::initialize()
{
    std::ifstream ifFbi (m_filename, std::ios_base::binary);
    ifFbi.seekg(0);
    
    //read the hdr file
    readFbiHeader(hdr.get(), &ifFbi);
    hdr->dump(log());
    
    ifFbi.close();
}

void FbiReader::addArgs(ProgramArgs& args)
{
    //nothing for now
}

void FbiReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
    
    if (hdr->BitsEcho>0) layout->registerDim(Dimension::Id::ReturnNumber);
    if (hdr->BitsTime>0) layout->registerDim(Dimension::Id::EchoRange);
    if (hdr->BitsAngle>0) layout->registerDim(Dimension::Id::ScanAngleRank);
    if (hdr->BitsClass>0) layout->registerDim(Dimension::Id::Classification);
    if (hdr->BitsLine>0) layout->registerDim(Dimension::Id::PointSourceId);
    if (hdr->BitsIntensity>0) layout->registerDim(Dimension::Id::Intensity);
    if (hdr->BitsGroup>0) layout->registerDim(Dimension::Id::ClusterID);
    if (hdr->BitsScanner>0) layout->registerDim(Dimension::Id::UserData);
    if (hdr->BitsTime>0) layout->registerDim(Dimension::Id::OffsetTime);

    if (hdr->BitsColor>0)
    {
        layout->registerDim(Dimension::Id::Red);
        layout->registerDim(Dimension::Id::Green);
        layout->registerDim(Dimension::Id::Blue);
        if (hdr->BitsColor>24) layout->registerDim(Dimension::Id::Alpha);
    }
}

void FbiReader::ready(PointTableRef)
{
    ifFbi.reset(new std::ifstream(m_filename, std::ios_base::binary));
    ifFbi->seekg( hdr->HdrSize );
}

point_count_t FbiReader::read(PointViewPtr view, point_count_t count)
{
    ifFbi->seekg(hdr->PosXyz);
    double Mul = 1.0 / hdr->UnitsXyz;
    
    for (size_t i(0); i<hdr->FastCnt; i++)
    {
        UINT xr,yr,zr;
        ifFbi->read(reinterpret_cast<char *>(&xr), hdr->BitsX/8);
        ifFbi->read(reinterpret_cast<char *>(&yr), hdr->BitsY/8);
        ifFbi->read(reinterpret_cast<char *>(&zr), hdr->BitsZ/8);
        
        double X = static_cast<double>(xr*Mul + hdr->OrgX);
        double Y = static_cast<double>(yr*Mul + hdr->OrgY);
        double Z = static_cast<double>(zr*Mul + hdr->OrgZ);

        view->setField(Dimension::Id::X, i, X);
        view->setField(Dimension::Id::Y, i, Y);
        view->setField(Dimension::Id::Z, i, Z);
    }
    
    if (hdr->BitsTime>0)
    {
        ifFbi->seekg(hdr->PosTime);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            UINT64 timeGPS;
            ifFbi->read(reinterpret_cast<char *>(&timeGPS), hdr->BitsTime/8);
            view->setField(Dimension::Id::OffsetTime, i, uint32_t(timeGPS));
        }
    }
    
    if (hdr->BitsGroup>0)
    {
        ifFbi->seekg(hdr->PosGroup);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            UINT grpId;
            ifFbi->read(reinterpret_cast<char *>(&grpId), hdr->BitsGroup/8);
            view->setField(Dimension::Id::ClusterID, i , uint64_t(grpId));
        }
    }
    
    if (hdr->BitsColor>0)
    {
        ifFbi->seekg(hdr->PosColor);
        int rad ( hdr->BitsColor>24 ? 8*4 : 8*3);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            UINT blue, green, red;
            ifFbi->read(reinterpret_cast<char *>(&red), hdr->BitsColor/rad);
            ifFbi->read(reinterpret_cast<char *>(&green), hdr->BitsColor/rad);
            ifFbi->read(reinterpret_cast<char *>(&blue), hdr->BitsColor/rad);
            
            view->setField(Dimension::Id::Red, i, uint8_t(red));
            view->setField(Dimension::Id::Green, i, uint8_t(green));
            view->setField(Dimension::Id::Blue, i, uint8_t(blue));
            
            if (hdr->BitsColor>24)
            {
                UINT alpha;
                ifFbi->read(reinterpret_cast<char *>(&alpha), hdr->BitsColor/rad);
                view->setField(Dimension::Id::Alpha, i, uint8_t(alpha));
            }
        }
    }
    
    if (hdr->BitsIntensity>0)
    {
        ifFbi->seekg(hdr->PosIntensity, std::ios::beg);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            UINT intensity;
            ifFbi->read(reinterpret_cast<char *>(&intensity), hdr->BitsIntensity/8);
            view->setField(Dimension::Id::Intensity, i, uint16_t(intensity));
        }
    }
    
    if (hdr->BitsLine>0)
    {
        ifFbi->seekg(hdr->PosLine);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            UINT line;
            ifFbi->read(reinterpret_cast<char *>(&line), hdr->BitsLine/8);
            view->setField(Dimension::Id::PointSourceId, i, uint8_t(line));
        }
    }
    
    if (hdr->BitsEchoLen>0)
    {
        ifFbi->seekg(hdr->PosEchoLen);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            UINT echoLenght;
            ifFbi->read(reinterpret_cast<char *>(&echoLenght), hdr->BitsEchoLen/8);
            view->setField(Dimension::Id::ReturnNumber, i, uint8_t(echoLenght));
        }
    }
    
    if (hdr->BitsScanner>0)
    {
        ifFbi->seekg(hdr->PosScanner);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            BYTE userData;
            ifFbi->read(reinterpret_cast<char *>(&userData), hdr->BitsScanner/8);
            view->setField(Dimension::Id::UserData, i , uint8_t(userData));
        }
    }
    
    if (hdr->BitsEcho>0)
    {
        ifFbi->seekg(hdr->PosEcho);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            BYTE echo;
            ifFbi->read(reinterpret_cast<char *>(&echo), hdr->BitsEcho/8);
            view->setField(Dimension::Id::ReturnNumber, i , uint8_t(echo));
        }
    }
    
    PointRef point(*view, 0);
    
    if (hdr->BitsAngle>0)
    {
        ifFbi->seekg(hdr->PosAngle);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            BYTE angle;
            ifFbi->read(reinterpret_cast<char *>(&angle), hdr->BitsAngle/8);
            view->setField(Dimension::Id::ScanAngleRank, i , int8_t(angle));
        }
    }
    
    if (hdr->BitsClass>0)
    {
        ifFbi->seekg(hdr->PosClass);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            BYTE classif;
            ifFbi->read(reinterpret_cast<char *>(&classif), hdr->BitsClass/8);
            view->setField(Dimension::Id::Classification, i , int(classif));
        }
    }
        
    /*
     Other Fbi contents - Not found equivalent in pdal model for now
    
     if (hdr->BitsDistance>0)
    {
        ifFbi->seekg(hdr->PosDistance);
        for (size_t i(0); i<hdr->FastCnt; i++)
            ifFbi->read(reinterpret_cast<char *>(&vPt[i].distance), hdr->BitsDistance/8);
    }
     
     if (hdr->BitsImage>0)
     {
         ifFbi->seekg(hdr->PosImage);
         for (size_t i(0); i<hdr->FastCnt; i++)
             ifFbi->read(reinterpret_cast<char *>(&vPt[i].imNumber), hdr->BitsImage/8);
     }
     
     if (hdr->BitsNormal>0)
     {
         ifFbi->seekg(hdr->PosNormal);
         for (size_t i(0); i<hdr->FastCnt; i++)
             ifFbi->read(reinterpret_cast<char *>(&vPt[i].normVec), hdr->BitsNormal/8);
     }
     
     if (hdr->BitsAmplitude>0)
     {
         ifFbi->seekg(hdr->PosAmplitude);
         for (size_t i(0); i<hdr->FastCnt; i++)
             ifFbi->read(reinterpret_cast<char *>(&vPt[i].amplitude), hdr->BitsAmplitude/8);
     }
     
     if (hdr->BitsReflect>0)
     {
         ifFbi->seekg(hdr->PosReflect);
         for (size_t i(0); i<hdr->FastCnt; i++)
             ifFbi->read(reinterpret_cast<char *>(&vPt[i].reflectance), hdr->BitsReflect/8);
     }
     
     if (hdr->BitsDeviation>0)
     {
         ifFbi->seekg(hdr->PosDeviation);
         for (size_t i(0); i<hdr->FastCnt; i++)
             ifFbi->read(reinterpret_cast<char *>(&vPt[i].deviation), hdr->BitsDeviation/8);
     }
     
     if (hdr->BitsEchoNorm>0)
     {
         ifFbi->seekg(hdr->PosEchoNorm);
         for (size_t i(0); i<hdr->FastCnt; i++)
             ifFbi->read(reinterpret_cast<char *>(&vPt[i].echoNormality), hdr->BitsEchoNorm/8);
     }

     if (hdr->BitsEchoPos>0)
     {
         ifFbi->seekg(hdr->PosEchoPos);
         for (size_t i(0); i<hdr->FastCnt; i++)
             ifFbi->read(reinterpret_cast<char *>(&vPt[i].echoPos), hdr->BitsEchoPos/8);
     }
     
     if (hdr->BitsReliab>0)
     {
         ifFbi->seekg(hdr->PosReliab);
         for (size_t i(0); i<hdr->FastCnt; i++)
             ifFbi->read(reinterpret_cast<char *>(&vPt[i].reliability), hdr->BitsReliab/8);
     }
     */

    ifFbi->close();

    return hdr->FastCnt;
}

void FbiReader::done(PointTableRef)
{
    ifFbi->close();
    ifFbi.reset();
}

} // namespace pdal

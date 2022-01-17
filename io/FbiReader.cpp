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
#include <pdal/PDALUtils.hpp>

#include <map>

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

void readFbiHeader(fbi::FbiHdr* hdr, std::istream* m_istreamPtr)
{
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->Signature), sizeof(hdr->Signature));
    assert(std::string(hdr->Signature)=="FASTBIN");
    
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->Version), sizeof(hdr->Version));
    assert(hdr->Version==1);
    
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->HdrSize), sizeof(hdr->HdrSize));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->TimeType), sizeof(hdr->TimeType));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->Order), sizeof(hdr->Order));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->Reserved1), sizeof(hdr->Reserved1));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->VlrCnt), sizeof(hdr->VlrCnt));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->VlrSize), sizeof(hdr->VlrSize));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->RecSize), sizeof(hdr->RecSize));
    
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->FastCnt), sizeof(hdr->FastCnt));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->RecCnt), sizeof(hdr->RecCnt));
    
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->UnitsXyz), sizeof(hdr->UnitsXyz));
    assert(hdr->UnitsXyz > 0.);
    
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->UnitsDistance), sizeof(hdr->UnitsDistance));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->OrgX), sizeof(hdr->OrgX));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->OrgY), sizeof(hdr->OrgY));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->OrgZ), sizeof(hdr->OrgZ));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->MinX), sizeof(hdr->MinX));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->MaxX), sizeof(hdr->MaxX));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->MinY), sizeof(hdr->MinY));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->MaxY), sizeof(hdr->MaxY));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->MinZ), sizeof(hdr->MinZ));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->MaxZ), sizeof(hdr->MaxZ));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->System), sizeof(hdr->System));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->Software), sizeof(hdr->Software));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->Reserved2), sizeof(hdr->Reserved2));
    
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsX), sizeof(hdr->BitsX));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsY), sizeof(hdr->BitsY));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsZ), sizeof(hdr->BitsZ));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsTime), sizeof(hdr->BitsTime));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsDistance), sizeof(hdr->BitsDistance));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsGroup), sizeof(hdr->BitsGroup));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsNormal), sizeof(hdr->BitsNormal));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsColor), sizeof(hdr->BitsColor));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsIntensity), sizeof(hdr->BitsIntensity));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsLine), sizeof(hdr->BitsLine));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsEchoLen), sizeof(hdr->BitsEchoLen));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsAmplitude), sizeof(hdr->BitsAmplitude));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsScanner), sizeof(hdr->BitsScanner));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsEcho), sizeof(hdr->BitsEcho));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsAngle), sizeof(hdr->BitsAngle));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsEchoNorm), sizeof(hdr->BitsEchoNorm));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsClass), sizeof(hdr->BitsClass));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsEchoPos), sizeof(hdr->BitsEchoPos));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsImage), sizeof(hdr->BitsImage));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsReflect), sizeof(hdr->BitsReflect));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsDeviation), sizeof(hdr->BitsDeviation));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->BitsReliab), sizeof(hdr->BitsReliab));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->Reserved5), sizeof(hdr->Reserved5));
    
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosVlr), sizeof(hdr->PosVlr));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosXyz), sizeof(hdr->PosXyz));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosTime), sizeof(hdr->PosTime));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosDistance), sizeof(hdr->PosDistance));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosGroup), sizeof(hdr->PosGroup));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosNormal), sizeof(hdr->PosNormal));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosColor), sizeof(hdr->PosColor));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosIntensity), sizeof(hdr->PosIntensity));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosLine), sizeof(hdr->PosLine));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosEchoLen), sizeof(hdr->PosEchoLen));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosAmplitude), sizeof(hdr->PosAmplitude));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosScanner), sizeof(hdr->PosScanner));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosEcho), sizeof(hdr->PosEcho));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosAngle), sizeof(hdr->PosAngle));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosEchoNorm), sizeof(hdr->PosEchoNorm));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosClass), sizeof(hdr->PosClass));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosRecord), sizeof(hdr->PosRecord));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosEchoPos), sizeof(hdr->PosEchoPos));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosImage), sizeof(hdr->PosImage));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosReflect), sizeof(hdr->PosReflect));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosDeviation), sizeof(hdr->PosDeviation));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosReliab), sizeof(hdr->PosReliab));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->PosImgNbr), sizeof(hdr->PosImgNbr));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->ImgNbrCnt), sizeof(hdr->ImgNbrCnt));
    m_istreamPtr->read(reinterpret_cast<char *>(&hdr->Reserved6), sizeof(hdr->Reserved6));
}

FbiReader::FbiReader():
pdal::Reader(),
hdr(new fbi::FbiHdr())
{
}

std::string FbiReader::getName() const { return s_info.name; }

void FbiReader::initialize()
{
    m_istreamPtr = Utils::openFile(m_filename, true);
    
    m_istreamPtr->seekg(0);
    
    //read the hdr file
    readFbiHeader(hdr.get(), m_istreamPtr);
    hdr->dump(log());
    
    Utils::closeFile(m_istreamPtr);
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
    
    if (hdr->BitsEcho > 0) layout->registerDim(Dimension::Id::ReturnNumber);
    if (hdr->BitsTime > 0) layout->registerDim(Dimension::Id::EchoRange);
    
    //Fbi assumes only uint8 for ScanAngleRank
    if (hdr->BitsAngle > 0) layout->registerDim(Dimension::Id::ScanAngleRank, Dimension::Type::Unsigned8);
    
    if (hdr->BitsClass > 0) layout->registerDim(Dimension::Id::Classification);
    if (hdr->BitsLine > 0) layout->registerDim(Dimension::Id::PointSourceId);
    if (hdr->BitsIntensity > 0) layout->registerDim(Dimension::Id::Intensity);
    if (hdr->BitsGroup > 0) layout->registerDim(Dimension::Id::ClusterID);
    if (hdr->BitsScanner > 0) layout->registerDim(Dimension::Id::UserData);
    if (hdr->BitsTime > 0) layout->registerDim(Dimension::Id::OffsetTime);

    if (hdr->BitsColor > 0)
    {
        layout->registerDim(Dimension::Id::Red);
        layout->registerDim(Dimension::Id::Green);
        layout->registerDim(Dimension::Id::Blue);
        if (hdr->BitsColor > 24) layout->registerDim(Dimension::Id::Alpha);
    }
}

void FbiReader::ready(PointTableRef)
{
    m_istreamPtr = Utils::openFile(m_filename, true);
    m_istreamPtr->seekg( hdr->HdrSize );
}

point_count_t FbiReader::read(PointViewPtr view, point_count_t count)
{
    m_istreamPtr->seekg(hdr->PosXyz);
    double Mul = 1.0 / hdr->UnitsXyz;
    
    for (size_t i(0); i<hdr->FastCnt; i++)
    {
        fbi::UINT xr,yr,zr;
        m_istreamPtr->read(reinterpret_cast<char *>(&xr), hdr->BitsX/8);
        m_istreamPtr->read(reinterpret_cast<char *>(&yr), hdr->BitsY/8);
        m_istreamPtr->read(reinterpret_cast<char *>(&zr), hdr->BitsZ/8);
        
        double X = xr*Mul + hdr->OrgX;
        double Y = yr*Mul + hdr->OrgY;
        double Z = zr*Mul + hdr->OrgZ;

        view->setField(Dimension::Id::X, i, X);
        view->setField(Dimension::Id::Y, i, Y);
        view->setField(Dimension::Id::Z, i, Z);
    }
    
    if (hdr->BitsTime > 0)
    {
        m_istreamPtr->seekg(hdr->PosTime);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::UINT64 timeGPS;
            m_istreamPtr->read(reinterpret_cast<char *>(&timeGPS), hdr->BitsTime/8);
            view->setField(Dimension::Id::OffsetTime, i, uint32_t(timeGPS));
        }
    }
    
    if (hdr->BitsDistance > 0)
   {
       m_istreamPtr->seekg(hdr->PosDistance);
       for (size_t i(0); i<hdr->FastCnt; i++)
       {
           fbi::UINT distance;
           m_istreamPtr->read(reinterpret_cast<char *>(&distance), hdr->BitsDistance/8);
           view->setField(Dimension::Id::NNDistance, i, uint8_t(distance));
       }
   }
    
    if (hdr->BitsGroup > 0)
    {
        m_istreamPtr->seekg(hdr->PosGroup);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::UINT grpId;
            m_istreamPtr->read(reinterpret_cast<char *>(&grpId), hdr->BitsGroup/8);
            view->setField(Dimension::Id::ClusterID, i , uint64_t(grpId));
        }
    }
    
    if (hdr->BitsColor > 0)
    {
        m_istreamPtr->seekg(hdr->PosColor);
        int rad ( hdr->BitsColor > 24 ? 8*4 : 8*3);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::UINT blue, green, red;
            m_istreamPtr->read(reinterpret_cast<char *>(&red), hdr->BitsColor/rad);
            m_istreamPtr->read(reinterpret_cast<char *>(&green), hdr->BitsColor/rad);
            m_istreamPtr->read(reinterpret_cast<char *>(&blue), hdr->BitsColor/rad);
            
            view->setField(Dimension::Id::Red, i, uint8_t(red));
            view->setField(Dimension::Id::Green, i, uint8_t(green));
            view->setField(Dimension::Id::Blue, i, uint8_t(blue));
            
            if (hdr->BitsColor > 24)
            {
                fbi::UINT alpha;
                m_istreamPtr->read(reinterpret_cast<char *>(&alpha), hdr->BitsColor/rad);
                view->setField(Dimension::Id::Alpha, i, uint8_t(alpha));
            }
        }
    }
    
    if (hdr->BitsIntensity > 0)
    {
        m_istreamPtr->seekg(hdr->PosIntensity, std::ios::beg);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::UINT intensity;
            m_istreamPtr->read(reinterpret_cast<char *>(&intensity), hdr->BitsIntensity/8);
            view->setField(Dimension::Id::Intensity, i, uint16_t(intensity));
        }
    }
    
    if (hdr->BitsLine > 0)
    {
        m_istreamPtr->seekg(hdr->PosLine);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::UINT line;
            m_istreamPtr->read(reinterpret_cast<char *>(&line), hdr->BitsLine/8);
            view->setField(Dimension::Id::PointSourceId, i, uint8_t(line));
        }
    }
    
    if (hdr->BitsEchoLen > 0)
    {
        m_istreamPtr->seekg(hdr->PosEchoLen);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::UINT echoLenght;
            m_istreamPtr->read(reinterpret_cast<char *>(&echoLenght), hdr->BitsEchoLen/8);
            view->setField(Dimension::Id::ReturnNumber, i, uint8_t(echoLenght));
        }
    }
    
    if (hdr->BitsScanner > 0)
    {
        m_istreamPtr->seekg(hdr->PosScanner);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::BYTE userData;
            m_istreamPtr->read(reinterpret_cast<char *>(&userData), hdr->BitsScanner/8);
            view->setField(Dimension::Id::UserData, i , uint8_t(userData));
        }
    }
    
    if (hdr->BitsEcho > 0)
    {
        m_istreamPtr->seekg(hdr->PosEcho);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::BYTE echo;
            m_istreamPtr->read(reinterpret_cast<char *>(&echo), hdr->BitsEcho/8);
            view->setField(Dimension::Id::ReturnNumber, i , uint8_t(echo));
        }
    }
    
    if (hdr->BitsAngle > 0)
    {
        m_istreamPtr->seekg(hdr->PosAngle);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::BYTE angle;
            m_istreamPtr->read(reinterpret_cast<char *>(&angle), hdr->BitsAngle/8);
            view->setField(Dimension::Id::ScanAngleRank, i , int8_t(angle));
        }
    }
    
    if (hdr->BitsClass > 0)
    {
        m_istreamPtr->seekg(hdr->PosClass);
        for (size_t i(0); i<hdr->FastCnt; i++)
        {
            fbi::BYTE classif;
            m_istreamPtr->read(reinterpret_cast<char *>(&classif), hdr->BitsClass/8);
            view->setField(Dimension::Id::Classification, i , int(classif));
        }
    }
        
    /*
     Other Fbi contents - Not found equivalent in pdal model for now
    
     
     if (hdr->BitsImage > 0)
     {
         m_istreamPtr->seekg(hdr->PosImage);
         for (size_t i(0); i<hdr->FastCnt; i++)
             m_istreamPtr->read(reinterpret_cast<char *>(&vPt[i].imNumber), hdr->BitsImage/8);
     }
     
     if (hdr->BitsNormal > 0)
     {
         m_istreamPtr->seekg(hdr->PosNormal);
         for (size_t i(0); i<hdr->FastCnt; i++)
             m_istreamPtr->read(reinterpret_cast<char *>(&vPt[i].normVec), hdr->BitsNormal/8);
     }
     
     if (hdr->BitsAmplitude > 0)
     {
         m_istreamPtr->seekg(hdr->PosAmplitude);
         for (size_t i(0); i<hdr->FastCnt; i++)
             m_istreamPtr->read(reinterpret_cast<char *>(&vPt[i].amplitude), hdr->BitsAmplitude/8);
     }
     
     if (hdr->BitsReflect > 0)
     {
         m_istreamPtr->seekg(hdr->PosReflect);
         for (size_t i(0); i<hdr->FastCnt; i++)
             m_istreamPtr->read(reinterpret_cast<char *>(&vPt[i].reflectance), hdr->BitsReflect/8);
     }
     
     if (hdr->BitsDeviation > 0)
     {
         m_istreamPtr->seekg(hdr->PosDeviation);
         for (size_t i(0); i<hdr->FastCnt; i++)
             m_istreamPtr->read(reinterpret_cast<char *>(&vPt[i].deviation), hdr->BitsDeviation/8);
     }
     
     if (hdr->BitsEchoNorm > 0)
     {
         m_istreamPtr->seekg(hdr->PosEchoNorm);
         for (size_t i(0); i<hdr->FastCnt; i++)
             m_istreamPtr->read(reinterpret_cast<char *>(&vPt[i].echoNormality), hdr->BitsEchoNorm/8);
     }

     if (hdr->BitsEchoPos > 0)
     {
         m_istreamPtr->seekg(hdr->PosEchoPos);
         for (size_t i(0); i<hdr->FastCnt; i++)
             m_istreamPtr->read(reinterpret_cast<char *>(&vPt[i].echoPos), hdr->BitsEchoPos/8);
     }
     
     if (hdr->BitsReliab > 0)
     {
         m_istreamPtr->seekg(hdr->PosReliab);
         for (size_t i(0); i<hdr->FastCnt; i++)
             m_istreamPtr->read(reinterpret_cast<char *>(&vPt[i].reliability), hdr->BitsReliab/8);
     }
     */

    return hdr->FastCnt;
}

void FbiReader::done(PointTableRef)
{
    Utils::closeFile(m_istreamPtr);
}

} // namespace pdal

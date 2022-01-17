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

#include "FbiWriter.hpp"

#include <pdal/PDALUtils.hpp>

namespace pdal {

namespace
{
    const StaticPluginInfo s_info
    {
        "writers.fbi",
        "FBI Writer",
        "http://pdal.io/stages/writers.fbi.html"
    };
}

CREATE_STATIC_STAGE(FbiWriter, s_info)

FbiWriter::FbiWriter():
hdr(new fbi::FbiHdr())
{}

FbiWriter::~FbiWriter()
{}

std::string FbiWriter::getName() const { return s_info.name; }

void buildHdrFromPoints(fbi::FbiHdr& hdr, const PointViewPtr view)
{
    hdr.Version = 1;
    hdr.HdrSize = 1808; // by construction
    hdr.TimeType = 1; // standard time by default
    hdr.Order = 0; // order is not know
    hdr.Reserved1 = 0; // Not use for now
    hdr.VlrCnt = 0; // no more varaibles
    hdr.VlrSize = 0; // idem
    hdr.RecSize = 0; // ?????
     
    hdr.FastCnt = view->size();
    hdr.RecCnt = 0; // no additional points
    
    BOX3D box;
    view->calculateBounds(box);
    
    hdr.MinX = box.minx;
    hdr.MaxX = box.maxx;
    hdr.MinY = box.miny;
    hdr.MaxY = box.maxy;
    hdr.MinZ = box.minz;
    hdr.MaxZ = box.maxz;

    // By default for now
    hdr.UnitsXyz = 100;
    hdr.UnitsDistance = 0;
    
    hdr.OrgX = std::abs(hdr.MinX)-1;
    hdr.OrgY = std::abs(hdr.MinY)-1;
    hdr.OrgZ = std::abs(hdr.MinZ)-1;
     
    hdr.BitsX = 32;
    hdr.BitsY = 32;
    hdr.BitsZ = 32;
    
    hdr.BitsTime = ( view->hasDim(Dimension::Id::OffsetTime) ? 64 : 0 );
    hdr.BitsGroup = ( view->hasDim(Dimension::Id::ClusterID) ? 32 : 0 );
    hdr.BitsIntensity = ( view->hasDim(Dimension::Id::Intensity) ? 16 : 0 );
    hdr.BitsScanner = ( view->hasDim(Dimension::Id::UserData) ? 8 : 0 );
    hdr.BitsEcho = ( view->hasDim(Dimension::Id::ReturnNumber) ? 8 : 0 );
    hdr.BitsAngle = ( view->hasDim(Dimension::Id::ScanAngleRank) ? 8 : 0 );
    hdr.BitsClass = ( view->hasDim(Dimension::Id::Classification) ? 8 : 0 );
    hdr.BitsLine = ( view->hasDim(Dimension::Id::PointSourceId) ? 16 : 0 );
    hdr.BitsEchoLen = ( view->hasDim(Dimension::Id::ReturnNumber) ? 16 : 0 );
    
    hdr.BitsColor = 0;
    if ( view->hasDim(Dimension::Id::Red) && view->hasDim(Dimension::Id::Blue) && view->hasDim(Dimension::Id::Green) ) hdr.BitsColor = 24;
    if ( view->hasDim(Dimension::Id::Alpha) ) hdr.BitsColor += 24;

    //Not found quivalent in pdal for now
    hdr.BitsDistance = 0;
    hdr.BitsNormal = 0;
    hdr.BitsAmplitude = 0;
    hdr.BitsEchoNorm = 0;
    hdr.BitsEchoPos = 0;
    hdr.BitsReflect = 0;
    hdr.BitsDeviation = 0;
    hdr.BitsReliab = 0;
    hdr.BitsImage = 0;

    hdr.Reserved5 = 0;
    
    hdr.PosVlr = 0 ; // je ne sais pas ce que c'est
    hdr.PosXyz = hdr.HdrSize;
    hdr.PosTime = hdr.PosXyz + 3*view->size()*sizeof(fbi::UINT);
    hdr.PosDistance = hdr.PosTime + view->size()*hdr.BitsTime/8;
    hdr.PosGroup = hdr.PosDistance + view->size()*hdr.BitsDistance/8;
    hdr.PosImage = hdr.PosGroup + view->size()*hdr.BitsGroup/8;
    hdr.PosNormal = hdr.PosImage + view->size()*hdr.BitsImage/8;
    hdr.PosColor = hdr.PosNormal + view->size()*hdr.BitsNormal/8;
    hdr.PosIntensity = hdr.PosColor + view->size()*hdr.BitsColor/8;
    hdr.PosLine = hdr.PosIntensity + view->size()*hdr.BitsIntensity/8;
    hdr.PosEchoLen = hdr.PosLine + view->size()*hdr.BitsLine/8;
    hdr.PosAmplitude = hdr.PosEchoLen + view->size()*hdr.BitsEchoLen/8;
    hdr.PosScanner = hdr.PosAmplitude + view->size()*hdr.BitsAmplitude/8;
    hdr.PosEcho = hdr.PosScanner + view->size()*hdr.BitsScanner/8;
    hdr.PosAngle = hdr.PosEcho + view->size()*hdr.BitsEcho/8;
    hdr.PosEchoNorm = hdr.PosAngle + view->size()*hdr.BitsAngle/8;
    hdr.PosClass = hdr.PosEchoNorm + view->size()*hdr.BitsEchoNorm/8;
    hdr.PosEchoPos = hdr.PosClass + view->size()*hdr.BitsClass/8;
    hdr.PosReflect = hdr.PosEchoPos + view->size()*hdr.BitsEchoPos/8;
    hdr.PosDeviation = hdr.PosReflect + view->size()*hdr.BitsReflect/8;
    hdr.PosReliab = hdr.PosDeviation + view->size()*hdr.BitsDeviation/8;
    hdr.PosImgNbr = hdr.PosReliab + view->size()*hdr.BitsReliab/8;
    hdr.PosRecord = hdr.PosImgNbr + view->size()*hdr.BitsImage/8;

    hdr.ImgNbrCnt = 0; //  I don't know what to put here for now...
    strcpy(hdr.Reserved6,""); // Not use for now
}

void writeFbiHeader(const fbi::FbiHdr& hdr, std::ostream* ofFBI)
{
    ofFBI->write(hdr.Signature, sizeof(hdr.Signature));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.Version), sizeof(hdr.Version));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.HdrSize), sizeof(hdr.HdrSize));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.TimeType), sizeof(hdr.TimeType));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.Order), sizeof(hdr.Order));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.Reserved1), sizeof(hdr.Reserved1));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.VlrCnt), sizeof(hdr.VlrCnt));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.VlrSize), sizeof(hdr.VlrSize));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.RecSize), sizeof(hdr.RecSize));
    
    ofFBI->write(reinterpret_cast<const char *>(&hdr.FastCnt), sizeof(hdr.FastCnt));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.RecCnt), sizeof(hdr.RecCnt));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.UnitsXyz), sizeof(hdr.UnitsXyz));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.UnitsDistance), sizeof(hdr.UnitsDistance));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.OrgX), sizeof(hdr.OrgX));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.OrgY), sizeof(hdr.OrgY));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.OrgZ), sizeof(hdr.OrgZ));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.MinX), sizeof(hdr.MinX));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.MaxX), sizeof(hdr.MaxX));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.MinY), sizeof(hdr.MinY));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.MaxY), sizeof(hdr.MaxY));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.MinZ), sizeof(hdr.MinZ));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.MaxZ), sizeof(hdr.MaxZ));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.System), sizeof(hdr.System));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.Software), sizeof(hdr.Software));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.Reserved2), sizeof(hdr.Reserved2));
    
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsX), sizeof(hdr.BitsX));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsY), sizeof(hdr.BitsY));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsZ), sizeof(hdr.BitsZ));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsTime), sizeof(hdr.BitsTime));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsDistance), sizeof(hdr.BitsDistance));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsGroup), sizeof(hdr.BitsGroup));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsNormal), sizeof(hdr.BitsNormal));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsColor), sizeof(hdr.BitsColor));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsIntensity), sizeof(hdr.BitsIntensity));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsLine), sizeof(hdr.BitsLine));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsEchoLen), sizeof(hdr.BitsEchoLen));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsAmplitude), sizeof(hdr.BitsAmplitude));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsScanner), sizeof(hdr.BitsScanner));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsEcho), sizeof(hdr.BitsEcho));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsAngle), sizeof(hdr.BitsAngle));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsEchoNorm), sizeof(hdr.BitsEchoNorm));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsClass), sizeof(hdr.BitsClass));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsEchoPos), sizeof(hdr.BitsEchoPos));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsImage), sizeof(hdr.BitsImage));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsReflect), sizeof(hdr.BitsReflect));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsDeviation), sizeof(hdr.BitsDeviation));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.BitsReliab), sizeof(hdr.BitsReliab));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.Reserved5), sizeof(hdr.Reserved5));
    
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosVlr), sizeof(hdr.PosVlr));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosXyz), sizeof(hdr.PosXyz));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosTime), sizeof(hdr.PosTime));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosDistance), sizeof(hdr.PosDistance));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosGroup), sizeof(hdr.PosGroup));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosNormal), sizeof(hdr.PosNormal));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosColor), sizeof(hdr.PosColor));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosIntensity), sizeof(hdr.PosIntensity));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosLine), sizeof(hdr.PosLine));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosEchoLen), sizeof(hdr.PosEchoLen));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosAmplitude), sizeof(hdr.PosAmplitude));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosScanner), sizeof(hdr.PosScanner));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosEcho), sizeof(hdr.PosEcho));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosAngle), sizeof(hdr.PosAngle));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosEchoNorm), sizeof(hdr.PosEchoNorm));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosClass), sizeof(hdr.PosClass));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosRecord), sizeof(hdr.PosRecord));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosEchoPos), sizeof(hdr.PosEchoPos));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosImage), sizeof(hdr.PosImage));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosReflect), sizeof(hdr.PosReflect));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosDeviation), sizeof(hdr.PosDeviation));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosReliab), sizeof(hdr.PosReliab));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.PosImgNbr), sizeof(hdr.PosImgNbr));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.ImgNbrCnt), sizeof(hdr.ImgNbrCnt));
    ofFBI->write(reinterpret_cast<const char *>(&hdr.Reserved6), sizeof(hdr.Reserved6));
}

void FbiWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
}

void FbiWriter::write(const PointViewPtr view)
{
    buildHdrFromPoints(*hdr, view);
    
    PointRef point(*view, 0);
    
    std::ostream* ofFBI = Utils::createFile(m_filename, true);
        
    writeFbiHeader(*hdr.get(), ofFBI);
    
    assert(hdr->UnitsXyz > 0);
    double Mul = 1.0 / hdr->UnitsXyz;
    
    for (PointId i = 0; i < view->size(); ++i)
    {
        point.setPointId(i);
        double X = point.getFieldAs<double>(Dimension::Id::X);
        double Y = point.getFieldAs<double>(Dimension::Id::Y);
        double Z = point.getFieldAs<double>(Dimension::Id::Z);

        fbi::UINT xr = (X - hdr->OrgX)*Mul;
        fbi::UINT yr = (Y - hdr->OrgY)*Mul;
        fbi::UINT zr = (Z - hdr->OrgZ)*Mul;

        ofFBI->write(reinterpret_cast<const char *>(&xr), sizeof(hdr->BitsX));
        ofFBI->write(reinterpret_cast<const char *>(&yr), sizeof(hdr->BitsY));
        ofFBI->write(reinterpret_cast<const char *>(&zr), sizeof(hdr->BitsZ));
    }
    
    if (hdr->BitsTime > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint64_t time = point.getFieldAs<uint64_t>(Dimension::Id::OffsetTime);
            ofFBI->write(reinterpret_cast<const char *>(&time), hdr->BitsTime/8);
        }
    }
    
    if (hdr->BitsDistance > 0)
    {
        //nothing to do for now
    }

    if (hdr->BitsGroup > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint64_t cluster = point.getFieldAs<uint64_t>(Dimension::Id::ClusterID);
            ofFBI->write(reinterpret_cast<const char *>(&cluster), hdr->BitsGroup/8);
        }
    }
    
    if (hdr->BitsImage > 0)
    {
        //nothing to do for now
    }
    
    if (hdr->BitsNormal > 0)
    {
        //nothing to do for now
    }
    
    if (hdr->BitsColor > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            
            uint16_t blue = point.getFieldAs<uint16_t>(Dimension::Id::Blue);
            uint16_t green = point.getFieldAs<uint16_t>(Dimension::Id::Green);
            uint16_t red = point.getFieldAs<uint16_t>(Dimension::Id::Red);
            ofFBI->write(reinterpret_cast<const char *>(&blue), 2);
            ofFBI->write(reinterpret_cast<const char *>(&green), 2);
            ofFBI->write(reinterpret_cast<const char *>(&red), 2);
            
            if (hdr->BitsColor > 24)
            {
                uint16_t alpha = point.getFieldAs<uint16_t>(Dimension::Id::Alpha);
                ofFBI->write(reinterpret_cast<const char *>(&alpha), 2);
            }
        }
    }
    
    if (hdr->BitsIntensity > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint16_t intensity = point.getFieldAs<uint16_t>(Dimension::Id::Intensity);
            ofFBI->write(reinterpret_cast<const char *>(&intensity), hdr->BitsIntensity/8);
        }
    }
    
    if (hdr->BitsLine > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint16_t line = point.getFieldAs<uint16_t>(Dimension::Id::PointSourceId);
            ofFBI->write(reinterpret_cast<const char *>(&line), hdr->BitsLine/8);
        }
    }
    
    if (hdr->BitsEchoLen > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint8_t echoLenght = point.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber);
            ofFBI->write(reinterpret_cast<const char *>(&echoLenght), hdr->BitsEchoLen/8);
        }
    }

    if (hdr->BitsAmplitude > 0)
    {
        //nothing to do for now
    }
    
    if (hdr->BitsReflect > 0)
    {
        //nothing to do for now
    }
    
    if (hdr->BitsDeviation > 0)
    {
        //nothing to do for now
    }
    
    if (hdr->BitsScanner > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint8_t scanNbr = point.getFieldAs<uint8_t>(Dimension::Id::UserData);
            ofFBI->write(reinterpret_cast<const char *>(&scanNbr), hdr->BitsScanner/8);
        }
    }
    
    if (hdr->BitsEcho > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint8_t echo = point.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber);
            ofFBI->write(reinterpret_cast<const char *>(&echo), hdr->BitsEcho/8);
        }
    }
    
    if (hdr->BitsAngle > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            int8_t angle = point.getFieldAs<int8_t>(Dimension::Id::ScanAngleRank);
            ofFBI->write(reinterpret_cast<const char *>(&angle), hdr->BitsAngle/8);
        }
    }
    
    if (hdr->BitsEchoNorm > 0)
    {
        //nothing to do for now
    }

    if (hdr->BitsEchoPos > 0)
    {
        //nothing to do for now
    }
    
    if (hdr->BitsReliab > 0)
    {
        //nothing to do for now
    }
    
    if (hdr->BitsClass > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint8_t classif = point.getFieldAs<uint8_t>(Dimension::Id::Classification);
            ofFBI->write(reinterpret_cast<const char *>(&classif), hdr->BitsClass/8);
        }
    }

    Utils::closeFile(ofFBI);
}

}

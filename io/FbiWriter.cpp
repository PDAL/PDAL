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
#include "FbiHeader.hpp"

#include <pdal/PDALUtils.hpp>

namespace pdal {

namespace
{
    const StaticPluginInfo s_info
    {
        "writers.fbi",
        "FBI Writer",
        "http://pdal.org/stages/writers.fbi.html"
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

    // no additional values in this fbi export (for manage by terrascan for now)
    hdr.VlrCnt = 0;
    hdr.VlrSize = 0;
    hdr.RecSize = 0;

    hdr.FastCnt = view->size();
    hdr.RecCnt = 0; // no additional points in this fbi export

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

    // Fbi only accept 8 bits ScanAngleRank
    hdr.BitsAngle = ( view->hasDim(Dimension::Id::ScanAngleRank)
                     && view->dimType(Dimension::Id::ScanAngleRank)== Dimension::Type::Signed8 ? 8 : 0 );

    hdr.BitsClass = ( view->hasDim(Dimension::Id::Classification) ? 8 : 0 );
    hdr.BitsLine = ( view->hasDim(Dimension::Id::PointSourceId) ? 16 : 0 );
    hdr.BitsEchoLen = ( view->hasDim(Dimension::Id::ReturnNumber) ? 16 : 0 );

    hdr.BitsColor = 0; int NbCanal (0);
    if ( view->hasDim(Dimension::Id::Red) && view->hasDim(Dimension::Id::Blue) && view->hasDim(Dimension::Id::Green) )
    {
        hdr.BitsColor = 8*view->dimSize(Dimension::Id::Red);
        assert ( hdr.BitsColor == 8*view->dimSize(Dimension::Id::Blue) );
        assert ( hdr.BitsColor == 8*view->dimSize(Dimension::Id::Green) );
        NbCanal = 3;
    }
    if ( view->hasDim(Dimension::Id::Infrared) )
    {
        if (hdr.BitsColor>0)
            assert ( hdr.BitsColor == 8*view->dimSize(Dimension::Id::Infrared) );
        else
            hdr.BitsColor = 8*view->dimSize(Dimension::Id::Infrared);
        NbCanal +=1;
    }

    hdr.BitsDistance = ( view->hasDim(Dimension::Id::NNDistance) ? 32 : 0 );
    hdr.BitsAmplitude = ( view->hasDim(Dimension::Id::Amplitude) ? 16 : 0 );
    hdr.BitsEchoNorm = ( view->hasDim(Dimension::Id::EchoNorm) ? 8 : 0 );
    hdr.BitsEchoPos = ( view->hasDim(Dimension::Id::EchoPos) ? 16 : 0 );
    hdr.BitsReflect = ( view->hasDim(Dimension::Id::Reflectance) ? 16 : 0 );
    hdr.BitsDeviation = ( view->hasDim(Dimension::Id::Deviation) ? 16 : 0 );
    hdr.BitsReliab = ( view->hasDim(Dimension::Id::Reliability) ? 8 : 0 );
    hdr.BitsNormal = ( view->hasDim(Dimension::Id::NormalX) ? 32 : 0 );

    // why not 32 ?
    hdr.BitsImage = ( view->hasDim(Dimension::Id::Image) ? 16 : 0 );

    hdr.Reserved5 = 0;

    hdr.PosVlr = 0 ; // not use in fbi files for now
    hdr.PosXyz = hdr.HdrSize;
    hdr.PosTime = hdr.PosXyz + 3*view->size()*sizeof(fbi::UINT);
    hdr.PosDistance = hdr.PosTime + view->size()*hdr.BitsTime/8;
    hdr.PosGroup = hdr.PosDistance + view->size()*hdr.BitsDistance/8;
    hdr.PosNormal =  hdr.PosGroup + view->size()*hdr.BitsGroup/8;
    hdr.PosColor = hdr.PosNormal + view->size()*hdr.BitsNormal/8;
    hdr.PosIntensity = hdr.PosColor + NbCanal*view->size()*hdr.BitsColor/8;
    hdr.PosLine = hdr.PosIntensity + view->size()*hdr.BitsIntensity/8;
    hdr.PosEchoLen = hdr.PosLine + view->size()*hdr.BitsLine/8;
    hdr.PosAmplitude = hdr.PosEchoLen + view->size()*hdr.BitsEchoLen/8;
    hdr.PosScanner = hdr.PosAmplitude + view->size()*hdr.BitsAmplitude/8;
    hdr.PosEcho = hdr.PosScanner + view->size()*hdr.BitsScanner/8;
    hdr.PosAngle = hdr.PosEcho + view->size()*hdr.BitsEcho/8;
    hdr.PosEchoNorm = hdr.PosAngle + view->size()*hdr.BitsAngle/8;
    hdr.PosClass = hdr.PosEchoNorm + view->size()*hdr.BitsEchoNorm/8;
    hdr.PosEchoPos = hdr.PosClass + view->size()*hdr.BitsClass/8;
    hdr.PosImage = hdr.PosEchoPos + view->size()*hdr.BitsEchoPos/8;
    hdr.PosReflect = hdr.PosImage + view->size()*hdr.BitsImage/8;
    hdr.PosDeviation = hdr.PosReflect + view->size()*hdr.BitsReflect/8;
    hdr.PosReliab = hdr.PosDeviation + view->size()*hdr.BitsDeviation/8;
    hdr.PosImgNbr = hdr.PosReliab + view->size()*hdr.BitsReliab/8;
    hdr.PosRecord = hdr.PosImgNbr + view->size()*hdr.PosImgNbr/8;

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

void NrmVecSet(fbi::NrmVec *Vp, int Dim, double X, double Y, double Z)
{
    double Hml = 32767.0 / fbi::hc_2pi ;
    double Vml = 32767.0 / fbi::hc_pi ;
    double Van = asin(Z);
    int Vvl = (int) std::floor( Vml * (Van + fbi::hc_piover2)) ;
    Vvl = (std::max)( Vvl, 0) ;
    Vvl = (std::min)( Vvl, 32767) ;
    Vp->Dim     = Dim ;
    Vp->VertAng = Vvl ;
    Vp->HorzAng = 0;
    if ((X) || (Y)) {
        double Han = atan2( Y, X) ;
        if (Han < 0.0)
            Han += fbi::hc_2pi ;
        int Hvl = (int)std::floor( Hml * Han) ;
        Hvl = (std::max)( Hvl, 0) ;
        Hvl = (std::min)( Hvl, 32767) ;
        Vp->HorzAng = Hvl ;
    }
}

void FbiWriter::write(const PointViewPtr view)
{
    buildHdrFromPoints(*hdr, view);

    PointRef point(*view, 0);

    std::ostream* ofFBI = Utils::createFile(filename(), true);

    writeFbiHeader(*hdr.get(), ofFBI);

    assert(hdr->UnitsXyz > 0);
    double Mul = 1.0 / hdr->UnitsXyz;

    for (PointId i = 0; i < view->size(); ++i)
    {
        point.setPointId(i);
        double X = point.getFieldAs<double>(Dimension::Id::X);
        double Y = point.getFieldAs<double>(Dimension::Id::Y);
        double Z = point.getFieldAs<double>(Dimension::Id::Z);

        fbi::UINT xr = (fbi::UINT) ((X - hdr->OrgX)*Mul);
        fbi::UINT yr = (fbi::UINT) ((Y - hdr->OrgY)*Mul);
        fbi::UINT zr = (fbi::UINT) ((Z - hdr->OrgZ)*Mul);

        ofFBI->write(reinterpret_cast<const char *>(&xr), hdr->BitsX/8);
        ofFBI->write(reinterpret_cast<const char *>(&yr), hdr->BitsY/8);
        ofFBI->write(reinterpret_cast<const char *>(&zr), hdr->BitsZ/8);
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
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint32_t distance = point.getFieldAs<uint32_t>(Dimension::Id::NNDistance);
            ofFBI->write(reinterpret_cast<const char *>(&distance), hdr->BitsDistance/8);
        }
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

    if (hdr->BitsNormal > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint8_t dim = (uint8_t) point.getFieldAs<double>(Dimension::Id::Dimension);
            double norm_x = point.getFieldAs<double>(Dimension::Id::NormalX);
            double norm_y = point.getFieldAs<double>(Dimension::Id::NormalY);
            double norm_z = point.getFieldAs<double>(Dimension::Id::NormalZ);
            fbi::NrmVec normVec;
            NrmVecSet(&normVec, dim, norm_x, norm_y, norm_z);
            ofFBI->write(reinterpret_cast<const char *>(&normVec), hdr->BitsNormal/8);
        }

    }

    if (hdr->BitsColor > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);

            uint16_t blue = point.getFieldAs<uint16_t>(Dimension::Id::Blue);
            uint16_t green = point.getFieldAs<uint16_t>(Dimension::Id::Green);
            uint16_t red = point.getFieldAs<uint16_t>(Dimension::Id::Red);

            ofFBI->write(reinterpret_cast<const char *>(&blue), hdr->BitsColor/8);
            ofFBI->write(reinterpret_cast<const char *>(&green), hdr->BitsColor/8);
            ofFBI->write(reinterpret_cast<const char *>(&red), hdr->BitsColor/8);

            if (view->hasDim(Dimension::Id::Infrared))
            {
                uint16_t infra = point.getFieldAs<uint16_t>(Dimension::Id::Infrared);
                ofFBI->write(reinterpret_cast<const char *>(&infra), hdr->BitsColor/8);
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
            uint8_t echoLen = point.getFieldAs<uint8_t>(Dimension::Id::PulseWidth);
            ofFBI->write(reinterpret_cast<const char *>(&echoLen), hdr->BitsEchoLen/8);
        }
    }

    if (hdr->BitsAmplitude > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint16_t amplitude = point.getFieldAs<uint16_t>(Dimension::Id::Amplitude);
            ofFBI->write(reinterpret_cast<const char *>(&amplitude), hdr->BitsAmplitude/8);
        }
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
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint8_t echoNorm = point.getFieldAs<uint8_t>(Dimension::Id::EchoNorm);
            ofFBI->write(reinterpret_cast<const char *>(&echoNorm), hdr->BitsEchoNorm/8);
        }
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

    if (hdr->BitsEchoPos > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint16_t echoPos = point.getFieldAs<uint16_t>(Dimension::Id::EchoPos);
            ofFBI->write(reinterpret_cast<const char *>(&echoPos), hdr->BitsEchoPos/8);
        }
    }

    if (hdr->BitsImage > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint16_t image = point.getFieldAs<uint16_t>(Dimension::Id::Image);
            ofFBI->write(reinterpret_cast<const char *>(&image), hdr->BitsImage/8);
        }
    }

    if (hdr->BitsReflect > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint16_t reflectance = point.getFieldAs<uint16_t>(Dimension::Id::Reflectance);
            ofFBI->write(reinterpret_cast<const char *>(&reflectance), hdr->BitsReflect/8);
        }
    }

    if (hdr->BitsDeviation > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint16_t deviation = point.getFieldAs<uint16_t>(Dimension::Id::Deviation);
            ofFBI->write(reinterpret_cast<const char *>(&deviation), hdr->BitsDeviation/8);
        }
    }

    if (hdr->BitsReliab > 0)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            point.setPointId(i);
            uint8_t reliability = point.getFieldAs<uint8_t>(Dimension::Id::Reliability);
            ofFBI->write(reinterpret_cast<const char *>(&reliability), hdr->BitsReliab/8);
        }
    }

    if (hdr->ImgNbrCnt > 0)
    {

    }

    Utils::closeFile(ofFBI);
}

}

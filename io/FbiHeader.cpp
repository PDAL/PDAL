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

#include "FbiHeader.hpp"

namespace pdal
{

namespace fbi
{

void FbiHdr::dump(const LogPtr& log)
{
    log->get(LogLevel::Debug) << "Fbi header : Signature " << Signature << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : Version " << Version << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : HdrSize " << HdrSize << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : TimeType " << TimeType << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : Order " << Order << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : Reserved1 " << Reserved1 << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : VlrCnt " << VlrCnt << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : VlrSize " << VlrSize << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : RecSize " << RecSize << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : FastCnt " << FastCnt << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : RecCnt " << RecCnt << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : UnitsXyz " << UnitsXyz << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : UnitsDistance " << UnitsDistance << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : OrgX " << OrgX << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : OrgY " << OrgY << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : OrgZ " << OrgZ << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : MinX " << MinX << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : MaxX " << MaxX << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : MinY " << MinY << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : MaxY " << MaxY << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : MinZ " << MinZ << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : MaxZ " << MaxZ << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : System " << System << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : Software " << Software << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : Reserved2 " << Reserved2 << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsX " << BitsX << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsY " << BitsY << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsZ " << BitsZ << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsTime " << BitsTime << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsDistance " << BitsDistance << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsGroup " << BitsGroup << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsImage " << BitsImage << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsNormal " << BitsNormal << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsColor " << BitsColor << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsIntensity " << BitsIntensity << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsLine " << BitsLine << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsEchoLen " << BitsEchoLen << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsAmplitude " << BitsAmplitude << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsScanner " << BitsScanner << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsEcho " << BitsEcho << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsAngle " << BitsAngle << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsEchoNorm " << BitsEchoNorm << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsClass " << BitsClass << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsEchoPos " << BitsEchoPos << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsReflect " << BitsReflect << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsDeviation " << BitsDeviation << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : BitsReliab " << BitsReliab << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : Reserved5 " << Reserved5 << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosVlr " << PosVlr << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosXyz " << PosXyz << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosTime " << PosTime << " " <<  std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosDistance " << PosDistance << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosGroup " << PosGroup << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosNormal " << PosNormal << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosColor " << PosColor << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosIntensity " << PosIntensity << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosLine " << PosLine << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosEchoLen " << PosEchoLen << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosAmplitude " << PosAmplitude << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosScanner " << PosScanner << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosEcho " << PosEcho << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosAngle " << PosAngle << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosEchoNorm " << PosEchoNorm << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosClass " << PosClass << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosRecord " << PosRecord << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosEchoPos " << PosEchoPos << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosImage " << PosImage << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosReflect " << PosReflect << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosDeviatio " << PosDeviation << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosReliab " << PosReliab << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : PosImgNbr " << PosImgNbr << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : ImgNbrCnt " << ImgNbrCnt << std::endl;
    log->get(LogLevel::Debug) << "Fbi header : Reserved6 " << Reserved6 << std::endl;
}

} // namespace fbi
} // namespace pdal

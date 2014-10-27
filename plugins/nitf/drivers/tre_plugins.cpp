/******************************************************************************
* Copyright (c) 2014, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include "tre_plugins.hpp"

#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wredundant-decls"
#  pragma GCC diagnostic ignored "-Wfloat-equal"
#  pragma GCC diagnostic ignored "-Wextra"
#  pragma GCC diagnostic ignored "-Wcast-qual"
   // The following pragma doesn't actually work:
   //   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61653
#  pragma GCC diagnostic ignored "-Wliteral-suffix"
#endif
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wfloat-equal"
#  pragma clang diagnostic ignored "-Wunused-private-field"
#endif

#define IMPORT_NITRO_API
#include <nitro/c++/import/nitf.hpp>
#include <nitro/c++/except/Trace.h>

#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic pop
#endif
#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic pop
#endif

namespace pdal
{
namespace drivers
{
namespace nitf
{

// these are all the ones listed in nitro's c/nitf/shared/ directory
NITF_TRE_STATIC_HANDLER_REF(ACCHZB);
NITF_TRE_STATIC_HANDLER_REF(ACCPOB);
NITF_TRE_STATIC_HANDLER_REF(ACCVTB);
NITF_TRE_STATIC_HANDLER_REF(ACFTA);
NITF_TRE_STATIC_HANDLER_REF(ACFTB);
NITF_TRE_STATIC_HANDLER_REF(AIMIDA);
NITF_TRE_STATIC_HANDLER_REF(AIMIDB);
NITF_TRE_STATIC_HANDLER_REF(AIPBCA);
NITF_TRE_STATIC_HANDLER_REF(ASTORA);
NITF_TRE_STATIC_HANDLER_REF(BANDSA);
NITF_TRE_STATIC_HANDLER_REF(BANDSB);
NITF_TRE_STATIC_HANDLER_REF(BCKGDA);
NITF_TRE_STATIC_HANDLER_REF(BLOCKA);
NITF_TRE_STATIC_HANDLER_REF(BNDPLB);
NITF_TRE_STATIC_HANDLER_REF(CLCTNA);
NITF_TRE_STATIC_HANDLER_REF(CLCTNB);
NITF_TRE_STATIC_HANDLER_REF(CMETAA);
NITF_TRE_STATIC_HANDLER_REF(CSCCGA);
NITF_TRE_STATIC_HANDLER_REF(CSCRNA);
NITF_TRE_STATIC_HANDLER_REF(CSDIDA);
NITF_TRE_STATIC_HANDLER_REF(CSEPHA);
NITF_TRE_STATIC_HANDLER_REF(CSEXRA);
NITF_TRE_STATIC_HANDLER_REF(CSPROA);
NITF_TRE_STATIC_HANDLER_REF(CSSFAA);
NITF_TRE_STATIC_HANDLER_REF(CSSHPA);
NITF_TRE_STATIC_HANDLER_REF(ENGRDA);
NITF_TRE_STATIC_HANDLER_REF(EXOPTA);
NITF_TRE_STATIC_HANDLER_REF(EXPLTA);
NITF_TRE_STATIC_HANDLER_REF(EXPLTB);
NITF_TRE_STATIC_HANDLER_REF(GEOLOB);
NITF_TRE_STATIC_HANDLER_REF(GEOPSB);
NITF_TRE_STATIC_HANDLER_REF(GRDPSB);
NITF_TRE_STATIC_HANDLER_REF(HISTOA);
NITF_TRE_STATIC_HANDLER_REF(ICHIPB);
NITF_TRE_STATIC_HANDLER_REF(IMGDTA);
NITF_TRE_STATIC_HANDLER_REF(IOMAPA);
NITF_TRE_STATIC_HANDLER_REF(J2KLRA);
NITF_TRE_STATIC_HANDLER_REF(JITCID);
NITF_TRE_STATIC_HANDLER_REF(MAPLOB);
NITF_TRE_STATIC_HANDLER_REF(MENSRA);
NITF_TRE_STATIC_HANDLER_REF(MENSRB);
NITF_TRE_STATIC_HANDLER_REF(MPDSRA);
NITF_TRE_STATIC_HANDLER_REF(MSTGTA);
NITF_TRE_STATIC_HANDLER_REF(MTIRPA);
NITF_TRE_STATIC_HANDLER_REF(MTIRPB);
NITF_TRE_STATIC_HANDLER_REF(NBLOCA);
NITF_TRE_STATIC_HANDLER_REF(OBJCTA);
NITF_TRE_STATIC_HANDLER_REF(OFFSET);
NITF_TRE_STATIC_HANDLER_REF(PATCHA);
NITF_TRE_STATIC_HANDLER_REF(PATCHB);
NITF_TRE_STATIC_HANDLER_REF(PIAEQA);
NITF_TRE_STATIC_HANDLER_REF(PIAEVA);
NITF_TRE_STATIC_HANDLER_REF(PIAIMB);
NITF_TRE_STATIC_HANDLER_REF(PIAIMC);
NITF_TRE_STATIC_HANDLER_REF(PIAPEA);
NITF_TRE_STATIC_HANDLER_REF(PIAPEB);
NITF_TRE_STATIC_HANDLER_REF(PIAPRC);
NITF_TRE_STATIC_HANDLER_REF(PIAPRD);
NITF_TRE_STATIC_HANDLER_REF(PIATGA);
NITF_TRE_STATIC_HANDLER_REF(PIATGB);
NITF_TRE_STATIC_HANDLER_REF(PIXQLA);
NITF_TRE_STATIC_HANDLER_REF(PLTFMA);
NITF_TRE_STATIC_HANDLER_REF(PRJPSB);
NITF_TRE_STATIC_HANDLER_REF(REGPTB);
NITF_TRE_STATIC_HANDLER_REF(RPC00B);
NITF_TRE_STATIC_HANDLER_REF(RPFDES);
NITF_TRE_STATIC_HANDLER_REF(RPFHDR);
NITF_TRE_STATIC_HANDLER_REF(RPFIMG);
NITF_TRE_STATIC_HANDLER_REF(RSMAPA);
NITF_TRE_STATIC_HANDLER_REF(RSMDCA);
NITF_TRE_STATIC_HANDLER_REF(RSMECA);
NITF_TRE_STATIC_HANDLER_REF(RSMGGA);
NITF_TRE_STATIC_HANDLER_REF(RSMGIA);
NITF_TRE_STATIC_HANDLER_REF(RSMIDA);
NITF_TRE_STATIC_HANDLER_REF(RSMPCA);
NITF_TRE_STATIC_HANDLER_REF(RSMPIA);
NITF_TRE_STATIC_HANDLER_REF(SECTGA);
NITF_TRE_STATIC_HANDLER_REF(SENSRA);
NITF_TRE_STATIC_HANDLER_REF(SENSRB);
NITF_TRE_STATIC_HANDLER_REF(SNSPSB);
NITF_TRE_STATIC_HANDLER_REF(SNSRA);
NITF_TRE_STATIC_HANDLER_REF(SOURCB);
NITF_TRE_STATIC_HANDLER_REF(STDIDC);
NITF_TRE_STATIC_HANDLER_REF(STEROB);
NITF_TRE_STATIC_HANDLER_REF(STREOB);
NITF_TRE_STATIC_HANDLER_REF(TRGTA);
NITF_TRE_STATIC_HANDLER_REF(USE00A);


void register_tre_plugins()
{
    ::nitf::PluginRegistry::registerTREHandler(ACCHZB_init, ACCHZB_handler);
    ::nitf::PluginRegistry::registerTREHandler(ACCPOB_init, ACCPOB_handler);
    ::nitf::PluginRegistry::registerTREHandler(ACCVTB_init, ACCVTB_handler);
    ::nitf::PluginRegistry::registerTREHandler(ACFTA_init, ACFTA_handler);
    ::nitf::PluginRegistry::registerTREHandler(ACFTB_init, ACFTB_handler);
    ::nitf::PluginRegistry::registerTREHandler(AIMIDA_init, AIMIDA_handler);
    ::nitf::PluginRegistry::registerTREHandler(AIMIDB_init, AIMIDB_handler);
    ::nitf::PluginRegistry::registerTREHandler(AIPBCA_init, AIPBCA_handler);
    ::nitf::PluginRegistry::registerTREHandler(ASTORA_init, ASTORA_handler);
    ::nitf::PluginRegistry::registerTREHandler(BANDSA_init, BANDSA_handler);
    ::nitf::PluginRegistry::registerTREHandler(BANDSB_init, BANDSB_handler);
    ::nitf::PluginRegistry::registerTREHandler(BCKGDA_init, BCKGDA_handler);
    ::nitf::PluginRegistry::registerTREHandler(BLOCKA_init, BLOCKA_handler);
    ::nitf::PluginRegistry::registerTREHandler(BNDPLB_init, BNDPLB_handler);
    ::nitf::PluginRegistry::registerTREHandler(CLCTNA_init, CLCTNA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CLCTNB_init, CLCTNB_handler);
    ::nitf::PluginRegistry::registerTREHandler(CMETAA_init, CMETAA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CSCCGA_init, CSCCGA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CSCRNA_init, CSCRNA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CSDIDA_init, CSDIDA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CSEPHA_init, CSEPHA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CSEXRA_init, CSEXRA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CSPROA_init, CSPROA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CSSFAA_init, CSSFAA_handler);
    ::nitf::PluginRegistry::registerTREHandler(CSSHPA_init, CSSHPA_handler);
    ::nitf::PluginRegistry::registerTREHandler(ENGRDA_init, ENGRDA_handler);
    ::nitf::PluginRegistry::registerTREHandler(EXOPTA_init, EXOPTA_handler);
    ::nitf::PluginRegistry::registerTREHandler(EXPLTA_init, EXPLTA_handler);
    ::nitf::PluginRegistry::registerTREHandler(EXPLTB_init, EXPLTB_handler);
    ::nitf::PluginRegistry::registerTREHandler(GEOLOB_init, GEOLOB_handler);
    ::nitf::PluginRegistry::registerTREHandler(GEOPSB_init, GEOPSB_handler);
    ::nitf::PluginRegistry::registerTREHandler(GRDPSB_init, GRDPSB_handler);
    ::nitf::PluginRegistry::registerTREHandler(HISTOA_init, HISTOA_handler);
    ::nitf::PluginRegistry::registerTREHandler(ICHIPB_init, ICHIPB_handler);
    ::nitf::PluginRegistry::registerTREHandler(IMGDTA_init, IMGDTA_handler);
    ::nitf::PluginRegistry::registerTREHandler(IOMAPA_init, IOMAPA_handler);
    ::nitf::PluginRegistry::registerTREHandler(J2KLRA_init, J2KLRA_handler);
    ::nitf::PluginRegistry::registerTREHandler(JITCID_init, JITCID_handler);
    ::nitf::PluginRegistry::registerTREHandler(MAPLOB_init, MAPLOB_handler);
    ::nitf::PluginRegistry::registerTREHandler(MENSRA_init, MENSRA_handler);
    ::nitf::PluginRegistry::registerTREHandler(MENSRB_init, MENSRB_handler);
    ::nitf::PluginRegistry::registerTREHandler(MPDSRA_init, MPDSRA_handler);
    ::nitf::PluginRegistry::registerTREHandler(MSTGTA_init, MSTGTA_handler);
    ::nitf::PluginRegistry::registerTREHandler(MTIRPA_init, MTIRPA_handler);
    ::nitf::PluginRegistry::registerTREHandler(MTIRPB_init, MTIRPB_handler);
    ::nitf::PluginRegistry::registerTREHandler(NBLOCA_init, NBLOCA_handler);
    ::nitf::PluginRegistry::registerTREHandler(OBJCTA_init, OBJCTA_handler);
    ::nitf::PluginRegistry::registerTREHandler(OFFSET_init, OFFSET_handler);
    ::nitf::PluginRegistry::registerTREHandler(PATCHA_init, PATCHA_handler);
    ::nitf::PluginRegistry::registerTREHandler(PATCHB_init, PATCHB_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIAEQA_init, PIAEQA_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIAEVA_init, PIAEVA_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIAIMB_init, PIAIMB_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIAIMC_init, PIAIMC_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIAPEA_init, PIAPEA_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIAPEB_init, PIAPEB_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIAPRC_init, PIAPRC_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIAPRD_init, PIAPRD_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIATGA_init, PIATGA_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIATGB_init, PIATGB_handler);
    ::nitf::PluginRegistry::registerTREHandler(PIXQLA_init, PIXQLA_handler);
    ::nitf::PluginRegistry::registerTREHandler(PLTFMA_init, PLTFMA_handler);
    ::nitf::PluginRegistry::registerTREHandler(PRJPSB_init, PRJPSB_handler);
    ::nitf::PluginRegistry::registerTREHandler(REGPTB_init, REGPTB_handler);
    ::nitf::PluginRegistry::registerTREHandler(RPC00B_init, RPC00B_handler);
    ::nitf::PluginRegistry::registerTREHandler(RPFDES_init, RPFDES_handler);
    ::nitf::PluginRegistry::registerTREHandler(RPFHDR_init, RPFHDR_handler);
    ::nitf::PluginRegistry::registerTREHandler(RPFIMG_init, RPFIMG_handler);
    ::nitf::PluginRegistry::registerTREHandler(RSMAPA_init, RSMAPA_handler);
    ::nitf::PluginRegistry::registerTREHandler(RSMDCA_init, RSMDCA_handler);
    ::nitf::PluginRegistry::registerTREHandler(RSMECA_init, RSMECA_handler);
    ::nitf::PluginRegistry::registerTREHandler(RSMGGA_init, RSMGGA_handler);
    ::nitf::PluginRegistry::registerTREHandler(RSMGIA_init, RSMGIA_handler);
    ::nitf::PluginRegistry::registerTREHandler(RSMIDA_init, RSMIDA_handler);
    ::nitf::PluginRegistry::registerTREHandler(RSMPCA_init, RSMPCA_handler);
    ::nitf::PluginRegistry::registerTREHandler(RSMPIA_init, RSMPIA_handler);
    ::nitf::PluginRegistry::registerTREHandler(SECTGA_init, SECTGA_handler);
    ::nitf::PluginRegistry::registerTREHandler(SENSRA_init, SENSRA_handler);
    ::nitf::PluginRegistry::registerTREHandler(SENSRB_init, SENSRB_handler);
    ::nitf::PluginRegistry::registerTREHandler(SNSPSB_init, SNSPSB_handler);
    ::nitf::PluginRegistry::registerTREHandler(SNSRA_init, SNSRA_handler);
    ::nitf::PluginRegistry::registerTREHandler(SOURCB_init, SOURCB_handler);
    ::nitf::PluginRegistry::registerTREHandler(STDIDC_init, STDIDC_handler);
    ::nitf::PluginRegistry::registerTREHandler(STEROB_init, STEROB_handler);
    ::nitf::PluginRegistry::registerTREHandler(STREOB_init, STREOB_handler);
    ::nitf::PluginRegistry::registerTREHandler(TRGTA_init, TRGTA_handler);
    ::nitf::PluginRegistry::registerTREHandler(USE00A_init, USE00A_handler);
}


}
}
} // namespaces


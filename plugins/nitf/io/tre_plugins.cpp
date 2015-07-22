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
#  pragma GCC diagnostic ignored "-Wextra"
#  pragma GCC diagnostic ignored "-Wcast-qual"
   // The following pragma doesn't actually work:
   //   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61653
   //#  pragma GCC diagnostic ignored "-Wliteral-suffix"
#endif
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
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

void register_tre_handler(NITF_PLUGIN_INIT_FUNCTION init, NITF_PLUGIN_TRE_HANDLER_FUNCTION handler)
{
  nitf_Error error;
  if (!nitf_PluginRegistry_registerTREHandler(init, handler, &error))
    throw ::nitf::NITFException(&error);
}

void register_tre_plugins()
{
    register_tre_handler(ACCHZB_init, ACCHZB_handler);
    register_tre_handler(ACCPOB_init, ACCPOB_handler);
    register_tre_handler(ACCVTB_init, ACCVTB_handler);
    register_tre_handler(ACFTA_init, ACFTA_handler);
    register_tre_handler(ACFTB_init, ACFTB_handler);
    register_tre_handler(AIMIDA_init, AIMIDA_handler);
    register_tre_handler(AIMIDB_init, AIMIDB_handler);
    register_tre_handler(AIPBCA_init, AIPBCA_handler);
    register_tre_handler(ASTORA_init, ASTORA_handler);
    register_tre_handler(BANDSA_init, BANDSA_handler);
    register_tre_handler(BANDSB_init, BANDSB_handler);
    register_tre_handler(BCKGDA_init, BCKGDA_handler);
    register_tre_handler(BLOCKA_init, BLOCKA_handler);
    register_tre_handler(BNDPLB_init, BNDPLB_handler);
    register_tre_handler(CLCTNA_init, CLCTNA_handler);
    register_tre_handler(CLCTNB_init, CLCTNB_handler);
    register_tre_handler(CMETAA_init, CMETAA_handler);
    register_tre_handler(CSCCGA_init, CSCCGA_handler);
    register_tre_handler(CSCRNA_init, CSCRNA_handler);
    register_tre_handler(CSDIDA_init, CSDIDA_handler);
    register_tre_handler(CSEPHA_init, CSEPHA_handler);
    register_tre_handler(CSEXRA_init, CSEXRA_handler);
    register_tre_handler(CSPROA_init, CSPROA_handler);
    register_tre_handler(CSSFAA_init, CSSFAA_handler);
    register_tre_handler(CSSHPA_init, CSSHPA_handler);
    register_tre_handler(ENGRDA_init, ENGRDA_handler);
    register_tre_handler(EXOPTA_init, EXOPTA_handler);
    register_tre_handler(EXPLTA_init, EXPLTA_handler);
    register_tre_handler(EXPLTB_init, EXPLTB_handler);
    register_tre_handler(GEOLOB_init, GEOLOB_handler);
    register_tre_handler(GEOPSB_init, GEOPSB_handler);
    register_tre_handler(GRDPSB_init, GRDPSB_handler);
    register_tre_handler(HISTOA_init, HISTOA_handler);
    register_tre_handler(ICHIPB_init, ICHIPB_handler);
    register_tre_handler(IMGDTA_init, IMGDTA_handler);
    register_tre_handler(IOMAPA_init, IOMAPA_handler);
    register_tre_handler(J2KLRA_init, J2KLRA_handler);
    register_tre_handler(JITCID_init, JITCID_handler);
    register_tre_handler(MAPLOB_init, MAPLOB_handler);
    register_tre_handler(MENSRA_init, MENSRA_handler);
    register_tre_handler(MENSRB_init, MENSRB_handler);
    register_tre_handler(MPDSRA_init, MPDSRA_handler);
    register_tre_handler(MSTGTA_init, MSTGTA_handler);
    register_tre_handler(MTIRPA_init, MTIRPA_handler);
    register_tre_handler(MTIRPB_init, MTIRPB_handler);
    register_tre_handler(NBLOCA_init, NBLOCA_handler);
    register_tre_handler(OBJCTA_init, OBJCTA_handler);
    register_tre_handler(OFFSET_init, OFFSET_handler);
    register_tre_handler(PATCHA_init, PATCHA_handler);
    register_tre_handler(PATCHB_init, PATCHB_handler);
    register_tre_handler(PIAEQA_init, PIAEQA_handler);
    register_tre_handler(PIAEVA_init, PIAEVA_handler);
    register_tre_handler(PIAIMB_init, PIAIMB_handler);
    register_tre_handler(PIAIMC_init, PIAIMC_handler);
    register_tre_handler(PIAPEA_init, PIAPEA_handler);
    register_tre_handler(PIAPEB_init, PIAPEB_handler);
    register_tre_handler(PIAPRC_init, PIAPRC_handler);
    register_tre_handler(PIAPRD_init, PIAPRD_handler);
    register_tre_handler(PIATGA_init, PIATGA_handler);
    register_tre_handler(PIATGB_init, PIATGB_handler);
    register_tre_handler(PIXQLA_init, PIXQLA_handler);
    register_tre_handler(PLTFMA_init, PLTFMA_handler);
    register_tre_handler(PRJPSB_init, PRJPSB_handler);
    register_tre_handler(REGPTB_init, REGPTB_handler);
    register_tre_handler(RPC00B_init, RPC00B_handler);
    register_tre_handler(RPFDES_init, RPFDES_handler);
    register_tre_handler(RPFHDR_init, RPFHDR_handler);
    register_tre_handler(RPFIMG_init, RPFIMG_handler);
    register_tre_handler(RSMAPA_init, RSMAPA_handler);
    register_tre_handler(RSMDCA_init, RSMDCA_handler);
    register_tre_handler(RSMECA_init, RSMECA_handler);
    register_tre_handler(RSMGGA_init, RSMGGA_handler);
    register_tre_handler(RSMGIA_init, RSMGIA_handler);
    register_tre_handler(RSMIDA_init, RSMIDA_handler);
    register_tre_handler(RSMPCA_init, RSMPCA_handler);
    register_tre_handler(RSMPIA_init, RSMPIA_handler);
    register_tre_handler(SECTGA_init, SECTGA_handler);
    register_tre_handler(SENSRA_init, SENSRA_handler);
    register_tre_handler(SENSRB_init, SENSRB_handler);
    register_tre_handler(SNSPSB_init, SNSPSB_handler);
    register_tre_handler(SNSRA_init, SNSRA_handler);
    register_tre_handler(SOURCB_init, SOURCB_handler);
    register_tre_handler(STDIDC_init, STDIDC_handler);
    register_tre_handler(STEROB_init, STEROB_handler);
    register_tre_handler(STREOB_init, STREOB_handler);
    register_tre_handler(TRGTA_init, TRGTA_handler);
    register_tre_handler(USE00A_init, USE00A_handler);
}


} // namespaces

/******************************************************************************
* Copyright (c) 2011, OneOcean (mpg@1ocean.com)
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

#include "caris_pc_wrapper.h"

#pragma warning(disable: 4100)  // unrefeences formal parameter

#ifdef  __cplusplus
extern "C" {
#endif

/************************************************************************
 * misc functions
 ************************************************************************/

int         caris_init(size_t in_cache_bytes, char const* in_support_dir) { return 0; }
char const* caris_status_message(int in_status, int in_verbose) { return 0; }


/************************************************************************
 * caris_cloud functions
 ************************************************************************/

int         caris_cloud_open(
                const char* in_uri, 
                caris_cloud** out_cloud,
                caris_log_func in_log_callback,
                void* in_log_callback_user_data) { return 0; }
int         caris_cloud_release(caris_cloud* in_cloud) { return 0; }
int64_t     caris_cloud_num_points(caris_cloud* in_cloud) { return 0; }
void        caris_cloud_dimensions(caris_cloud* in_cloud, 
                caris_dimension const** out_dimensions,
                int * out_num_dimensions) { return; }
void        caris_cloud_extents(caris_cloud* in_cloud, 
                double* out_min_x, double* out_min_y, double* out_min_z, 
                double* out_max_x, double* out_max_y, double* out_max_z) { return; }
char const* caris_cloud_spatial_reference(caris_cloud* in_cloud) { return 0; }
int         caris_cloud_status(caris_cloud* in_cloud) { return 0; }
caris_itr*  caris_cloud_create_itr(caris_cloud* in_cloud) { return 0; }


/************************************************************************
 * caris_itr functions
 ************************************************************************/

void        caris_itr_release(caris_itr* in_itr) { return; }
void        caris_itr_next(caris_itr* in_itr) { return; }
int         caris_itr_done(caris_itr* in_itr) { return 0; }
int32_t     caris_itr_num_points(caris_itr* in_itr) { return 0; }
void const* caris_itr_read(caris_itr* in_itr, 
                int in_dimension_index, int32_t * out_num_points) { return 0; }
int         caris_itr_status(caris_itr* in_itr) { return 0; }



#ifdef  __cplusplus
}
#endif

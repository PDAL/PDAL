/************************************************************************
 * Copyright (c) 2012, CARIS
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of CARIS nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/
#ifndef CARIS_CARIS_H
#define CARIS_CARIS_H

/* stdint.h is missing from msvc < v10/2010 */
#if defined(_MSC_VER) && _MSC_VER < 1600
    typedef   signed __int8    int8_t;
    typedef unsigned __int8   uint8_t;
    typedef   signed __int16   int16_t;
    typedef unsigned __int16  uint16_t;
    typedef   signed __int32   int32_t;
    typedef unsigned __int32  uint32_t;
    typedef   signed __int64   int64_t;
    typedef unsigned __int64  uint64_t;
#else
    #include <stdint.h>
#endif
#include <stdlib.h>

#ifdef  __cplusplus
extern "C" {
#endif

/************************************************************************
 * constants
 ************************************************************************/
 
#define CARIS_VERSION_MAJOR 0
#define CARIS_VERSION_MINOR 1
#define CARIS_VERSION_PATCH 0

/*! \brief enum of numeric pod types */
enum caris_type
{
    CARIS_TYPE_FLOAT32 = 0,
    CARIS_TYPE_FLOAT64 = 1,
    CARIS_TYPE_INT8    = 2,
    CARIS_TYPE_UINT8   = 3,
    CARIS_TYPE_INT16   = 4,
    CARIS_TYPE_UINT16  = 5,
    CARIS_TYPE_INT32   = 6,
    CARIS_TYPE_UINT32  = 7,
    CARIS_TYPE_INT64   = 8,
    CARIS_TYPE_UINT64  = 9
};


/************************************************************************
 * Types
 ************************************************************************/
 
/*! \brief Opaque type for accessing Caris cloud data
    Similar to pdal::Reader */
typedef struct caris_cloud caris_cloud;

/*! \brief Opaque type for iterating Caris cloud data in blocks
    Similar to pdal::SequentialIterator */
typedef struct caris_itr caris_itr;

typedef void(*caris_log_func)(void* in_user_data, char const* in_message);

/*! \brief Details of dimension from a caris_cloud */
typedef struct caris_dimension
{
    int         type;           /*!< type of stored data (caris_type) */
    int         tuple_length;   /*!< number of \e type entries per field */
    int         reserved0;
    int         reserved1;
    int         reserved2;
    int         reserved3;
    char*       name;           /*!< name of the dimension */
    void*       reserved4;
    void*       reserved5;
    void*       reserved6;
    void*       reserved7;
    void*       reserved8;
} caris_dimension;

/************************************************************************
 * misc functions
 ************************************************************************/

int         caris_init(size_t in_cache_bytes, char const* in_support_dir);
char const* caris_status_message(int in_status, int in_verbose);


/************************************************************************
 * caris_cloud functions
 ************************************************************************/

int         caris_cloud_open(
                const char* in_uri, 
                caris_cloud** out_cloud,
                caris_log_func in_log_callback,
                void* in_log_callback_user_data);
int         caris_cloud_release(caris_cloud* in_cloud);
int64_t     caris_cloud_num_points(caris_cloud* in_cloud);
void        caris_cloud_dimensions(caris_cloud* in_cloud, 
                caris_dimension const** out_dimensions,
                int * out_num_dimensions);
void        caris_cloud_extents(caris_cloud* in_cloud, 
                double* out_min_x, double* out_min_y, double* out_min_z, 
                double* out_max_x, double* out_max_y, double* out_max_z);
char const* caris_cloud_spatial_reference(caris_cloud* in_cloud);
int         caris_cloud_status(caris_cloud* in_cloud);
caris_itr*  caris_cloud_create_itr(caris_cloud* in_cloud);


/************************************************************************
 * caris_itr functions
 ************************************************************************/

void        caris_itr_release(caris_itr* in_itr);
void        caris_itr_next(caris_itr* in_itr);
int         caris_itr_done(caris_itr* in_itr);
int32_t     caris_itr_num_points(caris_itr* in_itr);
void const* caris_itr_read(caris_itr* in_itr, 
                int in_dimension_index, int32_t * out_num_points);
int         caris_itr_status(caris_itr* in_itr);


/************************************************************************
 * doxygen
 ************************************************************************/

/*! \name General functions 
@{
    
    \fn int caris_init(size_t, char const*);
    \brief initialize the CARIS environment
    
    This should be called before using any other functions
    
    \param in_cache_bytes 
        \li cache size in bytes
    \param in_support_dir
        \li directory with CARIS support files
    \return
        \li 0 if the method succeeded, error code if it failed. 
    
    \fn char const* caris_status_message(int in_status, int in_verbose)
    \brief initialize the CARIS environment
    
    \param in_status 
        \li error code
    \param in_verbose
        \li set to true to include debug information
    \return
        \li a message for the given error code
        \li valid until the next call to caris_status_message
        \li never null
*/
/*! @} */


/*! \name caris_cloud functions
    @{
    
    \fn int     caris_cloud_open(
                    const char* in_uri, 
                    caris_cloud** out_cloud,
                    caris_log_func in_log_callback,
                    void* in_log_callback_user_data);
    \brief open a caris_cloud

    \param in_uri 
        \li URI of the cloud to open
    \param out_cloud
        \li set to new caris_cloud pointer
    \param in_log_callback
        \li optional callback for traking open progress
    \param in_log_callback_user_data
        \li parameter to be passed to in_log_callback
    \return
        \li 0 if the method succeeded, error code if it failed. 

    \fn int caris_cloud_release(caris_cloud* in_cloud)
    \brief release the resourses owned by a caris_cloud

    \param in_cloud 
        \li cloud to close
    \return
        \li 0 if the method succeeded, error code if it failed. 

    \fn int64_t     caris_cloud_num_points(caris_cloud* in_cloud);
    \brief get the number of points 
    \param in_cloud
        \li related caris_cloud
    \return
        \li the number of points in \e in_cloud
        \li 0 on failure, error available via caris_cloud_status()
        

    \fn void        caris_cloud_dimensions(caris_cloud* in_cloud, 
                        caris_dimension const** out_dimensions,
                        int * out_num_dimensions);
    \brief get the dimensions
    \param in_cloud
        \li related caris_cloud
    \param out_dimensions
        \li set to a pointer to an array of  caris_dimension
    \param out_num_dimensions
        \li set to the number of elements in \e out_dimensions
                        
    \fn void        caris_cloud_extents(caris_cloud* in_cloud, 
                        double* out_min_x, double* out_min_y, double* out_min_z, 
                        double* out_max_x, double* out_max_y, double* out_max_z);
    \brief get the extents (bound)
    \param in_cloud
        \li related caris_cloud
    \param out_min_x, out_min_y, out_min_z, out_max_x, out_max_y, out_max_z
        \li set the xyz extents of \e in_cloud

    \fn char const* caris_cloud_spatial_reference(caris_cloud* in_cloud);
    \brief get the spatial reference as a wkt string
    \param in_cloud
        \li related caris_cloud
    \return
        \li spatial reference as a wkt string
        \li NULL if the spatial reference could not be converted to WKT

    \fn int         caris_cloud_status(caris_cloud* in_cloud);
    \brief get the status
    \param in_cloud
        \li related caris_cloud
    \return
        \li 0 if the method succeeded, error code if it failed. 
        \li additionaly details available by passing this status to caris_status_message()

    \fn caris_itr*  caris_cloud_create_itr(caris_cloud* in_cloud)
    \brief create a new caris_itr
    \param in_cloud
        \li related caris_cloud
        \li must remain valid until the iterater is released
    \return
        \li a new caris_itr
        \li must be released with caris_itr_release()
        \li NULL on error, status available via caris_cloud_status()
    
*/
/*! @} */


/*! \name caris_itr functions
    @{

    \fn void caris_itr_release(caris_itr* in_itr)
    \brief release the resourses owned by a caris_itr
    \param in_itr
        \li iterator to release

    \fn void        caris_itr_next(caris_itr* in_itr)
    \brief advance the iterator to the next block of data
           May invalidate any pointers returned by caris_itr_read
    \sa caris_itr_done()
    \param in_itr
        \li related caris_itr

    \fn int         caris_itr_done(caris_itr* in_itr);
    \brief check if iteration is complete
    \param in_itr
        \li related caris_itr
    \return
        \li true if iteration is complete
        \li false if the current block is valid

    \fn int32_t     caris_itr_num_points(caris_itr* in_itr);
    \brief get the number of points in the current block
    \param in_itr
        \li related caris_itr
    \return
        \li number of points in the current block
        \li 0 on error

    \fn void const* caris_itr_read(caris_itr* in_itr, 
                        int in_dimension_index, int32_t * out_num_points);
    \brief read data from the current block
    \param in_itr
        \li related caris_itr
    \param in_dimension_index
        \li dimension index (array index of the dimension returned by caris_cloud_dimensions())
    \param out_num_points
        \li set to the number of points read
    \return
        \li pointer to data from the current block for dimension \e in_dimension_index
        \li NULL on error, status available via caris_itr_status()
                        
    \fn int         caris_itr_status(caris_itr* in_itr);
    \brief get the status
    \param in_itr
        \li related caris_itr
    \return
        \li 0 if the method succeeded, error code if it failed. 
        \li additionaly details available by passing this status to caris_status_message()

*/
/*! @} */

#ifdef  __cplusplus
}
#endif
#endif

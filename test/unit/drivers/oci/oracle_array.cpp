

#include <stdio.h>
#include <stdlib.h> 
 #include <string.h>
 #include <oci.h>
#include <iostream>
#include <sstream>

bool CheckError(sword nStatus, OCIError* hError)
{
    text    szMsg[512];
    sb4     nCode = 0;
    
    std::stringstream err;
    switch (nStatus)
    {
        case OCI_SUCCESS:
            return false;
            break;
        case OCI_NEED_DATA:
            err << "OCI_NEED_DATA" << std::endl;;
            break;
        case OCI_NO_DATA:
            err << "OCI_NO_DATA" << std::endl;;
            break;
        case OCI_INVALID_HANDLE:
            err << "OCI_INVALID_HANDLE" << std::endl;;
            break;
        case OCI_STILL_EXECUTING:
            err << "OCI_STILL_EXECUTING" << std::endl;;
            break;
        case OCI_CONTINUE:
            err << "OCI_CONTINUE" << std::endl;;
            break;
        case OCI_ERROR:
        case OCI_SUCCESS_WITH_INFO:

            if (hError == NULL)
            {
                err << "OCI_ERROR with no error handler" << std::endl;;
            }

            OCIErrorGet((dvoid *) hError, (ub4) 1,
                        (text *) NULL, &nCode, szMsg,
                        (ub4) sizeof(szMsg), OCI_HTYPE_ERROR);

            if (nCode == 1405)  // Null field
            {
                return false;
            }

            err << "Error: " << szMsg << std::endl;
            break;

        default:

            if (hError == NULL)
            {
                err << "default OCI_ERROR with no error handler" << std::endl;;

            }

            OCIErrorGet((dvoid *) hError, (ub4) 1,
                        (text *) NULL, &nCode, szMsg,
                        (ub4) sizeof(szMsg), OCI_HTYPE_ERROR);
            // err << "Error: " << std::string(szMsg, static_cast<int>(sizeof(szMsg))) << std::endl;
            break;

    }
    
    std::cout << "Error: " << err.str() << std::endl;
    exit(1);

    return true;
}


 int main()
 {
       // OCI handles
OCIEnv *envhp;
OCIError *errhp;
OCIServer *srvhp;
OCISvcCtx *svchp;
OCISession *authp;
OCIStmt *stmtp;
OCIDefine *defnpp;

// Connection information
text* user = (text*)"grid";
text* pwd = (text*)"grid";
text* sid = (text*)"localhost/orcl";

#define FETCH_COUNT 40
int prefetch_rows(FETCH_COUNT);
int fetched;

// char *query = "SELECT owner, table_name FROM all_tables";
// char *query = "SELECT obj_id, blk_id, points FROM autzen_blocks";
char *query = "SELECT points FROM autzen_blocks where obj_id=1";

// Fetched data
// char owner[FETCH_COUNT][31];
// char table_name[FETCH_COUNT][31];
oraub8 block_id[FETCH_COUNT];
oraub8 object_id[FETCH_COUNT];
OCILobLocator * lob_array[FETCH_COUNT];

// Fetched data indicators, lengths and codes
// sb2 owner_ind[100], table_name_ind[100];
// ub2 owner_len[100], table_name_len[100];
// ub2 owner_code[100], table_name_code[100];

sb2 block_id_ind[FETCH_COUNT], object_id_ind[FETCH_COUNT];
ub2 block_id_len[FETCH_COUNT], object_id_len[FETCH_COUNT];
ub2 block_id_code[FETCH_COUNT], object_id_code[FETCH_COUNT];

int rc(0);


   // lob_array[i] = OCIDescriptorAlloc(..., OCI_DTYPE_LOB, ...);

// Allocate environment
rc = OCIEnvCreate(&envhp, OCI_DEFAULT, NULL, NULL, NULL, NULL, 0, NULL);

// Allocate error handle
rc = OCIHandleAlloc(envhp, (void**)&errhp, OCI_HTYPE_ERROR, 0, NULL);

// Allocate server and service context handles
rc = OCIHandleAlloc(envhp, (void**)&srvhp, OCI_HTYPE_SERVER, 0, NULL);
rc = OCIHandleAlloc(envhp, (void**)&svchp, OCI_HTYPE_SVCCTX, 0, NULL);

// Attach to the server
rc = OCIServerAttach(srvhp, errhp, sid, strlen((char*)sid), 0);

// Set server in the service context 
rc = OCIAttrSet(svchp, OCI_HTYPE_SVCCTX, (dvoid*)srvhp, 0, OCI_ATTR_SERVER, errhp);

// Allocate session handle
rc = OCIHandleAlloc(envhp, (void**)&authp, OCI_HTYPE_SESSION, 0, NULL);

// Set user name and password
rc = OCIAttrSet(authp, OCI_HTYPE_SESSION, (void*)user, strlen((char*)user), 
                                      OCI_ATTR_USERNAME, errhp);
rc = OCIAttrSet(authp, OCI_HTYPE_SESSION, (void*)pwd, strlen((char *)pwd), 
                                      OCI_ATTR_PASSWORD, errhp);

// Connect
rc = OCISessionBegin(svchp, errhp, authp, OCI_CRED_RDBMS, OCI_DEFAULT);

// Set session in the service context
rc = OCIAttrSet(svchp, OCI_HTYPE_SVCCTX, authp, 0, OCI_ATTR_SESSION, errhp);

// Allocate statement handle
rc = OCIHandleAlloc(envhp, (void**)&stmtp, OCI_HTYPE_STMT, 0, NULL);

// Set prefetch count
rc = OCIAttrSet(stmtp, OCI_HTYPE_STMT, (void*)&prefetch_rows, sizeof(int), 
                                      OCI_ATTR_PREFETCH_ROWS, errhp);

// Prepare the query
rc = OCIStmtPrepare(stmtp, errhp, (text*)query, strlen(query), OCI_NTV_SYNTAX, OCI_DEFAULT);

// Define the select list items 
// rc = OCIDefineByPos(stmtp, &defnpp, errhp, 1, (void*)owner, 31, SQLT_STR, (void*)owner_ind,
//                     owner_len, owner_code, OCI_DEFAULT);
// rc = OCIDefineByPos(stmtp, &defnpp, errhp, 2, (void*)table_name, 31, SQLT_STR, (void*)table_name_ind,
//                     table_name_len, table_name_code, OCI_DEFAULT);

for (int i = 0; i < prefetch_rows; i++)
{

    rc = OCIDescriptorAlloc(
        envhp,
          (dvoid **) &lob_array[i],
        OCI_DTYPE_LOB, (size_t) 0, (void**) 0);
    CheckError(rc, errhp);    

    OCILobEnableBuffering(
                           svchp,
                           errhp,
                           lob_array[i]);
}

// rc = OCIDefineByPos(stmtp, &defnpp, errhp, 1, (void*)block_id, (sb4) sizeof(long int), SQLT_INT, (void*)block_id_ind,
//                     block_id_len, block_id_code, OCI_DEFAULT);
// CheckError(rc, errhp);
// rc = OCIDefineByPos(stmtp, &defnpp, errhp, 2, (void*)object_id, (sb4) sizeof(long int), SQLT_INT, (void*)object_id_ind,
//                     object_id_len, object_id_code, OCI_DEFAULT);
// CheckError(rc, errhp);
rc = OCIDefineByPos(stmtp, &defnpp, errhp, 1, (dvoid*)lob_array, (sb4) 0, SQLT_BLOB, 0,
					0, 0, OCI_DEFAULT);
CheckError(rc, errhp);

// Execute the statement and perform the initial fetch of 100 rows into the defined array
rc = OCIStmtExecute(svchp, stmtp, errhp, prefetch_rows, 0, NULL, NULL, OCI_DEFAULT);
std::cout << "executing query" << std::endl;
CheckError(rc, errhp);
int cnt(0);

oraub8 offset[FETCH_COUNT];
char  *bufp[FETCH_COUNT];
oraub8 bufl[FETCH_COUNT] = {0};
oraub8 buf_amtp[FETCH_COUNT] = {0};
ub4 array_iter = prefetch_rows;


while(rc >= 0)
{
	OCIAttrGet(stmtp, OCI_HTYPE_STMT, (void*)&fetched, NULL, OCI_ATTR_ROWS_FETCHED, errhp);
    
    std::cout << "Fetched : " << fetched << std::endl;
	// OCI_NO_DATA is returned by OCIStmtExecute and OCIStmtFetch2 when the number of fetched rows 
	// is less than the number of rows allocated in the array
    cnt += fetched;
	if(fetched == 0)
		break;

    // // Output fetched data 
    // for(int i = 0; i < fetched; i++)
    //         // printf("%s.%s\n", owner[i], table_name[i]);
    //         printf("block_id: %ld. object_id: %ld\n", block_id[i], object_id[i]);

    for (int i=0; i<prefetch_rows; i++)
    {
      buf_amtp[i] = 0;
    }    
    
    int candidate = 2;
    for (int i=0; i<fetched; i++)
    {
        oraub8 nLength      = (oraub8) 0;
        rc = OCILobGetLength2(
                            svchp,
                           errhp,
                           lob_array[i],
                           &nLength);
           std::cout << "i: " << i << " bufl[i]: " << bufl[i] << " nLength: " << nLength << std::endl;
           if (bufl[i] != 0 || bufl[i] < nLength)
           {
               if (bufl[i])
                   free(bufp[i]);
               bufp[i] = (char *)malloc(nLength);
               bufl[i] = nLength;
               
           }
      offset[i] = 1;
      buf_amtp[i] = nLength;

    } 
    

    std::cout << "before bufl["<<candidate<<"]: " << bufl[candidate] << " buf_amtp["<<candidate<<"]: " << buf_amtp[candidate] << " offset["<<candidate<<"]: "<< offset[candidate] << std::endl;             

    rc = OCILobArrayRead(svchp, errhp,
                    &array_iter, /* array size */
                    lob_array,   /* array of locators */
                    buf_amtp,        /* array of byte amounts */
                    NULL,   /* array of char amounts */
                    offset,      /* array of offsets */
           (void **)bufp,        /* array of read buffers */
                    bufl,        /* array of buffer lengths */
                    OCI_ONE_PIECE,  /* piece information */
                    NULL,           /* callback context */
                    NULL,           /* callback function */
                    0,              /* character set ID - default */
                    SQLCS_IMPLICIT);/* character set form */
    // std::cout << "called array read!" << std::endl;
    // CheckError(rc, errhp);      
    // std::cout << "did array read!" << std::endl;       
    // std::cout << afterblob length: " << nLength << std::endl;
    std::cout << "after bufl["<<candidate<<"]: " << bufl[candidate] << " buf_amtp["<<candidate<<"]: " << buf_amtp[candidate] << " offset["<<candidate<<"]: "<< offset[candidate] << std::endl;             
    

    while (rc == OCI_NEED_DATA)
    {
        rc = OCILobArrayRead(svchp, errhp,
                        &array_iter, /* array size */
                        lob_array,   /* array of locators */
                        buf_amtp,        /* array of byte amounts */
                        NULL,   /* array of char amounts */
                        offset,      /* array of offsets */
               (void **)bufp,        /* array of read buffers */
                        bufl,        /* array of buffer lengths */
                        OCI_NEXT_PIECE,  /* piece information */
                        NULL,           /* callback context */
                        NULL,           /* callback function */
                        0,              /* character set ID - default */
                        SQLCS_IMPLICIT);/* character set form */
        
        // for (int i = 0; i < prefetch_rows; ++i)
        // {
        //     offset[i] = offset[i] + bufl[i];
        // }    
        // std::cout << "read array part!" << std::endl;
        // CheckError(rc, errhp);
        std::cout << "part bufl[34]" << bufl[34] << " buf_amtp[34]: " << buf_amtp[34] << "offset[34]: " << offset[34] << std::endl;             
        
        // std::cout << "no error for array part!" << std::endl;       
    }
	if(rc == OCI_NO_DATA)
		break;

    // for (int i=0; i<prefetch_rows; i++)
    // {
    //   offset[i] = 1;
    // }

	// Fetch another set of rows
	rc = OCIStmtFetch2(stmtp, errhp, prefetch_rows, OCI_DEFAULT, 0, OCI_DEFAULT);
}

printf("selected %d blocks\n", cnt);

rc = OCIHandleFree(stmtp, OCI_HTYPE_STMT);

// Disconnect
rc = OCISessionEnd(svchp, errhp, authp, OCI_DEFAULT);
rc = OCIServerDetach(srvhp, errhp, OCI_DEFAULT);

rc = OCIHandleFree(envhp, OCI_HTYPE_ENV);
return 0;
 }

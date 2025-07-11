(filespec)=

# Filespec

In addition to a string or [GDAL VSI](https://gdal.org/en/stable/user/virtual_file_systems.html) filename, HTTP headers and query parameters to be forwarded to remote endpoints
can be specified within the filename option. As shown below, a JSON object can be substituted, with the
'headers' and 'query' fields as JSON objects of key/value string pairs.

```json
{
    "filename":
    {
        "path":"path to remote file [required]",
        "headers":
        {
            "some_header_key": "HTTP header value"
        },
        "query":
        {
            "some_query_key": "HTTP query value"
        }
    }
}
```

## Connecting to Azure Blob Storage Using a SAS Token

To specify a filename when accessing an Azure Blob container with a SAS token, use one of the following formats:

### Using the Arbiter Azure Driver
First, export the necessary environment variables:
```sh
export AZURE_STORAGE_ACCOUNT=<your_storage_account>
export AZURE_SAS_TOKEN=<your_sas_token>
```
Then, set the `filename` as follows:
```json
{
    "filename": {
        "path": "az://<PATH_TO_EPT>/ept.json"
    }
}
```
**Note:** `PATH_TO_EPT` is simply the path to the container; there is no need to specify `blob.core.windows.net` or the Azure storage account.

### Using an HTTPS Link with Query Parameters
If using a direct HTTPS link, include the SAS token as query parameters:
```json
{
    "filename": {
        "path": "https://AZURE_STORAGE_ACCOUNT.blob.core.windows.net/PATH_TO_EPT/ept.json",
        "query":{
            "sp": "r",
            "st": "2024-03-03T17:30:06Z",
            "sig": "dQkX7R%2BXHrQLP9qiNdS0zMhYNpmQwLW0D86UUrEgGao%3D"
        }
    }
}
```
Each query parameter is separated by `&`, and the key-value pairs must be included accordingly.

For more details on SAS token structure, refer to this helpful guide: [SAS Tokens Decoded](https://medium.com/@anandchandrasekaran1996/sas-tokens-decoded-how-to-ensure-data-security-in-azure-blob-storage-efbcfef32f3f).


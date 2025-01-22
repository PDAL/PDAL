
In addition to a string filename, HTTP headers and query parameters to be forwarded to remote endpoints
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

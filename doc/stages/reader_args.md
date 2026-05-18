reader_args

: A list of JSON objects with keys of reader options and the values to pass through.
  These will be in the exact same form as a Pipeline Stage object, minus the filename.

  Exmaple:

```bash
--readers.stac.reader_args \
'[{"type": "readers.ept", "resolution": 100}, {"type": "readers.las", "nosrs": true}]'
```

  HTTP headers & queries can be forwarded to the reader using a partial {ref}`filespec`
  JSON object, omitting the `path` component:

```bash
--readers.stac.reader_args \
'{"type": "readers.copc", "filename": {"headers": {"your_header_key": "header_val"}, "query": {"your_query_key": "query_val"}}}'
```
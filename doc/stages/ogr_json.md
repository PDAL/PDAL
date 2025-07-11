
```json
{
    "type":"ogr",
    "datasource": "File path to OGR-readable geometry",
    "drivers": ["OGR driver to use", "and OGR KEY=VALUE driver options"],
    "openoptions": ["Options to pass to the OGR open function [optional]"],
    "layer": "OGR layer from which to fetch polygons [optional]",
    "sql": "SQL query to use to filter the polygons in the layer [optional]",
    "options":
    {
        "geometry": "WKT or GeoJSON geomtry used to filter query [optional]",
        "dialect": "SQL dialect to use (default: OGR SQL) [optional]"
    }
}
```

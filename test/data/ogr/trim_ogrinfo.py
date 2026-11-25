#!/usr/bin/env python3
"""Trim `ogrinfo -json` output to the parts that OGRWriterTest checks.

OGRWriterTest runs `ogrinfo -json -nomd -features` on each file that
writers.ogr writes, and checks it against the .json file of the same name
here. Only what that file holds is checked, so this keeps only what doesn't
change between machines and GDAL/PROJ versions:

- the driver's short name
- each layer's name, FID column, feature count and features
- each geometry field's name, type and extent, and the name of its SRS
  but not the rest of the SRS, whose WKT and PROJJSON change with PROJ
- each field's name, type, subtype, width and precision

Arrays of plain values go on one line to keep the files short.

To make or update an expected file, trim the output that the test saved
next to the file it wrote, in the test temp directory:

    python3 trim_ogrinfo.py < test/temp/ogr/NAME.json > NAME.json

or run ogrinfo on any file:

    ogrinfo -json -nomd -features FILE | python3 trim_ogrinfo.py > NAME.json
"""

import json
import sys

FIELD_KEYS = ("name", "type", "subType", "width", "precision")


def pick(obj, keys):
    """Return the keys of obj that it has, in GDAL's order."""
    return {k: v for k, v in obj.items() if k in keys}


def trim_geometry_field(geom):
    out = pick(geom, ("name", "type", "extent"))
    crs = geom.get("coordinateSystem")
    if crs is not None:
        crs = {"projjson": {"name": crs["projjson"]["name"]}}
    out["coordinateSystem"] = crs
    return out


def trim_layer(layer):
    out = pick(layer, ("name", "fidColumnName"))
    out["geometryFields"] = [trim_geometry_field(g)
                             for g in layer["geometryFields"]]
    out["featureCount"] = layer["featureCount"]
    out["fields"] = [pick(f, FIELD_KEYS) for f in layer["fields"]]
    if "features" in layer:
        out["features"] = layer["features"]
    return out


def dump(value, indent=0):
    """Format value as JSON with 2-space indents, but with arrays of plain
    values on one line."""
    pad = " " * indent
    if isinstance(value, dict) and value:
        items = [f"{pad}  {json.dumps(k)}: {dump(v, indent + 2)}"
                 for k, v in value.items()]
        return "{\n" + ",\n".join(items) + f"\n{pad}}}"
    if isinstance(value, list) and any(isinstance(v, (dict, list))
                                       for v in value):
        items = [f"{pad}  {dump(v, indent + 2)}" for v in value]
        return "[\n" + ",\n".join(items) + f"\n{pad}]"
    if isinstance(value, list):
        return "[" + ", ".join(json.dumps(v) for v in value) + "]"
    return json.dumps(value)


def main():
    info = json.load(sys.stdin)
    trimmed = {
        "driverShortName": info["driverShortName"],
        "layers": [trim_layer(layer) for layer in info["layers"]],
    }
    sys.stdout.write(dump(trimmed) + "\n")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Convert OGRWriterTest's stored ogrinfo output to its JSON form.

OGRWriterTest compares the text of `ogrinfo -nomd -al` for each file that
writers.ogr writes with a stored .ogrinfo file, so it breaks whenever GDAL
changes how ogrinfo prints things. The test is moving to JSON that it
writes from the GDAL API. This reads each .ogrinfo file and writes the
same facts in that form, next to it as .json:

- the layer's name, geometry type, feature count and extent
- the SRS name, or null for none; not the rest of the SRS, which changes
  with the PROJ version
- each field's name, type, width and precision
- each feature's FID, field values, geometry type and points, as
  [x, y, z] or [x, y, z, m]

Numbers are rounded to 6 decimal places, as the test rounds them, so the
last bits of a double don't change the text.

Usage: ogrinfo_to_json.py FILE.ogrinfo...
"""

import json
import re
import sys

# The OGC names of the geometry types, as OGRToOGCGeomType() gives them,
# for ogrinfo's names on its Geometry line and its WKT geometry prefixes
OGC_TYPES = {
    "3D Point": "PointZ",
    "3D Measured Point": "PointZM",
    "3D Multi Point": "MultiPointZ",
    "3D Measured Multi Point": "MultiPointZM",
}
WKT_TYPES = {
    "POINT Z": "PointZ",
    "POINT ZM": "PointZM",
    "MULTIPOINT Z": "MultiPointZ",
    "MULTIPOINT ZM": "MultiPointZM",
}


def rounded(text):
    return round(float(text) * 1e6) / 1e6


def value(ftype, text):
    if ftype == "Real":
        return rounded(text)
    if ftype in ("Integer", "Integer64"):
        return int(text)
    return text


def geometry(wkt):
    """The geometry type and points of a POINT or MULTIPOINT WKT."""
    m = re.fullmatch(r"((?:MULTI)?POINT Z?M?) \((.*)\)", wkt)
    if not m or m.group(1) not in WKT_TYPES:
        sys.exit(f"unexpected geometry: {wkt}")
    body = m.group(2)
    if body.startswith("("):
        body = body[1:-1]
    points = [[rounded(c) for c in p.split()] for p in body.split("),(")]
    return WKT_TYPES[m.group(1)], points


def convert(path):
    with open(path, encoding="utf-8") as f:
        lines = f.read().splitlines()

    info = {}
    fields = []
    features = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("Layer name: "):
            info["name"] = line[len("Layer name: "):]
        elif line.startswith("Geometry: "):
            info["geometryType"] = OGC_TYPES[line[len("Geometry: "):]]
        elif line.startswith("Feature Count: "):
            info["featureCount"] = int(line[len("Feature Count: "):])
        elif line.startswith("Extent: "):
            info["extent"] = [rounded(v) for v in
                              re.findall(r"-?\d+\.\d+", line)]
        elif line == "Layer SRS WKT:":
            i += 1
            m = re.match(r'\w+\["([^"]*)"', lines[i])
            info["srs"] = m.group(1) if m else None  # "(unknown)": none
        elif m := re.fullmatch(r"(\w+): (\w+) \((\d+)\.(\d+)\)", line):
            fields.append({"name": m.group(1), "type": m.group(2),
                           "width": int(m.group(3)),
                           "precision": int(m.group(4))})
        elif m := re.fullmatch(r"OGRFeature\(.*\):(\d+)", line):
            features.append({"fid": int(m.group(1)), "properties": {}})
        elif features and line.startswith("  "):
            feat = features[-1]
            m = re.fullmatch(r"  (\w+) \((\w+)\) = (.*)", line)
            if m:
                feat["properties"][m.group(1)] = value(m.group(2), m.group(3))
            else:
                feat["geometryType"], feat["points"] = geometry(line.strip())
        i += 1

    if len(features) != info["featureCount"]:
        sys.exit(f"{path}: {info['featureCount']} features, but "
                 f"{len(features)} printed")
    info["fields"] = fields
    info["features"] = features
    return info


for path in sys.argv[1:]:
    out = re.sub(r"\.ogrinfo$", ".json", path)
    with open(out, "w", encoding="utf-8") as f:
        f.write(json.dumps(convert(path), indent=2) + "\n")

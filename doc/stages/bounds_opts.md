  ```{note}

  Bounds can be expressed in multiple forms:

    * Classical PDAL style

      * 3D – `([xmin, xmax], [ymin, ymax], [zmin, zmax])`
      * 2D – `([xmin, xmax], [ymin, ymax])`

    * [GeoJSON BBOX](https://datatracker.ietf.org/doc/html/rfc7946#section-5)

      * 3D – `[xmin, ymin, zmin, xmax, ymax, zmax]`
      * 2D – `[xmin, ymin, xmax, ymax]`

    * JSON Object

      * 3D

      ```
      {
        "minx": 1,
        "miny": 2,
        "minz": 3,
        "maxx": 101,
        "maxy": 102,
        "maxz": 103
      }
      ```

      * 2D

      ```
      {
        "minx": 1,
        "miny": 2,
        "maxx": 101,
        "maxy": 102
      }
      ```

      * CRS

      The JSON Object style form is the only one that can also include a
      definition using the `crs` member. It can be any valid PROJ-compatible
      CRS definition (WKT, WKT2, PROJJSON, PROJ.4 codes).


      ```
      {
        "minx": 1,
        "miny": 2,
        "minz": 3,
        "maxx": 101,
        "maxy": 102,
        "maxz": 103,
        "crs": "EPSG:4326"
      }
      ```
  ```


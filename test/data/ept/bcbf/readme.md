# bcbf

This is a test dataset with some toy data.  Each populated node has one point
at 1, 1, 99 - near the north pole but slightly in the positive X/Y directions.

The node data can be created with something like:
```
node -e "require('fs') \
    .writeFileSync( \
        'point.bin', \
        (() => { \
            let b = Buffer.alloc(12); \
            b.writeInt32LE(1, 0); b.writeInt32LE(1, 4); b.writeInt32LE(999, 8); \
            return b; \
        })())"
```

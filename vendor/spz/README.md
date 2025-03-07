# spz

`.spz` is a file format for compresed 3D gaussian splats. This directory contains a C++ library
for saving and loading data in the .spz format.

spz encoded splats are typically around 10x smaller than the corresponding .ply files,
with minimal visual differences between the two.

## Implementations

### C++

Requires `libz` as the only dependent library, otherwise the code is completely self-contained. No
build system is provided (bring your own build).

## API

```
bool saveSpz(const GaussianCloud &gaussians, std::vector<uint8_t> *output);
```

Converts a cloud of Gaussians in `.spz` format to a vector of bytes.
   - `gaussians`: The Gaussians to save
   - `output`: A vector that will be populated with bytes encoded in .spz format
   - Returns true on success and false on failure.

---

```
bool saveSpz(const GaussianCloud &g, const std::string &filename);
```

Saves a cloud of Gaussians in `.spz` format to a file
   - `gaussians`: The Gaussians to save
   - `filename`: The path to the file to save to.
   - Returns true on success and false on failure.

---

```
GaussianCloud loadSpz(const std::vector<uint8_t> &data);
```

Loads a cloud of Gaussians from bytes in `.spz` format.
   - `data`: A vector containing the encoded spz data
   - Returns a `GaussianCloud` decoded from the vector. In case of an error, this will return
     a result with no gaussians

---

```
GaussianCloud loadSpz(const std::string &filename);
```

Loads a cloud of Gaussians from a file in `.spz` format.
   - `filename`: The path to the file to load from.
   - Returns a `GaussianCloud` decoded from the file. In case of an error, this will return
     a result with no gaussians


## File Format

The .spz format is a gzipped stream of data consisting of a 16-byte header followed by the
gaussian data. This data is organized by attribute in the following order: positions,
scales, rotations, alphas, colors, spherical harmonics.

### Header

```c
struct PackedGaussiansHeader {
  uint32_t magic;
  uint32_t version;
  uint32_t numPoints;
  uint8_t shDegree;
  uint8_t fractionalBits;
  uint8_t flags;
  uint8_t reserved;
};
```

All values are little-endian.

1. **magic**: This is always 0x5053474e
2. **version**: Currently, the only valid version is 2
3. **numPoints**: The number of gaussians
4. **shDegree**: The degree of spherical harmonics. This must be between 0 and 3 (inclusive).
5. **fractionalBits**: The number of bits used to store the fractional part of coordinates in
   the fixed-point encoding.
6. **flags**: A bit field containing flags.
   - `0x1`: whether the splat was trained with [antialiasing](https://niujinshuchong.github.io/mip-splatting/).
7. **reserved**: Reserved for future use. Must be 0.


### Positions

Positions are represented as `(x, y, z)` coordinates, each as a 24-bit fixed point signed integer.
The number of fractional bits is determined by the `fractionalBits` field in the header.

### Scales

Scales are represented as `(x, y, z)` components, each represented as an 8-bit log-encoded integer.

### Rotation

Rotations are represented as the `(x, y, z)` components of the normalized rotation quaternion. The
`w` component can be derived from the others and is not stored. Each components is encoded as an
8-bit signed integer.

### Alphas

Alphas are represented as 8-bit unsigned integers.

### Colors

Colors are stored as `(r, g, b)` values, where each color component is represented as an
unsigned 8-bit integer.

### Spherical Harmonics

Depending on the degree of spherical harmonics for the splat, this can contain 0 (for degree 0),
9 (for degree 1), 24 (for degree 2), or 45 (for degree 3) coefficients per gaussian.

The coefficients for a gaussian are organized such that the color channel is the inner (faster
varying) axis, and the coefficient is the outer (slower varying) axis, i.e. for degree 1,
the order of the 9 values is:
```
sh1n1_r, sh1n1_g, sh1n1_b, sh10_r, sh10_g, sh10_b, sh1p1_r, sh1p1_g, sh1p1_b
```

Each coefficient is represented as an 8-bit signed integer. Additional quantization can be performed
to attain a higher compression ratio. This library currently uses 5 bits of precision for degree 0
and 4 bits of precision for degrees 1 and 2, but this may be changed in the future without breaking
backwards compatibility.

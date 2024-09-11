(random-command)=

# random

:::{warning}
As of PDAL v2.6.0, the `random` command is marked as DEPRECATED. It will
be removed from the default install in PDAL v2.7 and removed completely in
PDAL v2.8.

A simply Python script that creates uniformly distributed data and writes
the output using PDAL is given below.

```
import numpy as np
import pdal

def generate_points_uniform(num_points, x_min, x_max, y_min, y_max, z_min, z_max):
    x_coords = np.random.uniform(x_min, x_max, num_points)
    y_coords = np.random.uniform(y_min, y_max, num_points)
    z_coords = np.random.uniform(z_min, z_max, num_points)

    dtype = [('X', '<f8'), ('Y', '<f8'), ('Z', '<f8')]
    points = np.zeros(num_points, dtype=dtype)

    points['X'] = x_coords
    points['Y'] = y_coords
    points['Z'] = z_coords

    return points

# Configuration for uniform distribution
num_points_uniform = 100
x_min, x_max = 0, 10
y_min, y_max = 0, 10
z_min, z_max = 0, 10

# Generate points using uniform distribution
points_uniform = generate_points_uniform(num_points_uniform, x_min, x_max, y_min, y_max, z_min, z_max)

pipeline = pdal.Writer("output_uniform.laz").pipeline(points_uniform)
pipeline.execute()
```

A similar Python script that creates normally distributed data and writes
the output using PDAL is given below.

```
import numpy
import pdal

def generate_points_normal(num_points, x_mean, x_std, y_mean, y_std, z_mean, z_std):
    x_coords = np.random.normal(x_mean, x_std, num_points)
    y_coords = np.random.normal(y_mean, y_std, num_points)
    z_coords = np.random.normal(z_mean, z_std, num_points)

    dtype = [('X', '<f8'), ('Y', '<f8'), ('Z', '<f8')]
    points = np.zeros(num_points, dtype=dtype)

    points['X'] = x_coords
    points['Y'] = y_coords
    points['Z'] = z_coords

    return points

# Configuration for normal distribution
num_points_normal = 100
x_mean, x_std = 5, 1
y_mean, y_std = 5, 1
z_mean, z_std = 5, 1

# Generate points using normal distribution
points_normal = generate_points_normal(num_points_normal, x_mean, x_std, y_mean, y_std, z_mean, z_std)

pipeline = pdal.Writer("output_normal.laz").pipeline(points_normal)
pipeline.execute()
```
:::

The `random` command is used to create a random point cloud. It uses
{ref}`readers.faux` to create a point cloud containing `count` points
drawn randomly from either a uniform or normal distribution. For the uniform
distribution, the bounds can be specified (they default to a unit cube). For
the normal distribution, the mean and standard deviation can both be set for
each of the x, y, and z dimensions.

```
$ pdal random <output>
```

```
--output, -o       Output file name
--compress, -z     Compress output data (if supported by output format)
--count            How many points should we write?
--bounds           Extent (in XYZ to clip output to)
--mean             A comma-separated or quoted, space-separated list of means
    (normal mode): --mean 0.0,0.0,0.0 --mean "0.0 0.0 0.0"
--stdev            A comma-separated or quoted, space-separated list of
    standard deviations (normal mode): --stdev 0.0,0.0,0.0 --stdev "0.0 0.0 0.0"
--distribution     Distribution (uniform / normal)
```

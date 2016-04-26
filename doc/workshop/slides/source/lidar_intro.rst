.. _lidar_intro:

Introduction to LiDAR
================================================================================

.. include:: ../../includes/substitutions.rst

- Active remote sensing with visible or near-infrared lasers to measure distances
- Can be mounted on tripods, cars, planes, helicopters, drones, satellites, and more

.. figure:: img/google-car.jpg
    :width: 400px

How do LiDAR systems measure distance?
================================================================================

Three primary methods:

1. **By measuring the time-of-flight of laser pulses.**
2. By measuring the time-of-flight of single photons.
3. By using interferometry to count the fractional number of wavelengths between a scanner and a target.

.. figure:: img/scanner.svg
    :height: 200px


How is LiDAR used?
================================================================================

* Terrestrial Laser Scanning (**TLS**)
* Mobile Laser Scanning (**MLS**)
* Airborne Laser Scanning (**ALS**): also called Airborne Laser Swath Mapping (ALSM)
* Unmanned Laser Scanning (**ULS**)

Discrete vs. full-waveform
============================

- Outgoing puses are *not* instantaneous — they have a finite width and height (approximated by a Gaussian)
- Return energy is not instantaneous either (see picture)
- The "full waveform" output (see picture) is usually simplified down to single points (discrete-return)

.. figure:: ../../images/return-pulse.png
    :height: 250px

Georeferencing
================================================================================

* LiDAR data is usually only useful if placed in a larger project or global context.
* "Registering" LiDAR data to a global coordinate system is called **georeferencing**.
* Georeferencing static (TLS) scans is usually done via known control points, either benchmarks or GNSS survey points.
* Georeferencing mobile or airborne (MLS/TLS) scans is done by combining LiDAR data with position and orientation information from a GNSS/IMU mounted alongside the LiDAR scanner.

How is LiDAR processed?
================================================================================

* Vendor-specific software (e.g. RiSCAN Pro and RiProcess, from |Riegl|)
* Other commercial softwares (e.g. TerraStation, QT Modeler)
* Mixed-source software (e.g. LasTools)
* Open-source software (e.g. CloudCompare, PDAL, laspy)


Next
================================================================================

On to :ref:`pdal_intro`

# velodyne_base
some basic functions for velodyne 64E and 32E, including driver, dem, etc.


## Autonomous cars projects
Basic code for lidar data processing. Generating 2D grid maps for nevigation.

The lidar driver is a optimized version of the [ros package driver](http://wiki.ros.org/velodyne_driver)， but with new features:

1. the determing angle, which defines a specific frame, saying a new circle frame starts and ends from this angle.
2. parallelly grabing pcap packages and transforming raw data into point clouds.

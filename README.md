# Real-Time-Embedded-Systems
This repository contains code for the real-time embedded systems class project. The project resembles a goalkeeper but in the virtual world that tries to block objects in the real world

**Hardware** NVIDIA Jetson TK1 and Logitech webcam

**Software** C, OpenCV4Tegra

**Description**

1. The virtual goalkeeper has four different services.
1. The first service is responsible for image capture using the Logitech webcam
1. The second service uses image processing techniques (contours) to determine the moving object and calculate its centroid.
1. The centroid co-ordinates are passed on to the next service which uses this to move align the goalkeeper in the right position
1. The fourth service(best effort system) captures the camera output and saves it to disk
1. The system uses OpenCV4Tegra to provide a performance boost
1. The first three services (soft real time) were run on a single core and the video writing task was offloaded to a separate core
1. Jitter calculations were made and the system was found to have a 2% deadline miss for a sample size of 3000
1. Semaphores and mutexes were used to synchronize access to global variables and shared resources
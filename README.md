# aruco_pkg
A repository for creating ArucoDetect, a set of models that assists ARCS in finding a physical marker and scanning it once close enough

## Goals
* The rover will be able to utilize a set of models that can detect and scan Aruco images, in conjunction with the ARCS, to detect when the rover has arrived at a destination. 

* ArucoDetect will determine the relative position of an Aruco marker to assist the ARCS with getting closer to the marker. 

## Methodology
These are the strategies and technologies we will use to achieve the above goals  

* ArucoDetect is a set of CV models that will be used in the autonomous navigation challenge utilizing physical ARUCO markers. 

* We will research, develop, and deploy a set of CV models, pretrained and/or built and trained in-house, to the rover’s NVIDIA Jetson TX2 as a part of the ARCS. 

* ArucoDetect will interface with other portions of the ARCS to provide additional information about the position of a physical marker. 

* ArucoDetect will do this through utilizing CV models to analyze frames from the rover’s camera to detect a physical marker and determine the relative angle between the rover’s heading and the position of the marker. 

* ARCS will update its route as ArucoDetect updates the heading difference as the rover approaches the marker. 

* Once in range, ArucoDetect will scan the marker to indicate to ARCS that the rover has arrived at the position. 

## Deliverables
These are the products we will create as a result of achieving these goals.
* A set of models that detects, finds the relative heading of, and scans Aruco markers, called ArucoDetect. 

* These models will be able to find the heading of the marker relative to the rover’s current heading to assist the ARCS in navigating to a position where the marker can be scanned, and the rover has arrived at the destination. 

## References
* Detection of ArUco Markers (Like the navigation tasks): https://docs.opencv.org/3.4/d5/dae/tutorial_aruco_detection.html

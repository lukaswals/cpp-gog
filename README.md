# cpp-gog

C++ implementation of the GOG Tracker. It can be used on it's own, or plugged into the [UA-DETRAC](https://detrac-db.rit.albany.edu/) toolkit for evaluation.
For more information about the tracker, please refer to the the following links:
* [Original Implementation](https://github.com/mprat/meng-work/tree/master/MATLAB/%2Btracking_cvpr11_release_v1_0) - Matlab
* [Paper](http://ieeexplore.ieee.org/document/5995604/) - "Globally-optimal greedy algorithms for tracking a variable number of objects"

## Supported OS
* Windows

## Requeriments
There's only one requeriment at the moment
* opencv - For viewing and saving tracking results on image file. Tested with version >= 3

## Usage
The tracker can receive the input of any of the provided detections in UA-DETRAC (CompACT, R-CNN, ACF, DPM), and also from YOLO detector. The command is as follow:
```
cppGOG <sequence> <input_folder> <detector_type> <detection_file> <output_folder>
```
where <detector_type> can take the values "DETRAC" or "YOLO"

## Authors of Original Implementation
* Hamed Pirsiavash
* Deva Ramanan
* Charless C. Fowlkes

Multi Camera Calibration Suite
==============================

This toolset provides the basics for calibrating a multi-camera scene. it contains six utilities for
different purposes. In this README I will walk the user through the calibration of a multi camera
scene using this toolset.

Dependencies
------------

the use of this suite requires

- [ceres-solver](http://ceres-solver.org/)
- [google logging suite](https://code.google.com/archive/p/google-glog/)
- [google flags](https://gflags.github.io/gflags/)
- [OpenCV](http://opencv.org/)
- [numpy](http://www.numpy.org/)

Getting the source
------------------

clone the repository using :

```git clone git@github.com:idiap/multicamera-calibration.git```

Building the source
-------------------

Go to the source directory

Do

```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

Compute intrinsic camera parameters
-----------------------------------

The intrinsic camera parameters are usually computed finding a known grid with multiple poses on
different images. To do so we propose the following method.

Capture a video by waving the camera over a grid such as 
[this one](http://docs.opencv.org/2.4/_downloads/acircles_pattern.png) or 
[that one](http://docs.opencv.org/2.4/_downloads/pattern.png). Extract then the frames as images in
a folder then run the software ```bin\intrinsic```

```bash
intrinsic [-h] [-n NUM] [-W WIDTH] [-H HEIGHT]
          [-p {chessboard,circles,asymmetric_circles}] [-o FULL_OUTPUT]
          [-s SELECT_OUTPUT]
          input output

          -h:     show help
          -n:     number of frames to use
          -w:     width of grid
          -h:     height of grid
          -p:     type of grid
          -o:     output for all frames with a visible grid (optionnal)
          -s:     output for all n frames (option -n) selected for the calibration computation (optionnal)
          input:  pattern for the images
          output: calibration file (json format)
```

For example: let's imagine we have the frames in the folder /home/user1/camera1/frames with the asymmetric
circles grid provided earlier.

you would use the script this way
```bash
bin/intrinsic -o /home/user1/frames_with_grid -s /home/user1/camera1/selected_frames_with_grid /home/user1/camera1/frames/\*.bmp /home/user1/camera1/intrinsic.json
```

once the computation is done, the undistorted frames are shown.
  - 'n' goes to next frame
  - 'p' goes to previous frame
  - 'q' quits

this produces the folders with the usable frames and the selected frames so that reproducing the
calibration takes less time.

Compute extrinsic camera parameters
-----------------------------------

The extrinsic parameters are computed by clicking on points with known coordinates (in cm) in an
image. The syntax is the following.
```bash
extrinsic [-h] [-p POINTS] [-o OUTPUT_POINTS] intrinsic input output

          -h:        show help
          -p:        saved points file, used to resume or correct the calibration (optionnal)
          -o:        output point file. this file will be used with the -p option (optionnal)
          intrinsic: intrinsic camera parameters
          input:     image to annotate
          output:    calibration output
```

Now using the intrinsic.json file we computed on the previous step, do.
```bash
bin/extrinsic /home/user1/camera1/intrinsic.json /home/user1/camera1/video_frames/000000.bmp /home/user1/camera1/extrinsic.json
```

it shows an the frame 000000.bmp you can left click on a point to add it. it will ask for x and y in
cm. right click removes last point.

  - 'q' quits without saving
  - 's' computes the extrinsics and saves

Verify the 3d projections in the scene
--------------------------------------

Now both the intrinsic and extrinsic calibrations have been computed for cameras 1 to 4
to check the correspondance between the cameras, you can use the following utility

```bash
check3d [-h] intrinsic extrinsic images cameras
        -h: help
        intrinsic: pattern to intrinsic calibrations files
        extrinsic: pattern to extrinsic calibrations files
        images:    pattern to camera images (same frame for each camera)
        cameras:   camera list comma separated
```

using all our cameras calibrated, we would go to something like that

```bash
bin/check3d /home/user1/{}/intrinsic.json /home/user1/{}/extrinsic.json /home/user1/{}/video_frames/000000.bmp camera1,camera2,camera3,camera4
```

once the soft launched you get an image from the frame list.
you can click on a point in some views, it will appear as a blue dot.
once you clicked at least two views you get a red dot representing the
projection on the other views.
left click removes the blue dot.
if you clicked wrong, a right click moves the blue dot to the new location.

  - pressing 'n' goes to the next image
  - pressing 'p' goes to the previous image
  - pressing 'r' resets the points
  - pressing 'q' quits the soft

Perform the bundle adjustment
-----------------------------

to perform the bundle adjustment you need to provide annotations in the .pos format
the pos format has one file per frame such as /home/user1/camera1/annotations/000000.pos is the
annotation for the /home/user1/camera1/video_frames/000000.bmp image.

the pos format is done as such:
```
identity_name_1
0 x_val y_val
1 x_val y_val

identity_name_2
0 x_val y_val
1 x_val y_val
```

point number 0 corresponds to feet, 1 to head.
point numbers can be sparse and identities don't have to exist on each frame

once some frames are annotated, run

```bash
build_ba [-h] [-f FIXED_POINTS]
         intrinsic extrinsic observations cameras output

         -h:           help
         -f:           pattern to known points that must remain at the same place 
                       i.e. points outputed by the -o option in extrinsic calibration (optionnal)
         intrinsic:    pattern to intrinsic calibrations files
         extrinsic:    pattern to extrinsic calibrations files
         observations: pattern to pos files
         cameras:      camera list comma separated
         output:       bundle adjustment problem file
```

```bash
bin/build_ba -f /home/user1/{}/points.json /home/user1/{}/intrinsic.json /home/user1/{}/extrinsic.json /home/user1/{}/annotations/\*.pos /home/user1/ba_problem.txt
```

then run the bundle adjustment
```bash
bundle_adjuster --input=input --output=output --intrinsic_adjustment=[fixed, unconstrained, two_pass]
                --input:                bundle adjustment problem file
                --output:               bundle adjustment problem file, post processing
                --intrinsic_adjustment: type of adjustment to perform
```

there are three types of adjustment
  - fixed: the intrinsics are fixed, only perform on the extrinsics
  - unconstrained: adjust all parameters
  - two_pass: run once fixed and then unconstrained

using our example, run
```bash
bin/bundle_adjuster --input=/home/user1/ba_problem.txt --output=/home/user1/processed_ba.txt
```

then we need to convert back the bundle adjustment problem file to camera calibrations

```bash
extract_ba [-h] input intrinsic extrinsic cameras
           -h:        help
           input:     bundle adjustment problem file
           intrinsic: output to intrinsic calibration file
           extrinsic: output to extrinsic calibration file
           cameras:   camera list comma separated
```

In our example it translates to
```bash
bin/extract_ba /home/user1/processed_ba.txt /home/user1/{}/intrinsic_ba.json /home/user1/{}/extrinsic_ba.json camera1,camera2,camera3,camera4
```

then you may check again the projections in 3d like seen previously
```bash
bin/check3d /home/user1/{}/intrinsic_ba.json /home/user1/{}/extrinsic_ba.json /home/user1/{}/video_frames/000000.bmp camera1,camera2,camera3,camera4
```

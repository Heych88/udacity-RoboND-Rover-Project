## Project: Search and Sample Return

---

**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, colour threshold, etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./output/camera_image.jpg
[image2]: ./output/hsv_camera_image.jpg
[image3]: ./output/rock_img.jpg
[image4]: ./output/path_threshed.jpg
[image5]: ./output/rock_threshed.jpg
[image6]: ./output/wall_threshed.jpg
[image7]: ./output/grid_example.jpg
[image8]: ./output/warped_example.jpg
[image9]: ./output/warped_threshed.jpg
[image10]: ./output/Screenshot_3.png
[image11]: ./output/Screenshot_1.png
[image12]: ./output/Screenshot_2.png
[video1]: ./output/test_mapping.mp4


## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your write-up as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for colour selection of obstacles and rock samples.
This section will outline the process behind the testing of the input image data into a form that is used to locate a robot within the world map and the position of objects, navigatable paths and obstacles within the field of view. The code described here is in the accompanying Jupyter Notebook `Rover_Project_Test_Notebook.ipynb`.

An example of the input image from the simulation camera can be seen below.

![Rover camera image][image1]

To perform localisation and object detection from the camera image, the image is converted from the RGB colour scale to HSV format for path detection and sample object detection. However, obstacle and wall detection are performed in the RGB format. See below.

![Rover HSV camera image][image2]

 The HSV image is then past through the threshold function `color_thresh` located in section **Color Thresholding**, code block 6 of the notebook. This enables unneeded data to be eliminated from the image. This step is called with three different inputs and thresholds to locate sample objects, paths and obstacles. The below images show the thresholding of the navigable rover path, sample rocks to locate and walls and obstacles. NOTE: the desired outputs are the white area.

![input image][image3]
![navigatable path][image4]
![sample rocks to be found][image5]
![walls and obstacles][image6]

The final step for image processing is a Perspective Transform of the images to create a top-down view of the world for calculating positions of objects in the image. This is performed in the section title **Perspective Transform**, code block 5. The source and destination points were chosen by overlaying a 1m x 1m grid onto the world surface, locating the rover in front of a grid and finding the four corners of the grid square. See below.

![gridded world][image7]
![Perspective Transformed grid][image8]
![Perspective Transformed of thresholding the path][image9]

 NOTE: images shown below are for visualisation purposes only. The true Perspective Transform will be of the thresholding images described above.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples on a world map.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.
This section describes how the positional data is gained from the transformed camera images. The code is in code blocks 9 and 11 of the notebook.

The steps performed in `process_image()` are as follows.

1. Define source and destination points for perspective transform.
2. Convert image to HSV colour space
3. Apply thresholding
4. Apply perspective transform
5. Convert image pixel values to rover-centric coordinates.
  - Calculates the pixels between the rover base image to the objects, paths and obstacles.
6. Convert rover-centric pixel values to world coordinates.
  - Calculates the rover position with respect to the world coordinates enabling the objects and obstacles to be positioned on the world map.
7. Updates the world coordinates.

For more information read through the notebook section **Coordinate Transformations**, code block 9 and 11.

An example video `test_mapping.mp4` is supplied the folder `output` and below.

![Rover video][video1]

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and explanation are provided in the write-up of how and why these functions were modified as they were.

The code outlined above was placed into the file `perception.py` in the folder code. The rover controller and states update code in located in the file `decision.py`.

The code states are set out in a decision tree in the process order. See the code  function `decision_step()`, lines 148 to 180 in the file `decision.py` for more details.
1. Samples rocks found
  - navigate towards the sample and pick up if close.
2. Turn around
  - There is no way forward; the rover will stop and turn around.
3. Move forward
  - explores the map until one of the steps above is triggered.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your write-up.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your write-up when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I will improve it if I were going to pursue this project further.  

At the present time of writing the rover only explores navigatable paths using the calculated mean angle of the perspective transformed path. This then feeds into the `decision_step()`, as outlined above. This was found to be the most reliable at this stage, although other methods, as outlined below were tested but require extra work. The rover does not keep track of where it has already been or where it has yet to discover. In spite of this, the rover can still navigate the world map and explore above 95% of the area unaided. See image below.

![99% map exploration][image10]

During the testing of code control, two other methods were tested but at this stage require more time and testing to get functional on the rover.
1. An A* search and a Dynamic programming search algorithms.
  - A*
      - Fails due to a lack of a kinematic model of the rover and which requires the algorithm to compute every time the rover is in an uncomputed grid.
  - Dynamic
      - The basic algorithm runs the rover next to walls and objects producing collisions.
      - Stochastic algorithm Calculates odd angles causing the rover to perform unproductive actions.
2. Convolution Neral-Net for control.
  - At present more training and hyperparameter tuning are required.

### Problems and shortcomings
Future implementation of the above will help prevent the current problems observed by the software.

- Due to the steer angles being calculated using the mean angle of the observed path, when the rover finds itself facing an obstacle with equal distance either side of the object, it will move straight and crash.
- When a sample is detected, the code in the current state will not navigate around obstacles and therefore can crash.
- The rover only explores new areas if by chance it is navigating in a particular direction. This can lead to the rover exploring a very small area repeatedly for long times.
- if a path is not less than the `mim_wall_distance`, set in the file `drive_rover.py`, it will think the path is navigatable and travel forward which can prevent the rover from turning around causing crashes when objects are present ahead of the rover.
- When the rover stops or bounces around on the surface, it causes the camera image to wobble creating perspective transform images to both under and over estimate distances and objects causing sporadic behaviour. This is mainly observed when the rover is facing an object and needs to stop immediately. This has been minimised by slowing the rover down, implementing a PID speed controller and braking rather than stopping.

Below are some screen shots of the rover in action.

![Autonomous rover][image11]

![Autonomous rover][image12]

# Robot Soccer

Anna Buchele and Nina Tchirkova

## Blog 1
### 11/30/2018

We started our work for this project by scaffolding it into many parts and coming up with an minimum viable product. Our minimum viable product is to have a robot recognize a ball and kick it in a certain specified direction. To do this, we need to be able to recognize a ball, determine what angle and distance it is from the robot, and then approach it at a specified angle. So far we are able to recognize a ball. However, the angle we detect it is from us is not completely accurate so then we approach it at the wrong angle as well. Our stretch goal for this project is to also detect a goal and have the robot “kick” the ball into the goal. This involves determining at what angle the goal is from the robot and then kicking the ball in that direction.

Our first goal for this project was to successfully detect the presence of a ball. We used a green and black soccer ball, [PHOTO] for easy differentiation from the outside environment. To detect it, we used code from a prior similar project, which implemented a simple OpenCV color mask with blob detection. This did a pretty good job of detecting and localizing the ball, but it didn’t work in some situations. Occasionally the detector would find a wall, or a window, and decide that it was the ball. With some more testing and visualization, we re-tweaked the mask upper and lower bounds to more closely match the color of the ball, and this appears to have solved the issue. The bounding box is smaller and more accurately aligned to the ball as well, which is a nice bonus. Our mask and an image of the found ball can be seen below. [PHOTO]

The next goal for this project was to drive towards a found ball. This requires two things: successfully localizing the correct angle of the ball, and correctly going to a prescribed angle and distance. Prior implementations on our robots had used time and speed-based movement tracking (e.g. we are moving at 0.5 meters per second, and we moved for 1 second, so therefore we have moved 0.5 meters), but we weren’t sure that this method would be quite accurate enough for a precise angular measurement. So, we implemented movement tracking using odometry information, and found that to be very accurate. We had a small bug with going to a prescribed angle, as the signs on the image angle (left of image being a negative angle, and right of image being a positive angle) were the opposite of the angle signs of our robot. However, once we found this issue, fixing it was very simple. We then found another problem, which we are still working on: the angle found for the direction of the ball does not match up with the actual angle the robot should drive in order to get there. We think this is due to not calibrating the camera, so that will be our next step.

## Blog 2
### 12/7/2018

Like everything ever in engineering, calibration turned out to be harder than expected. Calibration is necessary because from an image, the program needs to accurately determine the angle away an object is and its distance away too. For these values to be determined accurately, it is necessary to know the focal length, and the focal length is determined by calibrating the camera. After trying many methods we found a ROS package that helps with calibrating a camera by providing a GUI (see below). By giving the actual size of the checkerboard to the program, it was able to get the necessary calibration values by having the checkerboard be in different locations and orientations.

![](/pics/nina.png)

Once the program was done, it outputted a k matrix which contained the focal length. If given the coordinates of a pixel, there is a distance X of how far left or right that pixel is, a distance Y of how far up or down that pixel is, and a distance Z of how far forward the pixel. (See image below).

For this project we only really care about X and Z because the robot stays at a constant height. Given the actual radius of the ball in meters, the width in pixels and the focal length, it is possible to calculate Z. Z = 2 * focal length * radius / width. Then to calculate X, the x coordinate of the pixel of the center of the ball is needed as well as the x offset (Cx) which is given in the K matrix. The equation is X = 2(xp - Cx)/focal length. X and Z could also be used to calculate the angle of the ball to the camera using arctan2. 

Another improvement we made was to the recognition of the ball in the image. Our previous version used a mask and blob detector to find the green ball, which usually worked, but the bounds of the ball found were less accurate than we felt they could be. In addition, using blob detection meant that our system would mistake any green object for the ball if the ball was not in the frame, or mistake larger green objects for the ball even if the ball was in frame. So, we combined the blob detector with a circle detector. Our system now finds all the circles in the image, and then checks to see which of those circles is in approximately the same location as a green blob. This helps filter out green objects which are not balls. We also more finely tuned the thresholds in both the blob detector and the circle detector to more closely match the color and size of the ball.

![](/pics/mask_calibration.png)

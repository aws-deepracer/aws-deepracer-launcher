# AWS DeepRacer Application - Modes of Operation

## Overview

The AWS DeepRacer core application on the DeepRacer device operates in three 
modes - **autonomous mode**, **calibration mode** and **manual mode**. The 
details about how the DeepRacer core application works in autonomous and manual 
modes is explained in detail below. You can follow the instructions explained in 
Getting Started with DeepRacer OpenSource to set up your car to the latest 
version of the DeepRacer device software to use the autonomous, calibration and 
manual modes of the DeepRacer core application.

The autonomous and manual modes are accessible in the **Control vehicle** page in the DeepRacer device console. In the autonomous mode, you can load the reinforcement learning model trained on the AWS DeepRacer service and run inference on the physical car. In the manual mode, you can manually control the car to steer and move it using a simple joystick in the device console. You can also change the speed using the device console to control the speed at which the car is driving in both the modes.

![Control vehicle console](/media/modes-control-vehicle.png)

Both the autonomous and manual modes leverage the **servo node** as their navigation component. You can read about how the various ROS nodes that are part of the AWS DeepRacer device software and the device console interact in the autonomous and manual mode below.

## Autonomous mode

While running the inference on the reinforcement learning model loaded in 
*Autonomous* mode, the DeepRacer device takes in the state information required 
at the rate of camera sensor and takes an action to either increase/decrease the 
speed and steering angle. Each **perception-inference-action** step involves a 
pipeline of a series of ROS messages published/subscribed at various nodes, from 
the point the camera and LiDAR data is published, and then a model takes image/
image+LiDAR data as an input to the point where the servo/motors attached to the 
wheel change angle/speed.

![Autonomous state](/media/modes-autonomous-state.png)

### Perception
As part of the perception step, the camera images are read from the camera and 
published in the **camera_node**. If you have a single camera connected, then the 
camera messages will contain only one image per message, whereas if there are two 
cameras connected (as in the DeepRacer Evo), the camera messages will contain one 
image from each camera per message. In the DeepRacer Evo, we also publish the 
LiDAR data read from the connected LiDAR. These camera and LiDAR data are 
combined together as sensor message and published at the rate of camera sensor in 
the **sensor_fusion_node**.

### Inference (Decision)
The reinforcement learning model used in the DeepRacer device is trained in the 
AWS DeepRacer service and uploaded to the device. Before running the inference, 
the reinforcement learning model is optimized on the device using the Intel 
OpenVino Model Optimizer in the **model_optimizer_node** to create the 
intermediate representation artifacts of the network, which can be read, loaded 
and inferred with the Intel OpenVino Inference Engine. As part of the inference 
step, the AWS DeepRacer application takes in the state information sent from the 
**sensor_fusion_node** and runs inference using the Inference Engine APIs in the 
**inference_node** to publish the reinforcement learning inference results.

### Action (Navigation)

![Navigation](/media/modes-autonomous-navigation.png)

The action step differs slightly based on the action space type that the model 
was trained on. The AWS DeepRacer service support two types of action space types 
for the models that are trained: **discrete** and **continuous**. For the 
discrete action space, a set of discrete values are returned from the neural 
network, that is interpreted as a probability distribution and are mapped to a 
set of actions. For continuous action space, the policy only outputs two discrete 
values. These values are interpreted to be the mean and standard deviation of a 
continuous normal distribution. You can learn more about these in the *“The ins 
and outs of action spaces”* section of this blog (https://aws.amazon.com/blogs/machine-learning/using-the-aws-deepracer-new-soft-actor-critic-algorithm-with-continuous-action-spaces/). 

As part of the action step, the results of the inference published by the 
**inference_node** is read, mapped and scaled to steering and speed values in the 
**deepracer_navigation_node**. These scaled values which is then published as 
servo messages, are converted to raw PWM duty cycle values that are centered 
around the mid, max and min values set during calibration setup in the 
**servo_node**. 

### Difference between discrete action space and continuous action space

The difference between the discrete and continuous action space is primarily in 
the way the neural network is architected to output the values. 

In the discrete action values, we get a probability distribution for a predefined 
set of actions each containing a fixed {steering_angle, speed} combination. Of 
which, we choose the maximum probable {steering_angle, speed} combination values 
for scaling.

In the continuous action values, we get a numerical mean value clipped between 
[-1.0, 1.0] which is rescaled linearly to the [minimum_value, maximum_value] for 
speed and steering_angle. 

At this point, the steering value is scaled based on its 
maximum value and the speed value is non linearly 
scaled to the range of [0.0, 1.0]. More details about this non linear scaling is 
explained in the "Non linear speed mapping equations” section below.

## Manual mode

In the manual mode, the car can be driven manually using a joystick on a 
rectangle track pad in the device side console. Dragging a joystick up towards 
the top of screen is considered increase in the speed, down to decrease the 
speed, right to turn right and left to turn left. 

![Manual drive](/media/modes-manual-drive.png)

The actual values are calculated as a difference between the top left coordinate 
of the rectangle track pad and the coordinate of joystick itself. The x and y 
values of these coordinates are calculated from the top left corner of the 
displayed screen on the browser. These raw joystick coordinates are then shifted 
to have the origin at the center of joystick at rest position and the (x, y) 
values are scaled to percentages. These raw values between [-1.0, 1.0] are then 
passed to the backend web server where we categorize and map the speed non 
linearly and categorize the angle.

* **Categorize the throttle and angle values** - This allows us to convert the 
joystick track pad into concentric rectangles of stepped values, allowing a 
better feedback and control for the user to move the joystick.

* **Use the maximum speed percentage and throttle as an input to calculate the 
non linear mapping** - This would allow to flatten the curve if the maximum speed 
percentage is lower, thereby reducing the impact of the raw joystick value on the 
final value. More details about this non linear mapping is explained in the 
section below.

These rescaled values are then finally mapped to the PWM values of the servo and motor.

![Manual drive flow](/media/modes-manual-flow.png)

Distinctions between autonomous mode action flow and manual mode action flow:

* Both steering angle and speed values are already in the range of [-1.0, 1.0] 
and there is no minimum and maximum speed associated with this. Autonomous mode 
has an additional mapping required to use the values passed in model_metatdata.
json.

* The maximum speed value is used to non linearly scale the speed unlike in autonomous mode, where maximum throttle value is used to get a fraction of the total value calculated.

### Non linear speed mapping equations

In the AWS DeepRacer device, we do not have a mechanism to know the actual speed 
of the car and need to depend only on the raw PWM values to control the speed. 
These PWM values have non linear mapping to the rpm of the wheel which makes the 
car go faster/slower. In order to connect the magnitude of speed to the actual 
rpm on the car, we use a non linear mapping of the input speed values as a proxy 
of the PWM values responsible for increasing speed. And one of the important 
functions in the DeepRacer device side software is a method that maps the action 
space value for speed into this transformed car speed value. 

The way it is implemented is by mapping the [maximum speed, half of the maximum 
speed] from the action space to the values of [1.0, 0.8] using the quadratic 
equation: 

> y = ax\*\*2 + bx

The equation corresponds to a parabola. In our use case, we have to find a parabola that will contain both the values [maximum_speed, 1.0] and [maximum_speed/2, 0.8] on it.  

**Why do we need to fit [maximum_speed, 1.0] and [maximum_speed/2, 0.8] on the ax\*\*2 + bx curve?** 

The value 1.0 and 0.8 correspond to the % of speed that needs 
to be non-linearly mapped to maximum speed and the half of maximum speed values 
that are passed as part of the model_metadata.json file(in autonomous mode). It 
implies that for a value of half of maximum speed in the model_metadata.json, we 
will consider 80% of PWM value. The mapping values selected have been empirically 
tested to provide a close and consistent mapping under various vehicle battery 
levels, devices and testing scenarios.

To begin understanding this in detail, let us consider the the curves below:

![Speed curves](/media/modes-equations-speed.png)

These show the mapping of different [maximum_speed, maximum_speed/2] values in 
model_metadata to the [1.0, 0.8] value on the y axis. It is important to note that the 
these values also pass through [0, 0] indicating that there is no constant value ‘c’ in 
our parabola expression ax\*\*2 + bx + c.

### Building some intuition around the mapping curves:

Whenever we consider the effects of coefficients a, b and c on the parabolic 
graph, we can observe that:

1. Changing the value of “a” changes the width of the opening of the parabola and that 
the sign of “a” determines whether the parabola opens upwards or downwards. 
1. Changing the value of “b” will move the axis of symmetry of the parabola from side 
to side; increasing b will move the axis in the opposite direction. 
1. Changing the value of “c” will move the vertex of the parabola up or down and “c” is 
always the value of the y-intercept.

We have already seen that the value of “c” in our expression is set to 0.

### Finding the values of coefficients “a” and “b”:

Let us consider the equation of the parabola:

> y = ax\*\*2 + bx

In our code, we notify the anchor values used for mapping as DEFAULT_SPEED_SCALES 
= [1.0, 0.8]. With this notation we can find the value of coefficients “a” and 
“b” by solving the above equation for two points: ([maximum_speed, 
DEFAULT_SPEED_SCALES[0]) and ([maximum_speed/2, DEFAULT_SPEED_SCALES[1])

Replace the value of x and y in the parabola equation:

> DEFAULT_SPEED_SCALES[0] = a * maximum_speed\*\*2 + b * maximum_speed

> DEFAULT_SPEED_SCALES[1] = (a * maximum_speed\*\*2 ) / 4 + (b * maximum_speed) / 2

Solving for “a” and “b”:

> 4 * DEFAULT_SPEED_SCALES[1] = (a * maximum_speed\*\*2 ) + 2 * (b * maximum_speed)

> b = (1  / maximum_speed) * ( 4 * DEFAULT_SPEED_SCALES[1]  - DEFAULT_SPEED_SCALES[0])

> b = (1  / maximum_speed) * 2.2

Replacing b in eq 3:

> DEFAULT_SPEED_SCALES[1] = (a * maximum_speed\*\*2 ) / 4 + ( 4 * DEFAULT_SPEED_SCALES[1]  - DEFAULT_SPEED_SCALES[0] ) *  maximum_speed/ (2 * maximum_speed)

> 4 * DEFAULT_SPEED_SCALES[1] = (a * maximum_speed\*\*2 ) + 2 * ( 4 * DEFAULT_SPEED_SCALES[1]  - DEFAULT_SPEED_SCALES[0])

> a = (4 * DEFAULT_SPEED_SCALES[1] - 8 * DEFAULT_SPEED_SCALES[1] + 2 * DEFAULT_SPEED_SCALES[0] ) / maximum_speed\*\*2

> a = (1 / maximum_speed\*\*2) * ( 2 * DEFAULT_SPEED_SCALES[0] - 4 * DEFAULT_SPEED_SCALES[1] )

> a =  - (1 / maximum_speed\*\*2) * 1.2 

The coefficients “a” and “b” are inversely related to the maximum speed. And we can 
confirm that increasing the maximum speed increases the width of the opening of 
parabola and moves the axis of the parabola side to side.

![Possible speeds](/media/modes-equations-possiblespeed1.png)

![Possible speeds](/media/modes-equations-possiblespeed1.png)

### Additional re-scaling in continuous action space to map network output to original range

In the continuous action space, we train the neural network to output mean values for steering_angle and speed in the range of [-1.0, 1.0]. These values are to be re-scaled back to the corresponding user provided minimum and maximum values. This is done by linearly scaling the mean value obtained for steering angle to the range of [minimum_steering_angle, maximum_steering_angle] and the mean value of speed to the range of [minimum_speed_value, maximum_speed_value].

This scaled speed value is mapped non linearly to a range of [0.0, 1.0] using the formula *y = ax^2 + bx*. We can see some of the examples of mapping the value [-1.0, 1.0] → [minimum_speed_value, maximum_speed_value] → [0.0, 1.0]. 

![Possible speeds](/media/modes-equations-scaling1.png)

![Possible speeds](/media/modes-equations-scaling2.png)

![Possible speeds](/media/modes-equations-scaling3.png)

### Autonomous mode: Impact of the Maximum Speed % value set by user in Device Console on the speed value

The DeepRacer console allows the user to set the maximum speed % values between [0, 100]% with a default value set to 50%.  In the autonomous mode, this value that is set from the front end is passed back to the navigation node via control_node where the non linearly scaled values are multiplied to this percentage. 

For example, if the user has set the maximum speed value to be 40%, and the model selected has a maximum speed of 0.8 m/s, then the throttle value set in servo message in the navigation node for a neural network output speed of 0.4 m/s is:

> non_linear_scaled_speed = 0.8 (0.4m/s corresponds to 0.8 of non linear scaled speed for a maximum speed of 0.8m/s )

> servo_msg.throttle = maximum_speed_threshold * non_linear_scaled_speed = 0.4 * 0.8 = 0.32

If we consider for the same maximum speed threshold of 40% and maximum speed of 0.8m/s, if the neural network outputs a speed of 0.2 m/s, then the value of the non_linear_scaled_speed decreases according to the equation derived before:

> coefficients = a, b = -1.875, 2.75 (for max speed 0.8 m/s) 
> speed= 0.2 m/s

> non_linear_scaled_speed = a * s\*\*2 + b * s =  -1.875 * 0.2\*\*2 + 2.75 * 0.2 
>                         = 0.475 (0.2m/s corresponds to 0.475 for a maximum speed of 0.8 m/s)

> servo_msg.throttle = maximum_speed_threshold * non_linear_scaled_speed*
>                    = 0.4 * 0.475 = 0.19

### Manual mode: Math behind the non linear speed mapping equations

As seen in the continuous action space in autonomous mode, we now need to non linearly 
map the raw value obtained from the joystick movement to the PWM values of the servo 
and motor for the car to move.

As we do not have a maximum speed value defined to calculate the coefficients of the 
equation *y = ax\*\*2 + bx* we use the maximum speed % from the device console to map to a 
range of [1.0, 5.0] speed scale values. This allows us to recalculate the curve for 
each maximum speed % value and use that to map it to throttle value in servo message.

The idea behind this is that a lower percentage of maximum speed % should map to a 
higher speed scale value while calculating the coefficients so that the curve is more 
flatter and the impact of actual speed values is less for lower max speed % as shown 
below:

![Possible speeds](/media/modes-manual-maxspeed.png)

**Why do we need to map maximum speed percent values [0.0, 1.0] to another speed scale 
value in range [5.0, 1.0] to calculate coefficients a and b?**

The maximum_speed_percentage values that we get as an user input is in the range of [0.
0, 1.0], and higher values mean the user wants more out of the joystick movement. In 
other words, higher the values for these maximum_speed_percentage means that we need to 
have a steep curve while mapping the possible speed values to their transformed 
counterpart(throttle value in servo message). So we need to inversely map the maximum speed percentage values to the 
speed scale value used to calculate the *non_linear_scaled_speed*.

We map the 100% maximum speed to 1.0 because the possible speed values from the 
joystick range from [0.0, 1.0] and 1.0 is the minimum value where the curve peaks 
before reversing in the figure seen above. This indicates that we will be guaranteed to 
have a non linearly increasing mapping for all joystick values [0.0, 1.0].

We map the 0% to 5.0, to add a safe buffer to accommodate different 
vehicle battery charge levels. For example, for a lower battery charge, the user might use a 
maximum speed percentage of 70% to control the car, but when fully charged, he can 
reduce the maximum_speed_percentage to 10% to have a similar effect.

#### Example 1

```
input:
throttle (from the joystick) = 0.567; 
max_speed_pct (from +/- adjust maximum speed buttons) = 0.5

calculated values:
categorized_throttle = 0.5
a =  -0.13333333333333336, b =  0.7333333333333334

mapped_speed = a * categorized_throttle**2 + b * categorized_throttle
             = -0.13333333333333336 * 0.5**2 + 0.7333333333333334 * 0.5 
             = -0.13333333333333336 * 0.25 +  0.7333333333333334 * 0.5
             = -0.033333333 + 0.366666667
             = 0.333333334
```

#### Example 2

```
input:
throttle (from the joystick) = 0.567; 
max_speed_pct (from +/- adjust maximum speed buttons) = 0.6

calculated values:
categorized_throttle = 0.5
a =  -0.17751479289940836, b =  0.8461538461538464

mapped_speed = -0.17751479289940836 * 0.25 + 0.8461538461538464 * 0.5 = 0.378698225
```

#### Example 3
```
input:
throttle (from the joystick) = 0.467; 
max_speed_pct (from +/- adjust maximum speed buttons) = 0.6

calculated values:
categorized_throttle = 0.3
a =  -0.17751479289940836, b =  0.8461538461538464

mapped_speed = -0.17751479289940836 * 0.09 + 0.8461538461538464 * 0.3 = 0.237869822
```

After we have the final categorizing and mapping completed for the speed and steering_angle, we have the output in the range of [-1.0, 1.0]. These values are passed to the servo node similar to the autonomous mode to move the car.

## Setting the servo and motor PWM duty values

The servo node expects the speed and steering_angle values sent as part of servo message in the range of [-1.0, 1.0]. Although for the autonomous mode, the speed will be in the range of [0.0, 1.0] as we do not support reverse driving for our car (yet). 

The DeepRacer device is open loop system and do not have a feedback to recognize the speed of the car. We use linearly map the value of speed and steering_angle obtained at the servo node to the duty cycle values which regulates the RPM of the servo and motor. The exact value written as the PWM duty on to the motor/servo file further depends on the bounding calibration values set during the calibration flow. 

## Summary

This component is one of the core features of the AWS DeepRacer application. The [Follow Me](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project) sample project developed, leverages most of the concepts used in the manual mode to build the **follow_me_navigation** node. You can learn more about the Follow Me sample project [here](https://github.com/awsdeepracer/aws-deepracer-follow-me-sample-project).



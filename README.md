# Project 3 - Implementation of Control Laws for Car-Like Robot
## Autonomous Robots Course

In this project you will implement the methods for four control laws as covered in class:
* Move to Point
* Follow Path
* Move to Pose
* Follow Line


The starter files contain code that implement most of the needed functionality and the project has the structure of a ROS workspace. After cloning your project and before compiling with catkin_make, please make to understand the code. Then, make the necessary editions so the project can compile.

### Getting the Starter Files
The files can be obtained using the URL listed in the green button above labeled "Clone or Download"

```$ git clone [Team URL]```

### Files
* auto_driver/src:
  * 03_robot.cpp - Robot class. Here is where you need to implement the control laws. 
    * Find sections between the following markers:
      ```
      // WRITE YOUR CODE: BEGIN
      ...
      // WRITE YOUR CODE: END
      ```
    * After you implement the methods, you also need to tune the constants in each controller to improve performance as much as possible
  * 03_controllers.cpp - this is the main program that uses the Robot class. 
  * 03_test.cpp - this program helps send paths to the controllers program without having to manually publish messages from the terminal. Feel free to try different paths
* auto_driver/CMakeLists.txt - compiling directives

### Submission
The project will be submitted through classroom.github.com

## Grade:
|      Code     | Move to Point |  Follow Path  |  Move to Pose |  Follow Line  | Total |
| ------------- | ------------- | ------------- | ------------- | ------------- | ----- |
|               |               |               |               |               |       |



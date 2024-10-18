# RO2 Navigation Final Project 

## Main Goal

The goal of this project was to implement a navigation system for the Turtlebot Burger 3 robot using SLAM. First, the robot created a map of the environment using the cartographer package. Then, the robot will use the map to localize itself using Adaptive Monte Carlo Localization and navigate to a given goal on the map using Nav2. Finally, the robot will have a running node to allow the user to save specific locations on the map and later give them as goals to the robot. Local and global cost maps were used to avoid permanent and moving obstacles. During the development, the robot was simulated in Gazebo and then tested on a real robot. Rviz2 was used to visualize the robot and the maps.

## Youtube Demo 

[![Youtube robot demo](https://img.youtube.com/vi/LfXKaNz3QBk/0.jpg)](https://youtu.be/LfXKaNz3QBk)
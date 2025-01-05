# Global Path Planning Algorithm

This program was created for my final year Engineering project at the University of Cape Town. The application creates a path through a map/image of terrain with obstacles to guide a unmanned surface vehicle from a start point to end point. The algorithm is based on rapidly exploring random trees (RRTs)
# File Layout

The folder consists of a Visual studio .sln file and root folder "opencvtry2" with the code. Images are included for testing. The program
utilizes openCV and contains 2 .lib files

# How to use the program

Change the filename on the first line of the main function to select a picture. If using a satelite image, uncomment the "satelitetoCspace" function and comment out the img.clone()
For regular images, do the opposite

additional functions are already in the code structure and can be commented/uncommented to view the effects

Use the mouse to select  a start and end point on the picture and enter a branch length on the console window. 

Enjoy!

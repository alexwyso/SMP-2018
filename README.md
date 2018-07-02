# SMP-2018

This repository contains a single library compiled from the various libraries written to operate the TOBI v3.2 robot for the Summer Mentorship Program, Engineering 2018. The original TOBI library and header file were created by Cherag Bhagwagar (10/21/2016). Updates (denoted by $ at the start of comments) and re-organization were done by Andrea Frank in the summer of 2017. Additional reorganization and compilation was done by Alex Wysoczanski (7/2/18).

INCLUDED LIBRARIES:

Tobi - Contains basic methods for initialization, flashing LEDs, controling the motors of each leg, and reading the encoders. 

TobiPro - Contains more advanced methods for calculating speed of motors and updating filters.

TobiCustom - Contains a single method to drive the robot forward.

Future commits will contain methods from TobiFilterManager as well.

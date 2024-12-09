# MIE444
Code repo from the MIE444 final project.

## Milestone 1
Contains test_client.py, the script used for the presentation of Milestone 1. The physical rover was undergoing major design changes at that moment, so simmer was used instead.


## Milestone 2
Contains obstacle_avoidance.py, used in Milestone 2.


## Milestone 3
Contains the code presented during the final assessment, Milestone 3.



Final scripts:

particle_ft.py is the main file of the particle filter node used for localization;

final_integration.ino is the script used on the ESP32;

bearing_to_target.py is the navigation script used to navigate from the position, estimated by the particle filter, to the specified cell in the maze. 


Intermediate and unfinished work:

esp32_ap.ino was used by the teammate working remotely, to set up a wifi access point on another ESP32 to connect the RPI to wifi.

unfinished_Astar.py is the nearly-finished version of our A* navigation, that wasn't used during the final presentation, as more debugging was needed.

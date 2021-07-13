# Hole_Detect

Main code initialize connection with the robot. (New: Drill Function Added) <br />
ArUco Detect, detect the aruco marker and returns the vector taken from the aruco marker. (Vector contains the translation and rotation of the camera in relativs to the aruco marker) <br />
Detect Holes Function, create custom mask (mask now is initialized, the option for custom mask still available though) to detect holes using blob detect, then return the x and y position of the closest circle center in relativs to optical center of the camera. <br />
Move to Hole function moves the robot arm to the hole and call detect hole function to correct the position of the hole. It also takes picture to test if the holes are on the center or not (now disabled) using: https://yangcha.github.io/iview/iview.html <br />
Drill function, needs the robot IP address to set the digital outputs of the robots in order to operate the drill. <br />
The whole thing does not need any user input, it all work by itself. (The Drill part takes user input only to check before proceeding, will be updated later to work automatically) <br />

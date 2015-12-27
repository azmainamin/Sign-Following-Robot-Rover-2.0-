# Sign-Following-Robot-Rover-2.0-

We used the Rover 2.0 tank made by Brookstone. We used OpenCV to detect blue signs via the rover's webcam. By finding the contours and measuring the area of the sign, the rover moves towards the sign untill the sign is big enough to be read determined by an area threshold. Then we used Pytesseract to read the sign and execute it. 

Known issues: It was scripted in Python 2.7.5. There are some wifi issues with Python 2.7.9 and above. 

You can find Rover20.py here: https://github.com/simondlevy/RoverPylot/

Demo of the project: https://www.youtube.com/watch?v=RdmbT84CEdU

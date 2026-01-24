All auto pathing code and missions curently require you to have PX4 and QGroundControl simulation open. 
It doesnt yet support automatic recording of camera data.

EDIT 2
Auto pathing no longer requires PX4 nor QGroundControl to be run in command windows before executing the code.
Everything is done via single python file (Fly_Orbit.py).
Data is automatycly saved to a created folder with name "mission_data_yyyymmdd_hhmmss" 
Data should be _synchronized_ , key word beeing should.

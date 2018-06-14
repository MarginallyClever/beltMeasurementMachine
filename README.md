# Belt Measurement Machine

Just what is says - a machine to measure out precise amounts of timing belt.

https://www.instagram.com/p/BkBHme7HyrU/

I have to cut precise lengths of timing belt for use in my Makelangelo polargraph robot.  By hand is pretty good... but it could be exact and save a few calibration steps later.  So I made a machine that measures exactly the length of belt requested.

It's open hardware, so feel free to remix it with an automated cutter.

# Assembly

Print:
3x guide rings (turn it over to avoid support material)
4x leg
1x guide 1
1x guide 2

All parts were printed on a Prusa MK3.  bearings were press-fit tight.

Laser cut one table.dxf.  The short side should be 375mm.

I used 
three M3x15 screws and 
six M3 nuts
to hold the RUMBA board on the board.

I used 
four M3x10 screws
four M3x20 standoffs
four M3 nuts
to hold the LCD panel on the board.

The RUMBA control board has one A4988 Polulu driver to move the stepper motor.
The stepper motor is 1.8 degrees per step (200 steps per turn).  The RUMBA board has full 1/16th microstepping activated.

I used a stepper driver that is complete overkill because I have lots in stock.
Even an Adafruit motor shield on an UNO would have sufficed.

An M5 bolt goes in the center of the table as a bobbin.
8x22x7 skate bearings were used around the legs.
A GT2-6 20-tooth pulley on the stepper motor pulls the belt between the idlers and out through the guide rail.

# More 

Code to run the robot is here:
https://github.com/MarginallyClever/beltMeasurementMachine
Find all the 3D printed parts and laser cut DXF here:
https://www.thingiverse.com/thing:2961400

If you like this please tell your friends about http://marginallyclever.com
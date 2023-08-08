# BeltMeasurementMachine

Timing belt measurement machine.  Uses a Sixi 3 PCB and some printed parts.

The gcode command to send is
```
G7 X1000
```
to "emit" 1000mm (1m) of belt.  Use negative values to retract.

System will use a crude acceleration/deceleration to run a little quicker.
System will detect interruptions in the sensor movement and attempt to compensate.
Note that the sensor plate is only a friction fit on the shaft so it can slip.

![Fusion 360 preview](image.jpg)

![actual build](IMG_1226.jpg)
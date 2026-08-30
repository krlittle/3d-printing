; Test gcode file for Monoprice Voxel
; Small benchy-like model ~80mm dimensions
G28 ; Home all axes
G29 ; Auto level bed
M140 S60 ; Set bed temp to 60C
M104 S200 ; Set nozzle temp to 200C
G1 Z0.4 F1200 ; Move to first layer height
G1 X10 Y10 Z0.4 F1200 ; Move to start position
G1 X80 Y10 Z0.4 F60 ; First extrusion line
G1 X80 Y80 Z0.4 F60 ; Move along Y
G1 X10 Y80 Z0.4 F60 ; Return along X
G1 X10 Y10 Z0.4 F60 ; Close the rectangle
; Move up for next layer
G1 Z0.5 F1200
G1 X10 Y10 Z0.5 F1200
G1 X80 Y10 Z0.5 F60
G1 X80 Y80 Z0.5 F60
G1 X10 Y80 Z0.5 F60
G1 X10 Y10 Z0.5 F60
; Retract and turn off
M104 S0 ; Turn off nozzle heater
M140 S0 ; Turn off bed heater
G28 ; Home axes at end
M84 ; Disable motors

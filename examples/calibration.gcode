(SimuForge IK Calibration / Stress-Test Program)
(Workpiece: 305mm cube, half_extent = 152.5mm)
(G-code origin = top center, Z=0 = surface, negative = into material)
(XY range: -152.5 to +152.5)
(A-axis = rotary table: A0=top, A90=front, A-90=back, A180=bottom)
(Tests: velocity limits, orientation control, config consistency,)
(  singularity damping, collision avoidance, null-space centering,)
(  A-axis rotation, multi-face access)

G0 Z10                (Safe height)
M3 S10000            (Spindle on)

(========================================================)
(=== FACE 1: TOP FACE - A0                            ===)
(========================================================)

(--- Section 1: Cross Pattern ---)
(--- Sweep +/-X and +/-Y axes at Z=-3 ---)

G0 X0 Y0
G1 Z-3 F300
G0 Z5

G0 X-120 Y0
G1 Z-3 F300
G1 X120 Y0 F800
G0 Z5

G0 X0 Y-120
G1 Z-3 F300
G1 X0 Y120 F800
G0 Z10

(--- Section 2: Perimeter Trace ---)

G0 X-120 Y-120
G1 Z-5 F300
G1 X120 Y-120 F600
G1 X120 Y120 F600
G1 X-120 Y120 F600
G1 X-120 Y-120 F600
G0 Z10

(--- Section 3: Diagonal Star ---)

G0 X-120 Y-120
G1 Z-5 F300
G1 X120 Y120 F800
G0 Z10

G0 X-120 Y120
G1 Z-5 F300
G1 X120 Y-120 F800
G0 Z10

(--- Section 4: Depth Spiral at Z=-15 ---)

G0 X-110 Y-110
G1 Z-15 F300
G1 X110 Y-110 F600
G1 X110 Y110 F600
G1 X-110 Y110 F600
G1 X-110 Y-70 F600
G1 X70 Y-70 F600
G1 X70 Y70 F600
G1 X-70 Y70 F600
G1 X-70 Y-30 F600
G1 X30 Y-30 F600
G1 X30 Y30 F600
G1 X-30 Y30 F600
G1 X-30 Y0 F600
G1 X0 Y0 F600
G0 Z10

(--- Section 5: Corner Triangles ---)

G0 X100 Y100
G1 Z-5 F300
G1 X120 Y100 F500
G1 X120 Y120 F500
G1 X100 Y120 F500
G1 X100 Y100 F500
G0 Z10

G0 X-100 Y100
G1 Z-5 F300
G1 X-120 Y100 F500
G1 X-120 Y120 F500
G1 X-100 Y120 F500
G1 X-100 Y100 F500
G0 Z10

G0 X-100 Y-100
G1 Z-5 F300
G1 X-120 Y-100 F500
G1 X-120 Y-120 F500
G1 X-100 Y-120 F500
G1 X-100 Y-100 F500
G0 Z10

G0 X100 Y-100
G1 Z-5 F300
G1 X120 Y-100 F500
G1 X120 Y-120 F500
G1 X100 Y-120 F500
G1 X100 Y-100 F500
G0 Z10

(--- Section 6: Speed Ramp ---)

G0 X-120 Y0
G1 Z-3 F300
G1 X-80 Y0 F200
G1 X-40 Y0 F400
G1 X0 Y0 F600
G1 X40 Y0 F1000
G1 X80 Y0 F1500
G1 X120 Y0 F2000
G0 Z10

(--- Section 7: Zigzag 5mm pitch ---)

G0 X-40 Y-40
G1 Z-5 F300
G1 X-40 Y40 F800
G1 X-35 Y40 F800
G1 X-35 Y-40 F800
G1 X-30 Y-40 F800
G1 X-30 Y40 F800
G1 X-25 Y40 F800
G1 X-25 Y-40 F800
G1 X-20 Y-40 F800
G1 X-20 Y40 F800
G1 X-15 Y40 F800
G1 X-15 Y-40 F800
G1 X-10 Y-40 F800
G1 X-10 Y40 F800
G1 X-5 Y40 F800
G1 X-5 Y-40 F800
G1 X0 Y-40 F800
G1 X0 Y40 F800
G1 X5 Y40 F800
G1 X5 Y-40 F800
G1 X10 Y-40 F800
G1 X10 Y40 F800
G1 X15 Y40 F800
G1 X15 Y-40 F800
G1 X20 Y-40 F800
G1 X20 Y40 F800
G1 X25 Y40 F800
G1 X25 Y-40 F800
G1 X30 Y-40 F800
G1 X30 Y40 F800
G1 X35 Y40 F800
G1 X35 Y-40 F800
G1 X40 Y-40 F800
G1 X40 Y40 F800
G0 Z10

(========================================================)
(=== FACE 2: FRONT FACE - A90                         ===)
(=== Rotary table tilts workpiece 90 degrees           ===)
(=== Former +Y face now faces up                       ===)
(========================================================)

M5
G0 Z25
G0 X0 Y0
G0 A90
M3 S10000
G0 Z10

(--- Cross on front face ---)
G0 X0 Y0
G1 Z-3 F300
G0 Z5

G0 X-100 Y0
G1 Z-3 F300
G1 X100 Y0 F800
G0 Z5

G0 X0 Y-100
G1 Z-3 F300
G1 X0 Y100 F800
G0 Z10

(--- Perimeter on front face ---)
G0 X-100 Y-100
G1 Z-5 F300
G1 X100 Y-100 F600
G1 X100 Y100 F600
G1 X-100 Y100 F600
G1 X-100 Y-100 F600
G0 Z10

(--- Diagonal on front face ---)
G0 X-100 Y-100
G1 Z-5 F300
G1 X100 Y100 F800
G0 Z10

G0 X-100 Y100
G1 Z-5 F300
G1 X100 Y-100 F800
G0 Z10

(========================================================)
(=== FACE 3: RIGHT SIDE - A90 already, work on +X edge===)
(=== Plunge along right edge                           ===)
(========================================================)

G0 X120 Y0
G1 Z0 F300
G1 Z-15 F400
G1 Z-30 F400
G0 Z10

G0 X-120 Y0
G1 Z0 F300
G1 Z-15 F400
G1 Z-30 F400
G0 Z10

(========================================================)
(=== FACE 4: BACK FACE - A-90                         ===)
(=== Rotary table tilts workpiece -90 degrees          ===)
(=== Former -Y face now faces up                       ===)
(========================================================)

M5
G0 Z25
G0 X0 Y0
G0 A-90
M3 S10000
G0 Z10

(--- Cross on back face ---)
G0 X-100 Y0
G1 Z-3 F300
G1 X100 Y0 F800
G0 Z5

G0 X0 Y-100
G1 Z-3 F300
G1 X0 Y100 F800
G0 Z10

(--- Perimeter on back face ---)
G0 X-100 Y-100
G1 Z-5 F300
G1 X100 Y-100 F600
G1 X100 Y100 F600
G1 X-100 Y100 F600
G1 X-100 Y-100 F600
G0 Z10

(========================================================)
(=== FACE 5: BOTTOM FACE - A180                       ===)
(=== Rotary table flips workpiece upside down          ===)
(========================================================)

M5
G0 Z25
G0 X0 Y0
G0 A180
M3 S10000
G0 Z10

(--- Cross on bottom face ---)
G0 X-80 Y0
G1 Z-3 F300
G1 X80 Y0 F800
G0 Z5

G0 X0 Y-80
G1 Z-3 F300
G1 X0 Y80 F800
G0 Z10

(--- Small spiral on bottom ---)
G0 X-60 Y-60
G1 Z-8 F300
G1 X60 Y-60 F600
G1 X60 Y60 F600
G1 X-60 Y60 F600
G1 X-60 Y-30 F600
G1 X30 Y-30 F600
G1 X30 Y30 F600
G1 X-30 Y30 F600
G1 X-30 Y0 F600
G1 X0 Y0 F600
G0 Z10

(========================================================)
(=== RETURN: Back to A0, center, done                 ===)
(========================================================)

M5
G0 Z25
G0 X0 Y0
G0 A0
G0 Z25
G0 X0 Y0

(SimuForge IK Calibration / Stress-Test Program)
(Workpiece: 305mm cube, half_extent = 152.5mm)
(G-code origin = top center, Z=0 = surface, negative = into material)
(XY range: -152.5 to +152.5)
(Tests: velocity limits, orientation control, config consistency,)
(  singularity damping, collision avoidance, null-space centering)

G0 Z10                (Safe height)
M3 S10000            (Spindle on)

(========================================================)
(=== SECTION 1: Top Face Cross Pattern                ===)
(=== Sweep +/-X and +/-Y axes at Z=-3                 ===)
(=== Tests: J1 rotation through full range             ===)
(========================================================)

(Center touch)
G0 X0 Y0
G1 Z-3 F300
G0 Z5

(Sweep +X axis)
G0 X-140 Y0
G1 Z-3 F300
G1 X140 Y0 F800      (Full +X sweep)
G0 Z5

(Sweep +Y axis)
G0 X0 Y-140
G1 Z-3 F300
G1 X0 Y140 F800      (Full +Y sweep)
G0 Z5

(Sweep -X axis back)
G0 X140 Y0
G1 Z-3 F300
G1 X-140 Y0 F800     (Full -X sweep)
G0 Z5

(Sweep -Y axis back)
G0 X0 Y140
G1 Z-3 F300
G1 X0 Y-140 F800     (Full -Y sweep)
G0 Z10

(========================================================)
(=== SECTION 2: Top Face Perimeter Trace              ===)
(=== One continuous cut around the full square         ===)
(=== Tests: smooth continuous motion, config at corners===)
(========================================================)

G0 X-140 Y-140
G1 Z-5 F300
G1 X140 Y-140 F600   (Bottom edge, +X direction)
G1 X140 Y140 F600    (Right edge, +Y direction)
G1 X-140 Y140 F600   (Top edge, -X direction)
G1 X-140 Y-140 F600  (Left edge, -Y direction, close loop)
G0 Z10

(========================================================)
(=== SECTION 3: Top Face Diagonal Star                ===)
(=== X-pattern diagonals corner-to-corner             ===)
(=== Tests: rapid direction changes, config flip reject===)
(========================================================)

(Diagonal 1: bottom-left to top-right)
G0 X-140 Y-140
G1 Z-5 F300
G1 X140 Y140 F800
G0 Z10

(Diagonal 2: top-left to bottom-right)
G0 X-140 Y140
G1 Z-5 F300
G1 X140 Y-140 F800
G0 Z10

(Diagonal 3: bottom-right to top-left)
G0 X140 Y-140
G1 Z-5 F300
G1 X-140 Y140 F800
G0 Z10

(Diagonal 4: top-right to bottom-left)
G0 X140 Y140
G1 Z-5 F300
G1 X-140 Y-140 F800
G0 Z10

(========================================================)
(=== SECTION 4: Concentric Depth Spirals              ===)
(=== Square spirals inward at Z=-10, -20, -30         ===)
(=== Tests: multi-depth stability, progressive reach  ===)
(========================================================)

(--- Depth Z=-10: outer spiral ---)
G0 X-130 Y-130
G1 Z-10 F300
G1 X130 Y-130 F600
G1 X130 Y130 F600
G1 X-130 Y130 F600
G1 X-130 Y-100 F600
G1 X100 Y-100 F600
G1 X100 Y100 F600
G1 X-100 Y100 F600
G1 X-100 Y-70 F600
G1 X70 Y-70 F600
G1 X70 Y70 F600
G1 X-70 Y70 F600
G1 X-70 Y-40 F600
G1 X40 Y-40 F600
G1 X40 Y40 F600
G1 X-40 Y40 F600
G1 X-40 Y-10 F600
G1 X0 Y0 F600        (Spiral to center)
G0 Z10

(--- Depth Z=-20: mid spiral ---)
G0 X-120 Y-120
G1 Z-20 F300
G1 X120 Y-120 F600
G1 X120 Y120 F600
G1 X-120 Y120 F600
G1 X-120 Y-80 F600
G1 X80 Y-80 F600
G1 X80 Y80 F600
G1 X-80 Y80 F600
G1 X-80 Y-40 F600
G1 X40 Y-40 F600
G1 X40 Y40 F600
G1 X-40 Y40 F600
G1 X-40 Y0 F600
G1 X0 Y0 F600        (Spiral to center)
G0 Z10

(--- Depth Z=-30: deep spiral ---)
G0 X-100 Y-100
G1 Z-30 F300
G1 X100 Y-100 F600
G1 X100 Y100 F600
G1 X-100 Y100 F600
G1 X-100 Y-60 F600
G1 X60 Y-60 F600
G1 X60 Y60 F600
G1 X-60 Y60 F600
G1 X-60 Y-20 F600
G1 X20 Y-20 F600
G1 X20 Y20 F600
G1 X-20 Y20 F600
G1 X-20 Y0 F600
G1 X0 Y0 F600        (Spiral to center)
G0 Z10

(========================================================)
(=== SECTION 5: Edge Plunge Profiles (4 Sides)        ===)
(=== Vertical plunges at each face center edge        ===)
(=== Tests: side-face access, deep Z reach, collision ===)
(========================================================)

(+X face center: plunge down the edge)
G0 X145 Y0
G1 Z0 F300
G1 Z-20 F400
G1 Z-40 F400
G1 Z-60 F400
G1 Z-80 F400
G0 Z10

(-X face center: plunge down the edge)
G0 X-145 Y0
G1 Z0 F300
G1 Z-20 F400
G1 Z-40 F400
G1 Z-60 F400
G1 Z-80 F400
G0 Z10

(+Y face center: plunge down the edge)
G0 X0 Y145
G1 Z0 F300
G1 Z-20 F400
G1 Z-40 F400
G1 Z-60 F400
G1 Z-80 F400
G0 Z10

(-Y face center: plunge down the edge)
G0 X0 Y-145
G1 Z0 F300
G1 Z-20 F400
G1 Z-40 F400
G1 Z-60 F400
G1 Z-80 F400
G0 Z10

(========================================================)
(=== SECTION 6: Corner Approach Stress Test           ===)
(=== Approach all 4 top corners, cut small triangles  ===)
(=== Tests: workspace boundary, singularity damping   ===)
(========================================================)

(Corner +X,+Y: approach and cut triangle)
G0 X0 Y0
G0 Z10
G0 X120 Y120
G1 Z-5 F300
G1 X140 Y120 F500
G1 X140 Y140 F500
G1 X120 Y140 F500
G1 X120 Y120 F500    (Close triangle)
G0 Z10

(Corner -X,+Y: approach and cut triangle)
G0 X0 Y0
G0 X-120 Y120
G1 Z-5 F300
G1 X-140 Y120 F500
G1 X-140 Y140 F500
G1 X-120 Y140 F500
G1 X-120 Y120 F500
G0 Z10

(Corner -X,-Y: approach and cut triangle)
G0 X0 Y0
G0 X-120 Y-120
G1 Z-5 F300
G1 X-140 Y-120 F500
G1 X-140 Y-140 F500
G1 X-120 Y-140 F500
G1 X-120 Y-120 F500
G0 Z10

(Corner +X,-Y: approach and cut triangle)
G0 X0 Y0
G0 X120 Y-120
G1 Z-5 F300
G1 X140 Y-120 F500
G1 X140 Y-140 F500
G1 X120 Y-140 F500
G1 X120 Y-120 F500
G0 Z10

(========================================================)
(=== SECTION 7: Speed Ramp Test                       ===)
(=== Single line across top face, increasing feedrate ===)
(=== Tests: velocity limit scaling, smooth accel      ===)
(========================================================)

G0 X-130 Y0
G1 Z-3 F300

(Slow start)
G1 X-100 Y0 F200
(Medium)
G1 X-60 Y0 F400
(Faster)
G1 X-20 Y0 F600
(Fast)
G1 X20 Y0 F800
(Very fast)
G1 X60 Y0 F1200
(Maximum)
G1 X100 Y0 F1600
(Sprint)
G1 X130 Y0 F2000
G0 Z10

(========================================================)
(=== SECTION 8: Zigzag Stress Pattern                 ===)
(=== Tight 5mm pitch zigzag across 100mm strip        ===)
(=== Tests: config consistency, null-space stability   ===)
(========================================================)

G0 X-50 Y-50
G1 Z-5 F300

G1 X-50 Y50 F800
G1 X-45 Y50 F800
G1 X-45 Y-50 F800
G1 X-40 Y-50 F800
G1 X-40 Y50 F800
G1 X-35 Y50 F800
G1 X-35 Y-50 F800
G1 X-30 Y-50 F800
G1 X-30 Y50 F800
G1 X-25 Y50 F800
G1 X-25 Y-50 F800
G1 X-20 Y-50 F800
G1 X-20 Y50 F800
G1 X-15 Y50 F800
G1 X-15 Y-50 F800
G1 X-10 Y-50 F800
G1 X-10 Y50 F800
G1 X-5 Y50 F800
G1 X-5 Y-50 F800
G1 X0 Y-50 F800
G1 X0 Y50 F800
G1 X5 Y50 F800
G1 X5 Y-50 F800
G1 X10 Y-50 F800
G1 X10 Y50 F800
G1 X15 Y50 F800
G1 X15 Y-50 F800
G1 X20 Y-50 F800
G1 X20 Y50 F800
G1 X25 Y50 F800
G1 X25 Y-50 F800
G1 X30 Y-50 F800
G1 X30 Y50 F800
G1 X35 Y50 F800
G1 X35 Y-50 F800
G1 X40 Y-50 F800
G1 X40 Y50 F800
G1 X45 Y50 F800
G1 X45 Y-50 F800
G1 X50 Y-50 F800
G1 X50 Y50 F800

G0 Z10

(========================================================)
(=== SECTION 9: Return Home                           ===)
(========================================================)

M5                    (Spindle off)
G0 Z25               (Final retract)
G0 X0 Y0             (Return to center)

(SimuForge Calibration - Carve bold numbers 1-4 on four faces)
(Workpiece: 305mm cube, half_extent = 152.5mm)
(G-code origin = top center, Z=0 = surface, negative = into material)
(Numbers ~200mm tall, strokes ~30mm wide via multiple parallel passes)
(A-axis = rotary table: A0=top, A90=front, A-90=back, A180=bottom)
(U-axis = linear track: auto-positions dynamically for optimal arm reach)

G0 Z10
M3 S10000

(========================================================)
(=== FACE 1: TOP - Bold "1"                           ===)
(========================================================)

(--- Vertical stroke, 5 passes at X=-8,-4,0,4,8 ---)
G0 X-8 Y100
G1 Z-10 F300
G1 X-8 Y-100 F600
G0 Z5
G0 X-4 Y100
G1 Z-10 F300
G1 X-4 Y-100 F600
G0 Z5
G0 X0 Y100
G1 Z-10 F300
G1 X0 Y-100 F600
G0 Z5
G0 X4 Y100
G1 Z-10 F300
G1 X4 Y-100 F600
G0 Z5
G0 X8 Y100
G1 Z-10 F300
G1 X8 Y-100 F600
G0 Z5

(--- Bottom serif, 3 passes at Y=-100,-92,-84 ---)
G0 X-50 Y-100
G1 Z-10 F300
G1 X50 Y-100 F600
G0 Z5
G0 X-50 Y-92
G1 Z-10 F300
G1 X50 Y-92 F600
G0 Z5
G0 X-50 Y-84
G1 Z-10 F300
G1 X50 Y-84 F600
G0 Z5

(--- Angled serif, 3 passes ---)
G0 X-30 Y52
G1 Z-10 F300
G1 X-8 Y100 F600
G0 Z5
G0 X-22 Y52
G1 Z-10 F300
G1 X0 Y100 F600
G0 Z5
G0 X-14 Y52
G1 Z-10 F300
G1 X8 Y100 F600
G0 Z10

(========================================================)
(=== FACE 2: FRONT - Bold "2"   (A90)                 ===)
(========================================================)

M5
G0 Z25
G0 X0 Y0
G0 A90
M3 S10000
G0 Z10

(--- Top bar, 3 passes ---)
G0 X-50 Y100
G1 Z-10 F300
G1 X50 Y100 F600
G0 Z5
G0 X-50 Y92
G1 Z-10 F300
G1 X50 Y92 F600
G0 Z5
G0 X-50 Y84
G1 Z-10 F300
G1 X50 Y84 F600
G0 Z5

(--- Right side down, 3 passes ---)
G0 X50 Y84
G1 Z-10 F300
G1 X50 Y20 F600
G0 Z5
G0 X42 Y84
G1 Z-10 F300
G1 X42 Y20 F600
G0 Z5
G0 X34 Y84
G1 Z-10 F300
G1 X34 Y20 F600
G0 Z5

(--- Diagonal, 3 passes ---)
G0 X50 Y20
G1 Z-10 F300
G1 X-50 Y-20 F600
G0 Z5
G0 X42 Y20
G1 Z-10 F300
G1 X-42 Y-20 F600
G0 Z5
G0 X50 Y12
G1 Z-10 F300
G1 X-50 Y-28 F600
G0 Z5

(--- Left side down, 3 passes ---)
G0 X-50 Y-20
G1 Z-10 F300
G1 X-50 Y-84 F600
G0 Z5
G0 X-42 Y-20
G1 Z-10 F300
G1 X-42 Y-84 F600
G0 Z5
G0 X-34 Y-20
G1 Z-10 F300
G1 X-34 Y-84 F600
G0 Z5

(--- Bottom bar, 3 passes ---)
G0 X-50 Y-84
G1 Z-10 F300
G1 X50 Y-84 F600
G0 Z5
G0 X-50 Y-92
G1 Z-10 F300
G1 X50 Y-92 F600
G0 Z5
G0 X-50 Y-100
G1 Z-10 F300
G1 X50 Y-100 F600
G0 Z10

(========================================================)
(=== FACE 3: BACK - Bold "3"   (A-90)                 ===)
(========================================================)

M5
G0 Z25
G0 X0 Y0
G0 A-90
M3 S10000
G0 Z10

(--- Top bar, 3 passes ---)
G0 X-50 Y100
G1 Z-10 F300
G1 X50 Y100 F600
G0 Z5
G0 X-50 Y92
G1 Z-10 F300
G1 X50 Y92 F600
G0 Z5
G0 X-50 Y84
G1 Z-10 F300
G1 X50 Y84 F600
G0 Z5

(--- Right side top half, 3 passes ---)
G0 X50 Y84
G1 Z-10 F300
G1 X50 Y16 F600
G0 Z5
G0 X42 Y84
G1 Z-10 F300
G1 X42 Y16 F600
G0 Z5
G0 X34 Y84
G1 Z-10 F300
G1 X34 Y16 F600
G0 Z5

(--- Middle bar, 3 passes ---)
G0 X-20 Y16
G1 Z-10 F300
G1 X50 Y16 F600
G0 Z5
G0 X-20 Y8
G1 Z-10 F300
G1 X50 Y8 F600
G0 Z5
G0 X-20 Y0
G1 Z-10 F300
G1 X50 Y0 F600
G0 Z5

(--- Right side bottom half, 3 passes ---)
G0 X50 Y0
G1 Z-10 F300
G1 X50 Y-84 F600
G0 Z5
G0 X42 Y0
G1 Z-10 F300
G1 X42 Y-84 F600
G0 Z5
G0 X34 Y0
G1 Z-10 F300
G1 X34 Y-84 F600
G0 Z5

(--- Bottom bar, 3 passes ---)
G0 X-50 Y-84
G1 Z-10 F300
G1 X50 Y-84 F600
G0 Z5
G0 X-50 Y-92
G1 Z-10 F300
G1 X50 Y-92 F600
G0 Z5
G0 X-50 Y-100
G1 Z-10 F300
G1 X50 Y-100 F600
G0 Z10

(========================================================)
(=== FACE 4: BOTTOM - Bold "4"   (A180)               ===)
(========================================================)

M5
G0 Z25
G0 X0 Y0
G0 A180
M3 S10000
G0 Z10

(--- Left vertical stroke, 3 passes ---)
G0 X-45 Y100
G1 Z-10 F300
G1 X-45 Y0 F600
G0 Z5
G0 X-37 Y100
G1 Z-10 F300
G1 X-37 Y0 F600
G0 Z5
G0 X-29 Y100
G1 Z-10 F300
G1 X-29 Y0 F600
G0 Z5

(--- Crossbar, 3 passes ---)
G0 X-50 Y0
G1 Z-10 F300
G1 X50 Y0 F600
G0 Z5
G0 X-50 Y-8
G1 Z-10 F300
G1 X50 Y-8 F600
G0 Z5
G0 X-50 Y-16
G1 Z-10 F300
G1 X50 Y-16 F600
G0 Z5

(--- Right vertical full height, 5 passes ---)
G0 X16 Y100
G1 Z-10 F300
G1 X16 Y-100 F600
G0 Z5
G0 X20 Y100
G1 Z-10 F300
G1 X20 Y-100 F600
G0 Z5
G0 X24 Y100
G1 Z-10 F300
G1 X24 Y-100 F600
G0 Z5
G0 X28 Y100
G1 Z-10 F300
G1 X28 Y-100 F600
G0 Z5
G0 X32 Y100
G1 Z-10 F300
G1 X32 Y-100 F600
G0 Z10

(========================================================)
(=== RETURN: Back to A0, done                         ===)
(========================================================)

M5
G0 Z25
G0 X0 Y0
G0 A0
G0 Z25
G0 X0 Y0

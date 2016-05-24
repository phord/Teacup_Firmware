G1X10Y10F3000
G28X
; Watch Y after we home X. We do not change Y in the gcode, so Y should not drift.
G1X10F3000
; The printer should report we are at X:10 Y:10. The step counters should match.
M114
G28Z


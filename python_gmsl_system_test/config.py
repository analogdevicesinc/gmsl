captures = 10000

#Points defining rejion of interest 
roi_pts = [(100, 100), (200, 200)]

#Target HSV value for ROI
hsv_target = (64, 128, 128)
hsv_error = (16, 32, 32)

#Camera resolution
hres = 1024
vres = 720

#Camera name or part of name for finding correct camera
cam_name = "C270"

#Logic analyser capture time after trigger
capture_time = 2.0

power_supply = "PS3005D"

PS3005D_PORT = "COM16"

SHOW_DEBUG_FRAME = True
WRITE_DEBUG_VIDEO = True
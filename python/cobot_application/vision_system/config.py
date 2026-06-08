""" All coordinates are assumed to have an origin located at the center of 
the base of the cobot, level with the table it rests on """

#Pixel Coordinates of the bin corners
#These will be read by roboflow in the future
BIN_IMAGE_CORNERS = [
    [120,80],  #top left
    [520, 75],  #top right
    [550, 430],  #bottom right
    [100, 440]   #bottom left
]

#Real world coordinates of the same 4 bin corners (mm)
BIN_REAL_WORLD_CORNERS =[
    [0,0], #top left
    [600, 0], #top right
    [600, 400], #bottom right
    [0, 400]  #bottom left
]
#Height of can in mm
CAN_HEIGHT = 87 #!!!! NOT MEASURED YET !!!!!

#Layer height z coordinate (assume bin filled with 7 layers of crush cans)
LAYER_Z1 = 400   #!!!!!!!! NOT MEASURED YET !!!!!!!!!
LAYER_Z2 = LAYER_Z1-CAN_HEIGHT
LAYER_Z3 = LAYER_Z2-CAN_HEIGHT
LAYER_Z4 = LAYER_Z3-CAN_HEIGHT
LAYER_Z5 = LAYER_Z4-CAN_HEIGHT
LAYER_Z6 = LAYER_Z5-CAN_HEIGHT
LAYER_Z7 = LAYER_Z6-CAN_HEIGHT

# Percentage threshold for determining if can objects are on a new layer or not
LAYER_THRESHOLD = 0.7
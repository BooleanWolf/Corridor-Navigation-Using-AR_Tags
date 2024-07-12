import cv2
import numpy as np
from skimage import measure
import sys
import math

print("EXECUTION STARTED")

lowThreshold = 50
highThreshold = 100
maxThreshold = 200

apertureSizes = [3, 5, 7]
maxapertureIndex = 2
apertureIndex = 0

blurAmount = 0
maxBlurAmount = 20

# Function for all trackbar calls
def applyCanny():
    global blurAmount, apertureIndex, lowThreshold, highThreshold
    blurAmount = 3
    apertureSize = 3
    lowThreshold = 50
    highThreshold = 100
    if blurAmount > 0:
        blurredSrc = cv2.GaussianBlur(src, (2 * blurAmount + 1, 2 * blurAmount + 1), 0)
    else:
        blurredSrc = src.copy()
    
    edges = cv2.Canny(blurredSrc, lowThreshold, highThreshold, apertureSize)
    cv2.namedWindow("without ero_dil", cv2.WINDOW_NORMAL)
    cv2.imshow("without ero_dil", edges)
    
    kernel = np.ones((5, 5), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=5)
    edges = cv2.erode(edges, kernel, iterations=3)
    
    labels, no_of_blobs = measure.label(edges, connectivity=2, background=0, return_num=True)
    print("number of blobs =", no_of_blobs)
    
    pix_limit_low = 500
    pix_limit_high = 2000
    global final_img
    final_img = np.zeros(edges.shape, dtype="uint8")
    
    for label in np.unique(labels):
        if label == 0:
            continue
        
        label_mask = np.zeros(edges.shape, dtype="uint8")
        label_mask[labels == label] = 255
        num_pix = cv2.countNonZero(label_mask)
        
        if num_pix > pix_limit_high or num_pix < pix_limit_low:
            continue
        else:
            final_img = final_img + label_mask
    
    cv2.imshow("Edges", final_img)

# Function to update low threshold value
def updateLowThreshold(*args):
    global lowThreshold
    lowThreshold = args[0]
    applyCanny()

# Function to update high threshold value
def updateHighThreshold(*args):
    global highThreshold
    highThreshold = args[0]
    applyCanny()

# Function to update blur amount
def updateBlurAmount(*args):
    global blurAmount
    blurAmount = args[0]
    applyCanny()

# Function to update aperture index
def updateApertureIndex(*args):
    global apertureIndex
    apertureIndex = args[0]
    applyCanny()

src = cv2.imread('../assets/data/24.jpg', 0)
src_c = cv2.imread('../assets/data/24.jpg', 1)

edges = src.copy()
cv2.namedWindow("Edges", cv2.WINDOW_NORMAL)
cv2.imshow("Edges", src)

applyCanny()

cv2.waitKey(0)

minLineLength = 20
maxLineGap = 50
lines = cv2.HoughLinesP(final_img, 0.02, np.pi / 500, 20, minLineLength, maxLineGap)

if lines is None:
    print("No lines detected. Exiting from program")
    sys.exit()

a, b, c = lines.shape
(rows, cols) = edges.shape

print("edges.shape =", edges.shape)
print("len(lines) =", len(lines))
print("lines.shape =", lines.shape)

x_deviation = 20
y_deviation = 20
count = 0
frame_center = cols / 2
cv2.namedWindow("LINES", cv2.WINDOW_NORMAL)

x_top_left = None
y_top_left = None
x_bottom_left = None
y_bottom_left = None
x_top_right = None
y_top_right = None
x_bottom_right = None
y_bottom_right = None

for i in range(b):
    x1 = lines[0][i][0]
    y1 = lines[0][i][1]
    x2 = lines[0][i][2]
    y2 = lines[0][i][3]
    x_min = min(x1, x2)
    y_min = min(y1, y2)
    x_max = max(x1, x2)
    y_max = max(y1, y2)
    x_length = x_max - x_min
    y_length = y_max - y_min
    
    if x_length <= x_deviation or y_length <= y_deviation:
        continue
    
    slope = round(((y1 - y2) / (float)(x2 - x1)), 2)
    
    if x_bottom_left == None and slope > 0:
        x_bottom_left = x_min
        y_bottom_left = y_max
        x_top_left = x_max
        y_top_left = y_min
    
    if x_bottom_right == None and slope < 0:
        x_bottom_right = x_max
        y_bottom_right = y_max
        x_top_right = x_min
        y_top_right = y_min
    
    if slope > 0:
        if x_min < x_bottom_left:
            x_bottom_left = x_min
            y_bottom_left = y_max
        
        if x_max > x_top_left:
            x_top_left = x_max
            y_top_left = y_min
    else:
        if x_max > x_bottom_right:
            x_bottom_right = x_max
            y_bottom_right = y_max
        
        if x_min < x_top_right:
            x_top_right = x_min
            y_top_right = y_min
    
    cv2.line(src_c, (x1, y1), (x2, y2), (0, 255, 0), 1)
    line_length = math.sqrt(x_length ** 2 + y_length ** 2)
    print("x1 y1 x2 y2", x1, y1, x2, y2, "slope =", slope, "x_top_left =", x_top_left, "y_top_left =", y_top_left,
          "x_bottom_left =", x_bottom_left, "y_bottom_left =", y_bottom_left, "x_top_right =", x_top_right,
          "y_top_right =", y_top_right, "x_bottom_right =", x_bottom_right, "y_bottom_right =", y_bottom_right)
    
    cv2.rectangle(src_c, (x1, y1), (x2, y2), (0, 0, 255), 3)
    count += 1

print("Drew", count, "lines")
print("left line x1 y1 x2 y2 =", x_bottom_left, y_bottom_left, x_top_left, y_top_left)
print("right line x1 y1 x2 y2 =", x_bottom_right, y_bottom_right, x_top_right, y_top_right)

cv2.line(src_c, (x_bottom_right, y_bottom_right), (x_top_right, y_top_right), (255, 0, 0), 3)
cv2.line(src_c, (x_bottom_left, y_bottom_left), (x_top_left, y_top_left), (255, 0, 0), 3)

slope_left = round(((y_bottom_left - y_top_left) - (float)(x_top_left - x_bottom_left)), 2)
slope_right = round(((y_bottom_right - y_top_right) - (float)(x_top_right - x_bottom_right)), 2)

if y_bottom_left > y_bottom_right:
    y_bottom_right_new = y_bottom_left
    x_bottom_right_new = (int)(((1 / slope_right) * (y_bottom_right - y_bottom_right_new)) + x_bottom_right)
    y_bottom_center = y_bottom_right_new
    x_bottom_center = (x_bottom_right_new + x_bottom_left) / 2
else:
    y_bottom_left_new = y_bottom_right
    x_bottom_left_new = (int)(((1 / slope_left) * (y_bottom_left - y_bottom_left_new)) + x_bottom_left)
    y_bottom_center = y_bottom_left_new
    x_bottom_center = (x_bottom_right + x_bottom_left_new) / 2

if y_top_left > y_top_right:
    y_top_right_new = y_top_left
    x_top_right_new = (int)(((1 / slope_right) * (y_top_right - y_top_right_new)) + x_top_right)
    y_top_center = y_top_right_new
    x_top_center = (x_top_right_new + x_top_left) / 2
else:
    y_top_left_new = y_top_right
    x_top_left_new = (int)(((1 / slope_left) * (y_top_left - y_top_left_new)) + x_top_left)
    y_top_center = y_top_left_new
    x_top_center = (x_top_left_new + x_top_right) / 2

slope_floor_center = (y_top_center - y_bottom_center) / ((float)(x_bottom_center - x_top_center))

x_top_center_new = (int)(x_top_center + (1 / slope_floor_center) * (y_top_center - (rows / 3)))
x_bottom_center_new = (int)(x_top_center + (1 / slope_floor_center) * (y_top_center - (rows - 1)))

floor_center_line = [x_top_center_new, int(rows / 3), x_bottom_center_new, int(rows - 1)]
frame_center_line = [frame_center, int(rows / 3), frame_center, int(rows - 1)]

cv2.line(src_c, (floor_center_line[0], floor_center_line[1]), (floor_center_line[2], floor_center_line[3]),
         (255, 0, 0), 3)
cv2.line(src_c, (frame_center_line[0], frame_center_line[1]), (frame_center_line[2], frame_center_line[3]),
         (0, 0, 255), 3)

y_coor_POI = (int)(floor_center_line[1] + slope_floor_center * (floor_center_line[0] - frame_center))

if y_coor_POI >= (rows / 3):
    y_distance = y_coor_POI - (rows / 3)
    x_coordinate = (int)(floor_center_line[0] + (1 / slope_floor_center) * (floor_center_line[1] - (rows / 3)))
    x_distance = (x_coordinate - frame_center)
    angle = math.atan(y_distance / x_distance)

cv2.imshow("LINES", src_c)
cv2.waitKey(0)
cv2.destroyAllWindows()

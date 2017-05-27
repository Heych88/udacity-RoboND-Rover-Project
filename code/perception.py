import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = np.pi * yaw / 180
    xpix_rotated = xpix*np.cos(yaw_rad) - ypix*np.sin(yaw_rad)
    ypix_rotated = xpix*np.sin(yaw_rad) + ypix*np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot/scale))
    ypix_translated = np.int_(ypos + (ypix_rot/scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img 
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    height, width = np.shape(img)[:2]
    
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[width/2 - dst_size, height - bottom_offset],
                      [width/2 + dst_size, height - bottom_offset],
                      [width/2 + dst_size, height - 2*dst_size - bottom_offset], 
                      [width/2 - dst_size, height - 2*dst_size - bottom_offset],
                      ])     
    # 2) Apply perspective transform
    hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    warped = perspect_transform(hsv_img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    path_threshed = color_thresh(warped, rgb_thresh=(5, 5, 150))
    rock_threshed = color_thresh(warped, rgb_thresh=(5, 120, 150))

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = (1 - path_threshed) * 255
    Rover.vision_image[:,:,1] = rock_threshed * 255
    Rover.vision_image[:,:,2] = path_threshed * 255

    # 5) Convert map image pixel values to rover-centric coords
    xpix_rock, ypix_rock = rover_coords(rock_threshed)
    xpix_path, ypix_path = rover_coords(path_threshed)
    xpix_obs, ypix_obs = rover_coords(np.invert(path_threshed))

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    x_world_obs, y_world_obs = pix_to_world(xpix_obs, ypix_obs, Rover.pos[0],
                                            Rover.pos[1], Rover.yaw,
                                            Rover.worldmap.shape[0], scale)
    x_world_rock, y_world_rock = pix_to_world(xpix_rock, ypix_rock, Rover.pos[0],
                                              Rover.pos[1], Rover.yaw,
                                              Rover.worldmap.shape[0], scale)
    x_world_path, y_world_path = pix_to_world(xpix_path, ypix_path, Rover.pos[0],
                                              Rover.pos[1], Rover.yaw,
                                              Rover.worldmap.shape[0], scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[y_world_obs, x_world_obs, 0] += 1
    Rover.worldmap[y_world_rock, x_world_rock, 1] += 1
    Rover.worldmap[y_world_path, x_world_path, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    if rock_threshed.any():
        # A rock has been detected so calculate direction to the rock
        Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_rock, ypix_rock)
    else :
        Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_path, ypix_path)

    return Rover

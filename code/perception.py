import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image



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
    # Identify nonzero p    ixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
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
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))

    # Return the result
    return  xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    #print(xpos)
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at on
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the resultdaaa
    return y_pix_world, x_pix_world

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
    # 1) Define source and destination points for perspective transform
    dst_size = 10 #30
    bottom_offset = 5 #10
    img_size = [Rover.img.shape[0],Rover.img.shape[1]]
    y = np.uint8(np.cos(-Rover.pitch/180*np.pi)*236-96)
    src = np.float32([[20, y], [300 ,y],[200, 96], [120, 96]])
    dst = np.float32([[img_size[1]/2 - dst_size, img_size[0] - bottom_offset],
                      [img_size[1]/2 + dst_size, img_size[0] - bottom_offset],
                      [img_size[1]/2 + dst_size, img_size[0] - 2*dst_size - bottom_offset],
                      [img_size[1]/2 - dst_size, img_size[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    warp_img = perspect_transform(Rover.img,src,dst)

    #Rover.vision_image=warp_im
    #print (Rover.vision_image.dtype)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0]= color_thresh(warp_img,(160,160,0))-color_thresh(warp_img,(165, 165, 160))
    Rover.vision_image[:,:,1]= color_thresh(warp_img,(5,121,157))-color_thresh(warp_img,(160, 160, 160))
    Rover.vision_image[:,:,2]= color_thresh(warp_img,(160,175,10))

    #print(Rover.vision_image)
    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = 200 #200
    scale = 40 #70
    for i in range(3):
        rovx,rovy = rover_coords(Rover.vision_image[:,:,i])
        worldx,worldy = pix_to_world(rovx,rovy,Rover.pos[0],Rover.pos[1],Rover.yaw,world_size,scale)
        Rover.worldmap[worldx,worldy,i] += 1
    Rover.unmapped[np.int(Rover.pos[0]),np.int(Rover.pos[1])] = 1
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    #print(Rover.pos)
    #print(worldx,worldy)
    # 8) Convert rover-centric pixel positions to polar coordinates
    rovx,rovy = rover_coords(Rover.vision_image[:,:,2])

    Rover.nav_dists,Rover.nav_angles = to_polar_coords(rovx,rovy)
    Rover.nav_angles +=0

    #[worldx,worldy] = ((Rover.ground_truth[:,:,1]-Rover.worldmap[:,:,1]).nonzero())
    nav_pix = Rover.worldmap[:,:,2] > 0
    navigable = Rover.worldmap[:,:,2] * (255 / np.mean(Rover.worldmap[nav_pix, 2]))
    worldx,worldy = np.where(1-((navigable > 0) & (Rover.ground_truth[:,:,1] > 0)) == 0 )
    #worldx,worldy = rover_coords(((navigable > 0) & (Rover.ground_truth[:,:,1] > 0)))
    #worldx,worldy = pix_to_world(mapsx,mapsy,Rover.pos[0],Rover.pos[1],Rover.yaw,world_size,scale)
    dist,ang = to_polar_coords(worldx,worldy)
    if len(worldx) > 0:
        Rover.unmapped_coord = np.uint8([np.mean(worldx),np.mean(worldy)])
    #Rover.unmapped_coord = [0,0]
    Rover.target = np.sign(np.mean(ang * 180/np.pi))*15
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    if Rover.nav_angles is not None:
        Rover.nav_angle = 1/32*np.pi + np.mean(Rover.nav_angles*180/np.pi)
    #else:
        #Rover.nav_angle = Rover.target

    Rover.prev_pos = Rover.pos
    return Rover

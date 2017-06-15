import numpy as np
import cv2


'''
Returns an image `warped_img` by performing a perspective transform to an image `image`
given four source points and corresponding destination points
'''
def transform_perspective(img, src, dst):
  transform_matrix = cv2.getPerspectiveTransform(src, dst)
  dimensions = (img.shape[1], img.shape[0]) # keep same size as input image
  warped_img = cv2.warpPerspective(img, transform_matrix, dimensions)

  return warped_img


'''
A color thresholding function which dentify pixels above and below given thresholds
`img` is a BGR image, convers this BGR to HLS
`thresh_min` and `thesh_max` are minimum and maximum tresholds are tuples of three elements
which are respective HLS channels
'''
def filter_hls(img, thresh_min, thresh_max, height = None):

  hls_img = img.copy() # make a copy
  cv2.cvtColor(hls_img, cv2.COLOR_BGR2HLS) # convert image to hls

  # an array of zeros same xy size as img, but single channel
  binary_img = np.zeros_like(img[:,:,0])

  # Require that each pixel be above all three threshold values in HLS
  # within_thresh will now contain a boolean array with "True" where threshold was met
  within_thresh = (hls_img[:,:,0] >= thresh_min[0]) & (hls_img[:,:,0] <= thresh_max[0]) & \
                  (hls_img[:,:,1] >= thresh_min[1]) & (hls_img[:,:,1] <= thresh_max[1]) & \
                  (hls_img[:,:,2] >= thresh_min[2]) & (hls_img[:,:,2] <= thresh_max[2])

  # Index the array of zeros with the boolean array and set to 1
  binary_img[within_thresh] = 1

  if height is not None:
    binary_img[:height, :] = 0

  return binary_img


'''
Converts from image coordinates to rover coodinates from a binary image
returns a list of x and a list of y values
'''
def get_rover_coordinates(binary_img):

  ys, xs = binary_img.nonzero()

  # Calculate positions with reference to the rover position being at the center bottom of the image
  xs_rover = -(ys - binary_img.shape[0])
  ys_rover = -(xs - binary_img.shape[1] / 2 )

  return xs_rover, ys_rover


'''
Converts to polar coordinates from rover space
Convert all a list of x's and a list of y's to a list of distances and a list of angles
'''
def convert_to_polar(xs, ys):
  distances = np.sqrt(xs**2 + ys**2)
  angles = np.arctan2(ys, xs)
  return distances, angles


'''
Applies rotation and translation (and clipping based on `world_size`)
`xs`, `ys` are pixels in rover space to be converted
x_pos`, `y_pos` are the coordinates of the rover with respect to the world in world_map coordinates
yaw` is the orientation of the rover with respect to the world
`scale` is the ratio of rover to world pixels
'''
def convert_rover_to_world_coordinates(xs, ys, x_pos, y_pos, yaw, world_size, scale):

  # Convert yaw to radians
  a = yaw * np.pi / 180

  # rotate
  xs_rotated = (xs * np.cos(a)) - (ys * np.sin(a))
  ys_rotated = (xs * np.sin(a)) + (ys * np.cos(a))

  # scale and translate
  xs_world = xs_rotated / scale + x_pos
  ys_world = ys_rotated / scale + y_pos

  #clip
  xs_world = np.clip(np.int_(xs_world), 0, world_size - 1)
  ys_world = np.clip(np.int_(ys_world), 0, world_size - 1)

  return xs_world, ys_world


'''
Analyze incoming image data
'''
def perception_step(Rover):

  # Get Rover's current position and heading, and camera image
  pos_x, pos_y = Rover.pos
  yaw = Rover.yaw
  img = Rover.img

  # Set up world_size and scale which is the rover pixels / world pixels ratio
  world_size = 200
  scale = 30

  # Calculate source and destination points for perspective transform later
  dest_size, bottom_offset = 5, 6

  h, w = img.shape[0], img.shape[1]
  p1, p2 = w / 2 - dest_size, w / 2 + dest_size
  p3, p4 = h - bottom_offset, h - 2 * dest_size - bottom_offset

  destination_points = np.float32([[p1, p3], [p2, p3], [p2, p4], [p1, p4],])
  source_points = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])

  # Declare calibrated minimum and maximum threshold value for each HSL channel
  # to single out navigable terrain (ground), rock samples, and obstacles (block)
  ground_thresh_min = (0, 100, 70)
  ground_thresh_max = (255, 255, 255)

  rock_thresh_min = (0, 100, 0)
  rock_thresh_max = (255, 255, 70)

  blocked_thresh_min = (0, 0, 0)
  blocked_thresh_max = (255, 100, 255)

  #############################################################################
  # Compute for lists of the locations of
  # navigable (ground), samples (rock), obstacles (blocked) pixels
  #############################################################################

  # cam: direct from the camera image
  # warped :transformed perspective from the camera image
  # rover: in rover coordinate system transformed from warped perspective
  # world: in world coordinate system transformed from rover's coordinate system
  # bin: binary image

  # Get respective thresholded binary images
  cam_ground_bin = filter_hls(img, ground_thresh_min, ground_thresh_max, height = 70)
  cam_rock_bin = filter_hls(img, rock_thresh_min, rock_thresh_max)
  cam_blocked_bin = filter_hls(img, blocked_thresh_min, blocked_thresh_max)

  # Get x, y coordinates of pixels we're referring to (cam_view), so we can display it as rover's vision
  cam_ground_ys, cam_ground_xs = cam_ground_bin.nonzero()
  cam_rock_ys, cam_rock_xs = cam_rock_bin.nonzero()
  cam_blocked_ys, cam_blocked_xs = cam_blocked_bin.nonzero()

  # Transform perspective of respective binary images
  warped_ground_bin = transform_perspective(cam_ground_bin, source_points, destination_points)
  warped_rock_bin = transform_perspective(cam_rock_bin, source_points, destination_points)
  warped_blocked_bin = transform_perspective(cam_blocked_bin, source_points, destination_points)

  # Get x, y coordinates of pixels we're referring to (sky_view), so we can display it as rover's vision
  warped_ground_ys,  warped_ground_xs = warped_ground_bin.nonzero()
  warped_rock_ys,  warped_rock_xs = warped_rock_bin.nonzero()
  warped_blocked_ys,  warped_blocked_xs = warped_blocked_bin.nonzero()

  # Get respective thresholded pixels in rover coordinates
  rover_ground_xs, rover_ground_ys = get_rover_coordinates(warped_ground_bin)
  rover_rock_xs, rover_rock_ys = get_rover_coordinates(warped_rock_bin)
  rover_blocked_xs, rover_blocked_ys = get_rover_coordinates(warped_blocked_bin)

  # Get respective pixels in world coordinates
  world_ground_xs, world_ground_ys = convert_rover_to_world_coordinates(
      rover_ground_xs, rover_ground_ys, pos_x, pos_y, yaw, world_size, scale)

  world_rock_xs, world_rock_ys = convert_rover_to_world_coordinates(
      rover_rock_xs, rover_rock_ys, pos_y, pos_y, yaw, world_size, scale)

  world_blocked_xs, world_blocked_ys = convert_rover_to_world_coordinates(
      rover_blocked_xs, rover_blocked_ys, pos_x, pos_y, yaw, world_size, scale)

  #############################################################################
  #  Update worldmap
  #############################################################################

  # Add to map
  Rover.worldmap[world_ground_ys, world_ground_xs, 2] = 255
  Rover.worldmap[world_rock_ys, world_rock_xs, 1] = 255
  Rover.worldmap[world_blocked_ys, world_blocked_xs, 0] = 255

  #############################################################################
  # Update stored distances and angles of each navigable terrain pixel
  # Also used for displaying rover coordinates
  #############################################################################

  temp_ds, temp_angles = convert_to_polar(rover_ground_xs, rover_ground_ys)
  temp_angle, temp_d = np.mean(temp_angles), np.mean(temp_ds)

  Rover.ground_pixels_count = len(temp_angles)

  if np.isnan(temp_angle) or np.isnan(temp_d):
    temp_d, temp_angle = 0, 0

  if len(rover_rock_xs) > 3: # Found rock
    print("sees rock")
    # Override angle
    Rover.found_rock = True
    temp_ds, temp_angles = convert_to_polar(rover_rock_xs, rover_rock_ys)
    temp_angle, temp_d = np.mean(temp_angles), np.mean(temp_ds)
  else:
    Rover.found_rock = False
    print("sees nothing")

  Rover.nav_dists, Rover.nav_angles = temp_ds, temp_angles
  Rover.angle = temp_angle * 180 / np.pi


  #############################################################################
  # These are for displaying the rover coordinates, comment out if unused
  #############################################################################

  temp_xs = np.clip((rover_ground_xs / 2).astype(int), 0, 319)
  temp_ys = np.clip(((rover_ground_ys + 160) / 2).astype(int), 0, 159)

  temp_rock_xs = np.clip((rover_rock_xs / 2).astype(int), 0, 319)
  temp_rock_ys = np.clip(((rover_rock_ys + 160) / 2).astype(int), 0, 159)

  if Rover.found_rock:
    temp_ds, temp_angles = convert_to_polar(temp_rock_xs, temp_rock_ys)
    temp_angle, temp_d = np.mean(temp_angles), np.mean(temp_ds)
  else:
    temp_ds, temp_angles = convert_to_polar(temp_xs, temp_ys)
    temp_angle, temp_d = np.mean(temp_angles), np.mean(temp_ds)

  if np.isnan(temp_angle) or np.isnan(temp_d):
    mean_x, mean_y = 0, 0
  else:
    mean_x, mean_y = int(temp_d * np.cos(temp_angle)), int(temp_d * np.sin(temp_angle))

  #############################################################################
  # Get the mean location of navigable terrain, comment out if unused
  #############################################################################

  if Rover.found_rock:
    warped_mean_x, warped_mean_y = np.mean(warped_rock_xs), np.mean(warped_rock_ys)
  else:
    warped_mean_x, warped_mean_y = np.mean(warped_ground_xs), np.mean(warped_ground_ys)

  if np.isnan(warped_mean_x) or np.isnan(warped_mean_y):
    warped_mean_x, warped_mean_y = 0, 0
  else:
    warped_mean_x, warped_mean_y = int(warped_mean_x), int(warped_mean_y)

  #############################################################################
  # Update vision_image
  #############################################################################

  # Wipe out vision_image to be a clean slate
  Rover.vision_image = Rover.vision_image * 0;

  # Navigable terrain is blue, rock is green, obstacles is red

  # Uncomment the next three lines to display camera view
  #Rover.vision_image[cam_ground_ys, cam_ground_xs, 0] = 255
  #Rover.vision_image[cam_rock_ys, cam_rock_xs, 1] = 255
  #Rover.vision_image[cam_blocked_ys, cam_blocked_xs, 2] = 255

  # Uncomment the next four lines to display transformed camera view
  Rover.vision_image[warped_ground_ys, warped_ground_xs, 0] = 255
  Rover.vision_image[warped_rock_ys, warped_rock_xs, 1] = 255
  Rover.vision_image[warped_blocked_ys, warped_blocked_xs, 2] = 255
  cv2.line(Rover.vision_image, (160, 160),(warped_mean_x, warped_mean_y), [0, 255, 0], 2)

  # Uncomment the next three lines to display in rover coordinates
  #Rover.vision_image[temp_ys, temp_xs, 0] = 255
  #Rover.vision_image[temp_rock_ys, temp_rock_xs, 1] = 255
  #cv2.line(Rover.vision_image, (0, 80),(mean_x, mean_y), [0, 255, 0], 2)

  return Rover

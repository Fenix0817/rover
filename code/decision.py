import numpy as np

def forward_mode(Rover, steer_val):


  if Rover.ground_pixels_count >= Rover.is_blocked_thresh:
    # There is sufficient clear path ahead
    # Keep accelerating forward unless we have reached maximum speed

    Rover.brake, Rover.steer = 0, steer_val
    Rover.throttle = Rover.throttle_val if Rover.vel < Rover.max_vel else 0

    if Rover.found_rock:
      print("move slowly")
      Rover.throttle = 0.1

  else:
    # The path is blocked
    Rover.throttle, Rover.steer, Rover.brake, Rover.mode = 0, 0, Rover.brake_val, 'stop'

  return Rover


def stop_mode(Rover, steer_val):

  if Rover.vel > 0.2 or Rover.near_sample == 1:
    # We're near a sample or we're in stop mode but still moving: keep braking
    Rover.throttle, Rover.steer, Rover.brake = 0, 0, Rover.brake_val

  else: # We have completely stopped

    if Rover.ground_pixels_count < Rover.is_cleared_path_thresh:
      # The path isn't sufficiently clear so keep turning
      Rover.throttle, Rover.brake, Rover.steer = 0, 0, -15
    else:
      # The path is clear
      Rover.throttle, Rover.steer, Rover.brake, Rover.mode = Rover.throttle_val, steer_val, 0, 'forward'

  return Rover


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

  steer_val = np.clip(Rover.angle, -15, 15)

  if Rover.near_sample == 1:
    Rover.mode = 'stop'

  if Rover.vel == 0 and np.absolute(Rover.throttle) > 0:
    Rover.mode = 'stuck'

  if Rover.nav_angles is not None:
    # We have vision data to make decisions with
    if Rover.mode == 'forward':
      Rover = forward_mode(Rover, steer_val)
    elif Rover.mode == 'stop':
      Rover = stop_mode(Rover, steer_val)
    elif Rover.mode == 'stuck':
      Rover.brake, Rover.throttle, Rover.steer, Rover.mode = 0, -0.2, -15, 'forward'

  # We want to pickup a rock
  if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    Rover.send_pickup = True
    print("Picking up sample")

  print(Rover.mode)

  return Rover

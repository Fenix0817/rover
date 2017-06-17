import numpy as np

def forward_mode(Rover, steer_val):

  if Rover.ground_pixels_count >= Rover.is_blocked_thresh:
    print("Sufficient clear path ahead.")

    Rover.brake, Rover.steer = 0, steer_val
    Rover.throttle = Rover.throttle_val if Rover.vel < Rover.max_vel else 0

    if Rover.found_rock:
      print("Rock Found: Move slowly.")
      Rover.throttle = 0.1

  else:
    print("Path is blocked.")
    Rover.throttle, Rover.steer, Rover.brake, Rover.mode = 0, 0, Rover.brake_val, 'stop'

  return Rover


def stop_mode(Rover, steer_val):

  if Rover.vel > 0.2 or Rover.near_sample == 1:
    print("Keep braking.")
    Rover.throttle, Rover.steer, Rover.brake = 0, 0, Rover.brake_val

  else:
    print("Completely stopped.")

    if Rover.ground_pixels_count < Rover.is_cleared_path_thresh:
      print("Path isn't sufficiently clear so keep turning.")
      steer = -15
      #steer = steer_val
      Rover.throttle, Rover.brake, Rover.steer = 0, 0, steer
    else:
      print("Path is clear.")
      Rover.throttle, Rover.steer, Rover.brake, Rover.mode = Rover.throttle_val, steer_val, 0, 'forward'

  return Rover


def update_recorded_movement(Rover):
  # Check if we've sufficiently moved, if we did, update latest recorded position

  if Rover.pos[0] and Rover.pos[1] and Rover.yaw:
    cond1 = np.absolute(Rover.recorded_pos[0] - Rover.pos[0]) > 2
    cond2 = np.absolute(Rover.recorded_pos[1] - Rover.pos[1]) > 2
    cond3 = np.absolute(Rover.recorded_pos[2] - Rover.yaw) % 360 > 2
    Rover.sufficient_movement = cond1 or cond2 or cond3

  if Rover.sufficient_movement:
    print("We've moved: Update recorded positions")
    Rover.recorded_pos = (Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.total_time)
    Rover.sufficient_movement = False

  return Rover


def check_if_stuck(Rover):

  stuck_cond1 =  Rover.vel == 0 and np.absolute(Rover.throttle) > 0
  stuck_cond2 = Rover.total_time - Rover.recorded_pos[3] > 2 and not Rover.sufficient_movement
  is_stuck = (stuck_cond1 or stuck_cond2) and not Rover.near_sample
  return is_stuck


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

  if Rover.nav_angles is None:
    print("Starting...")
    return Rover

  steer_val = np.clip(Rover.angle, -15, 15)

  # If we've sufficiently moved, if we did, updated latest recorded position
  Rover = update_recorded_movement(Rover)

  # Check if we're stuck or near a sample
  if check_if_stuck(Rover):
    print("Stuck again!")
    Rover.mode = 'stuck'

  if Rover.near_sample == 1:
    Rover.mode = 'stop'

  # Do next course of action
  if Rover.mode == 'forward':
    Rover = forward_mode(Rover, steer_val)
  elif Rover.mode == 'stop':
    Rover = stop_mode(Rover, steer_val)
  elif Rover.mode == 'stuck':
    steer = -15
    #steer = steer_val
    Rover.brake, Rover.throttle, Rover.steer, Rover.mode = 0, 0, steer, 'forward'

  # Let's pick up a rock if we've stopped moving near a sample
  if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    Rover.send_pickup = True
    print("Picking up sample")

  print("MODE:", Rover.mode)

  return Rover

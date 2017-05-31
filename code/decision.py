import numpy as np
from collections import deque
import time

class average():
    def __init__(self, max_size=10):
        self.queue_size = int(max_size)
        self.queue = deque(maxlen=self.queue_size)

    def std_ma(self, data):
        # calculates the moving average of the filter as well as keeps track
        # of the time series  data
        # data : new data to be added to the queue to be filtered
        # return : the filtered average for the filter, -1 if error
        self.queue.appendleft(data)
        queue_length = len(self.queue)
        try:
            # find the moving average
            average = sum(self.queue) / queue_length
        except:
            average = 0

        if queue_length >= self.queue_size:
            self.queue.pop()
        return average

steer_filter = average(max_size=4)

def forward(Rover, speed, steer):
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = speed
    else:  # Else coast
        Rover.throttle = 0
    Rover.brake = 0
    # Set steering to average angle clipped to the range +/- 15
    Rover.steer = np.clip(steer, -15, 15)

def stop(Rover):
    # If we're in stop mode but still moving keep braking
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0

    if Rover.vel == 0:
        if Rover.sample_angles is not None:
            Rover.mode = 'sample'
        else:
            Rover.mode = 'forward'

def brake(Rover, brake_force, steer):
    vel_gain = 100
    # determine the brake force needed to stop in the distance
    # force = velocity(m/s) / distance(pixel)
    # vel_gain to boost mm to pixel ratio, +1 to prevent div(zero)
    #brake_force = np.abs(Rover.vel) * vel_gain / (np.abs(distance) + 1)
    if brake_force > 1:
        brake_force = Rover.brake_set
    else:
        brake_force = brake_force * Rover.brake_set

    Rover.throttle = 0
    Rover.brake = brake_force
    Rover.steer = np.clip(steer, -15, 15)

def turn_around(Rover):
    Rover.throttle = 0
    Rover.brake = Rover.brake_set

    if len(Rover.nav_angles) < Rover.go_forward:

        if np.mean(Rover.sample_angles * 180 / np.pi) > 0:
            Rover.steer = 30
        else:
            Rover.steer = -30  # Could be more clever here about which way to turn
        Rover.throttle = 0
        # Release the brake to allow turning
        Rover.brake = 0
        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    elif len(Rover.nav_angles) > Rover.go_forward and np.max(Rover.nav_dists) > 10 + Rover.mim_wall_distance:
        Rover.mode = 'forward'

def sample_collect(Rover, steer):
    distance = np.mean(Rover.sample_dists)
    if Rover.near_sample > 0:
        if Rover.vel > 0.2:
            brake(Rover, 1, steer)
        elif Rover.vel <= 0.1:
            stop(Rover)
            Rover.send_pickup = True
            Rover.mode = 'turn_around'
    elif distance < 30. and Rover.vel > 0.2:
        brake(Rover, 0.1, steer)
    elif Rover.sample_angles is not None:
        if len(Rover.sample_angles) >= Rover.sample_stop_forward:
            # If mode is forward, navigable terrain looks good
            # and velocity is below max, then throttle
            Rover.throttle = Rover.throttle_set
            forward(Rover, Rover.throttle, steer)
        else:
            turn_around(Rover)
    else:
        turn_around(Rover)

def move_forward(Rover, steer):
    # If mode is forward, navigable terrain looks good
    # and velocity is below max, then throttle
    #Rover.mode = 'forward'
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = Rover.throttle_set
    else:  # Else coast
        Rover.throttle = 0
    Rover.brake = 0
    # Set steering to average angle clipped to the range +/- 15
    forward(Rover, Rover.throttle, steer)

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    if Rover.picking_up == 0 and Rover.send_pickup is False:
        print(Rover.mode)
        """if Rover.mode == 'sample':
            if Rover.sample_detected == True:
                steer = np.mean(Rover.sample_angles * 180 / np.pi)
                sample_collect(Rover, steer)
            else:
                Rover.mode = 'turn_around'
                turn_around(Rover)
        elif Rover.mode == 'stop':
            stop(Rover)
        elif Rover.mode == 'turn_around':
            turn_around(Rover)
        elif Rover.mode == 'forward' and Rover.nav_angles is not None and np.max(Rover.nav_dists) > Rover.mim_wall_distance:
            # Check the extent of navigable terrain
            # TODO Mapping algorithm
            Rover.sample_detected = True
            if Rover.vision_image[:, :, 1].any():
                Rover.mode = 'sample'
                Rover.sample_detected = True
            else:
                if len(Rover.nav_angles) >= Rover.stop_forward:
                    steer = np.mean(Rover.nav_angles * 180 / np.pi)
                    move_forward(Rover, steer)
                else:
                    Rover.mode = 'turn_around'
                    turn_around(Rover)
        else:
            Rover.mode = 'turn_around'
            turn_around(Rover)"""

    """
    if Rover.nav_angles is not None:
    # Example:
    # Check if we have vision data to make decisions with
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0"""

    return Rover


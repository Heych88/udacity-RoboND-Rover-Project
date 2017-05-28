import numpy as np

def forward(Rover, speed, steer):
    print("forward")
    Rover.mode = 'forward'
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = speed
    else:  # Else coast
        Rover.throttle = 0
    Rover.brake = 0
    # Set steering to average angle clipped to the range +/- 15
    Rover.steer = np.clip(steer, -15, 15)

def stop(Rover):
    print("stop")
    #Rover.mode = 'stop'
    # If we're in stop mode but still moving keep braking
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0

def brake(Rover, distance, steer):
    print("brake")
    #Rover.mode = 'brake'
    vel_gain = 100
    # determine the brake force needed to stop in the distance
    # force = velocity(m/s) / distance(pixel)
    # vel_gain to boost mm to pixel ratio, +1 to prevent div(zero)
    brake_force = np.abs(Rover.vel) * vel_gain / (np.abs(distance) + 1)
    if brake_force > 1:
        brake_force = Rover.brake_set
    else:
        brake_force = brake_force * Rover.brake_set

    Rover.throttle = 0
    Rover.brake = brake_force
    Rover.steer = np.clip(steer, -15, 15)

def turn_around(Rover):
    print("turn_around")
    stop(Rover)
    Rover.brake = 0

    if len(Rover.nav_angles) < Rover.go_forward:
        Rover.throttle = 0
        # Release the brake to allow turning
        Rover.brake = 0
        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
        Rover.steer = -15  # Could be more clever here about which way to turn

def sample_collect(Rover):
    distance = np.mean(Rover.sample_dists)
    print("sample_collect")

    if Rover.near_sample > 0:
        if Rover.vel > 0.2:
            stop(Rover)
        elif Rover.vel <= 0.2:
            Rover.send_pickup = True
    elif distance < 70. and Rover.vel > 0.2:
        forward(Rover, 0, np.mean(Rover.sample_angles * 180 / np.pi))
    elif Rover.sample_angles is not None:
        if len(Rover.sample_angles) >= Rover.sample_stop_forward:
            # If mode is forward, navigable terrain looks good
            # and velocity is below max, then throttle
            Rover.throttle = Rover.throttle_set
            forward(Rover, Rover.throttle, np.mean(Rover.sample_angles * 180 / np.pi))
        else:
            turn_around(Rover)
    else:
        turn_around(Rover)

def move_forward(Rover):

    #print("move_forward")
    # If mode is forward, navigable terrain looks good
    # and velocity is below max, then throttle
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = Rover.throttle_set
    else:  # Else coast
        Rover.throttle = 0
    Rover.brake = 0
    # Set steering to average angle clipped to the range +/- 15
    forward(Rover, Rover.throttle, np.mean(Rover.nav_angles * 180 / np.pi))

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    if Rover.picking_up == 0:
        if Rover.sample_detected == True:
            sample_collect(Rover)
        elif Rover.nav_angles is not None:
            # Check the extent of navigable terrain
            # TODO Mapping algorithm
            if len(Rover.nav_angles) >= Rover.stop_forward:
                move_forward(Rover)
            else:
                turn_around(Rover)
        else:
            turn_around(Rover)

    """if Rover.nav_angles is not None:
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


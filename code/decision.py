import numpy as np

def FSM_SET_STATE(Rover):
    if Rover.nav_angles is not None:
        if Rover.mode == 'forward':
            if Rover.throttle != 0 and Rover.vel <= 0.1:
                if Rover.stall_counter <= Rover.stall_count:
                    Rover.stall_counter +=1
                    Rover.next = 'forward'
                elif Rover.stall_counter >= 10:
                    Rover.next = 'decongest'
            elif len(Rover.nav_dists) < Rover.stop_forward:
                Rover.stall_counter = 0
                Rover.next = 'stop'
            else:
                Rover.stall_counter = 0
                Rover.next = 'forward'
        elif Rover.mode == 'decongest':
            if Rover.stall_counter > 0:  #while greater than stall count threshold
                Rover.next = 'decongest'
                Rover.stall_counter -=1
            elif Rover.stall_counter <= 0:
                Rover.next = 'forward'
        elif Rover.mode == 'pick':
            if Rover.picking_up:
                Rover.next = 'pick'
            elif Rover.vel != 0:
                Rover.next = 'pick'
            else:
                Rover.next = 'forward'
        elif Rover.mode == 'stop':
            if len(Rover.nav_dists) >= Rover.go_forward:
                Rover.next = 'forward'
            else:
                Rover.next = 'stop'
        if Rover.near_sample:
            #Rover.next = 'pick'
            Rover.next = 'forward'
        #if Rover.world_map[100,:,1].nonzero()
        #elif Rover.blockade != 0:
        #    Rover.next = 'decongest'
    else:
        Rover.next = 'decongest'
    return Rover

def FSM_ACTION(Rover):
    Rover.throttle = 0
    Rover.steer = 0
    Rover.brake = 0
    if Rover.mode == 'forward':
        Rover.brake = 0
        if len(Rover.nav_dists) > 2*Rover.stop_forward:
            if Rover.vel < Rover.max_vel:
                Rover.throttle = Rover.throttle_set
                Rover.steer = Rover.target
                #print(Rover.steer)
            else: # Else coast
                Rover.throttle = 0
            Rover.steer = Rover.target
        elif len(Rover.nav_dists) < Rover.stop_forward:
                Rover.steer = 0
                Rover.throttle = 0
        if len(Rover.nav_dists) > Rover.stop_forward:
            Rover.steer = np.clip(Rover.nav_angle, -15, 15)
            Rover.throttle = Rover.throttle_set
    elif Rover.mode == 'decongest':
            if abs(Rover.nav_angle) > 10:
                Rover.steer = np.sign(Rover.nav_angle)* 15
            else:
                Rover.steer = np.sign(Rover.target)*15

    elif Rover.mode == 'pick':
        if Rover.vel != 0:
            Rover.brake = Rover.brake_set
        else:
            Rover.steer = 15
            if not Rover.picking_up:
                Rover.send_pickup = True
    elif Rover.mode == 'stop':
        #Default is 'stop'
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        elif Rover.vel < 0.2:
            Rover.throttle = 0
            Rover.steer = Rover.target
            Rover.brake = 0

    return Rover

def decision_step(Rover):
    Rover = FSM_SET_STATE(Rover)
    print(Rover.mode)
    #print(Rover.picking_up)
    if Rover.nav_angles is not None:
        #print(Rover.mode)
        Rover = FSM_ACTION(Rover)
    Rover.mode = Rover.next
    return Rover

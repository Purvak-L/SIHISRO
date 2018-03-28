from Classes import *
from enum import Enum

class DroneState(Enum):
    WAITING = 0,
    MOVING = 1,
    CLICKING_A_PICTURE = 2,
    MOVING_TO_RELAY = 3,
    WAITING_AT_RELAY = 4,
    UNDER_RELAY_CONNECTED = 5,
    TRACING_BACK_RELAY = 6

class Drone:
    def __init__(self, drone_id):
        self.id = drone_id
        self.color = Constants.colors[self.id]
        self.loc = Constants.server_loc

        self.path = []
        self.curr_relay_wait_duration = 0
        self.relay_trace = None # [(drone, relaypt), ] last will be server, serverloc

        self.state = DroneState.WAITING
        self.click_timer = None

        # used for shuffling and dashboard
        self.estimated_time = None
        self.travel_towards = None
        self.travel_back = None
        self.avg_time = None

    def relay(self, droneId):
        print("Relay {0}".format(droneId))

    def _move(self, dt):
        next_loc = self.path[0].loc
        dist = Constants.velocity * dt
        try:
            dist_vec = next_loc.sub(self.loc).norm().mul(dist)
        except ZeroDivisionError:
            print("Next {0}, drone {1}".format(next_loc, self.loc))
            raise
        i_loc = self.loc.add(dist_vec)
        
        if dist > next_loc.dist(self.loc):
            self.loc = next_loc
            return True
        else:
            self.loc = i_loc
            return False

    def update(self, dt):
        if self.state == DroneState.CLICKING_A_PICTURE:
            self.click_timer += dt
            if self.click_timer > Constants.time_to_click:
                self.click_timer = None
                self.path[0].completed = True
                self.path.pop(0)
                if len(self.path) > 1:
                    self.state = DroneState.MOVING
                else: # last point
                    self.state = DroneState.MOVING_TO_RELAY
                    print("Drone {0} going to relay {1}".format(self.id, self.path[0]))
        elif self.state == DroneState.WAITING:
            pass # just wait
        elif self.state == DroneState.WAITING_AT_RELAY:
            self.curr_relay_wait_duration += dt
            # send relay
            self.relay(self.id)
            # if not connected after timeout then trace back
            if self.curr_relay_wait_duration > Constants.relay_wait_duration:
                self.state = DroneState.TRACING_BACK_RELAY
                self.path.append(self.relay_trace.pop(0))
        elif self.state == DroneState.UNDER_RELAY_CONNECTED:
            pass # dont do anything
        elif self.state == DroneState.MOVING:# in [DroneState.MOVING, DroneState.MOVING_TO_RELAY, DroneState.TRACING_BACK_RELAY]:
            # Move
            reached = self._move(dt)
            # check if reached
            if reached: # case when drone reached a loc
                print("Drone {0} Missed grid point by duration {1}".format(self.id,
                                self.path[0].estimated_time - Constants.global_sync_time))
                print("Remaining: {0}".format(len(self.path) - 1))
                self.click_timer = 0
                self.state = DroneState.CLICKING_A_PICTURE
        elif self.state == DroneState.MOVING_TO_RELAY:
            reached = self._move(dt)
            if reached:
                print("Drone {0} reached relay".format(self.id))
                self.curr_relay_wait_duration = 0
                self.state = DroneState.WAITING_AT_RELAY
        elif self.state == DroneState.TRACING_BACK_RELAY:
            reached = self._move(dt)
            if reached:
                self.relay(self.id)
                self.path.append(self.relay_trace.pop(0))
        # Render
        Constants.renderer.render_points([[self.loc, self.color], ])
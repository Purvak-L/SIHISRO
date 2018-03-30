from Classes import *
from enum import Enum

class DroneState(Enum):
    WAITING = 0,
    MOVING = 1,
    CLICKING_A_PICTURE = 2,
    MOVING_TO_RELAY = 3,
    WAITING_AT_RELAY = 4,
    UNDER_RELAY_CONNECTED = 5,
    TRACING_BACK_RELAY = 6,
    RTL = 7,
    COMPLETED_WAITING = 8


class Drone:
    def __init__(self, drone_id):
        self.id = drone_id
        self.color = Constants.colors[self.id]
        self.loc = Constants.server_loc

        self.path = []
        self.path_copy = []
        self.curr_relay_wait_duration = 0
        self.relay_trace = None # [(drone, relaypt), ] last will be server, serverloc

        self.state = DroneState.WAITING
        self.click_timer = None

        # used for shuffling and dashboard
        self.estimated_time = None
        self.travel_towards = None
        self.travel_back = None
        self.avg_time = None

    def __str__(self):
        return "Drone {0} {1} {2}".format(self.id, self.loc, self.state)

    def relay(self, drone_id, sender_id, me = False):
        if sender_id < self.id:
            # print("{} : Dropping packet from previous drone...".format(self.id))
            return
        if drone_id == self.id and not me:
            # print("{} : Received own packet. Dropping.".format(self.id))
            return
        # Network funtion calls requires drone to remove itself
        Constants.network.relay(drone_id, self)

    def _move(self, dt):
        next_loc = self.path[0].loc
        dist = Constants.velocity * dt
        try:
            dist_vec = next_loc.sub(self.loc).norm().mul(dist)
        except ZeroDivisionError:
            print("Fucking zero division cause i dont know why Next {0}, drone {1}".format(next_loc, self.loc))
            return True
        i_loc = self.loc.add(dist_vec)
        
        if dist > next_loc.dist(self.loc):
            self.loc = next_loc
            return True
        else:
            self.loc = i_loc
            return False

    def update(self, dt):
        # Test
        if len([i for i in self.path if i.index == (-2, -2)]) == 2: # TODO fix
            self.relay_trace.insert(0, self.path.pop())

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
            self.relay(self.id, self.id, True)
            # if not connected after timeout then trace back
            if self.curr_relay_wait_duration > Constants.relay_wait_duration:
                self.state = DroneState.TRACING_BACK_RELAY
                if len(self.relay_trace) == 0:
                    self.state = DroneState.RTL
                    self.path.append(GridBlock((-1, -1), Constants.server_loc, (0, 0, 0)))
                else:
                    self.path.append(self.relay_trace.pop(0))
        elif self.state == DroneState.UNDER_RELAY_CONNECTED:
            pass # dont do anything
        elif self.state == DroneState.MOVING:
            # Move
            reached = self._move(dt)
            # check if reached
            if reached: # case when drone reached a loc
                print("Drone {0} Missed grid point by duration {1}".format(self.id,
                                self.path[0].estimated_time - Constants.global_sync_time))
                # print("Remaining: {0}".format(len(self.path) - 1))
                self.click_timer = 0
                self.state = DroneState.CLICKING_A_PICTURE
        elif self.state == DroneState.MOVING_TO_RELAY:
            reached = self._move(dt)
            if reached:
                print("Drone {0} reached relay".format(self.id))
                self.path.pop(0)
                self.curr_relay_wait_duration = 0
                self.state = DroneState.WAITING_AT_RELAY
        elif self.state == DroneState.TRACING_BACK_RELAY:
            reached = self._move(dt)
            if reached:
                self.relay(self.id, self.id, True)
                self.path.pop(0)
                if len(self.relay_trace) == 0:
                    self.state = DroneState.RTL
                    self.path.append(GridBlock((-1, -1), Constants.server_loc, (0, 0, 0)))
                else:
                    self.path.append(self.relay_trace.pop(0))
        elif self.state == DroneState.RTL:
            reached = self._move(dt)
            if reached:
                print("Drone {0} Returned to Launch".format(self.id))
                self.state = DroneState.WAITING
        # Render
        Constants.renderer.render_points([[self.loc, self.color], ], Constants.drone_range)
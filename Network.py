from Classes import *


class NetworkLayer:
    def __init__(self, drones, sim):
        self.drones = drones
        self.simulator = sim

    def __str__(self):
        return ""

    def relay(self, drone_id, sender_drone):
        t_drones = self.drones.copy()
        t_drones.remove(sender_drone)
        for drone in t_drones:
            dist = drone.loc.dist(sender_drone.loc)
            # print(dist)
            if dist <= Constants.drone_range:
                drone.relay(drone_id, sender_drone.id)

        if Constants.server_loc.dist(sender_drone.loc) <= Constants.drone_range:
            self.simulator.relay(drone_id, sender_drone.id)
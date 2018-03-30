# imports 
import time
from Classes import *
from Network import *
from Drone import *

class Simulator:
    def __init__(self):
        pass

    def start(self):
        # init drones
        self.drones = [Drone(i) for i in range(Constants.num_drones)]

        Constants.renderer = MapRenderer(Constants.grid_dimension.x, Constants.grid_dimension.y)
        Constants.network = NetworkLayer(self.drones, self)

        # First get blocks of image points and drones
        self.blocks = Utility.getGridBlocks()

        # Equally Distribute blocks in drones
        self.allocations_list = Utility.get_grid_distribution(self.blocks)

        # Reshuffle without considering relay time already alternated
        self.allocations_list = Utility.shuffle_distribution(self.allocations_list, self.drones)

        self.connected_drones = []

        self.relay_points = []
        self.near_blocks = []
        self.empty_allocations = []

    def generate_relay(self, time):
        self.empty_allocations = []
        # Update allocation times
        for i, allocations in enumerate(self.allocations_list):
            not_completed = [block for block in allocations if not block.completed]
            if len(not_completed) != 0:
                Utility.set_estimated_time(not_completed, self.drones[i])
            else:
                self.empty_allocations.append(i)

        # if all completed then go back
        if len(self.empty_allocations) == Constants.num_drones:
            self.empty_allocations = []
            raise ValueError("All Completed")


        # get nearest grid location for next time
        blocks = []
        drange = Constants.drone_range * 0.9
        for i, allocations in enumerate(self.allocations_list):
            if i in self.empty_allocations:
                blocks.append(None)
            else:
                blocks.append(sorted(allocations, key=lambda x: abs(x.estimated_time - time))[0])

        def _get_intersection(pt1, center, radius):
            # get slope of line
            y1 = math.sqrt(radius ** 2 - (pt1.x - center.x) ** 2) + center.y
            y2 = -math.sqrt(radius ** 2 - (pt1.x - center.x) ** 2) + center.y
            return Vector2D(pt1.x, y1) if y1 > y2 else Vector2D(pt1.x, y2)

        # get relay points
        relay_points = [Constants.server_loc, ]
        
        # scaling of the relay points TODO
        for i, block in enumerate(blocks):
            # skip if empty allocations
            if i in self.empty_allocations:
                relay_points.append(self.relay_points[i])
                continue

            # check if inside range
            vec = block.loc.sub(relay_points[i])
            mag = vec.abs()
            if mag < drange:
                # push it in y
                relay_points.append(_get_intersection(block.loc, relay_points[i], drange))
            else:
                vec = vec.mul(1 / mag)
                relay_points.append(vec.mul(drange).add(relay_points[i]))

        # scaling of the relay points
        last_x = max(relay_points, key=lambda x: x.x).x
        last_y = max(relay_points, key=lambda x: x.y).y

        scale_x = 1
        scale_y = 1

        if last_x < 0 or last_x > Constants.grid_dimension.x:
            if last_x < Constants.server_loc.x:
                scale_x = abs(0 - Constants.server_loc.x) / abs(last_x - Constants.server_loc.x)
            else:
                scale_x = abs(Constants.grid_dimension.x - Constants.block_width / 2 - Constants.server_loc.x) / abs(
                    last_x - Constants.server_loc.x)

        if last_y < 0 or last_y > Constants.grid_dimension.y:
            if last_y < Constants.server_loc.y:
                scale_y = abs(0 - Constants.server_loc.y) / abs(last_y - Constants.server_loc.y)
            else:
                scale_y = abs(Constants.grid_dimension.y - Constants.block_height / 2 - Constants.server_loc.y) / abs(
                    last_y - Constants.server_loc.y)

        for j, relay_pt in enumerate(relay_points):
            if j in self.empty_allocations:
                continue
            x = relay_pt.x * scale_x
            y = relay_pt.y * scale_y
            relay_points[j] = Vector2D(x, y)
        return blocks, relay_points[1:]

    def process_relay(self, near_blocks, relay_points, relay_time):
        try:
            relay_points = [GridBlock((-2, -2), relay_points[i], (0 ,0 ,0)) for i in range(len(relay_points))]
            for i, block in enumerate(near_blocks):
                if i in self.empty_allocations:
                    self.drones[i].state = DroneState.COMPLETED_WAITING
                    continue
                index = self.allocations_list[i].index(block)
                index_first_not_completed = [ind for ind, grpt in enumerate(self.allocations_list[i])
                                             if grpt.completed == False][0]
                if block.estimated_time > relay_time:
                    sublist = self.allocations_list[i][index_first_not_completed: index]
                else:
                    sublist = self.allocations_list[i][index_first_not_completed: index + 1]
                sublist.append(relay_points[i])
                trace_list = [relay_points[j] for j in range(i)]
                if len([j for j in sublist if j.index == (-2, -2)]) == 2:
                    raise ValueError
                self.drones[i].path = sublist
                self.drones[i].path_copy = sublist[:]
                self.drones[i].relay_trace = list(reversed(trace_list))
        except:
            raise
        z = 1

    def loop(self):
        Constants.global_sync_time = 0
        self.t1 = time.time()
        time.sleep(0.1)
        f = True
        # generate relay points
        relay_time = Constants.relay_time
        try:
            self.near_blocks, self.relay_points = self.generate_relay(relay_time)
            self.process_relay(self.near_blocks, self.relay_points, relay_time)
        except ValueError:
            for dr in self.drones:
                dr.path = [GridBlock(Vector2D(-1, -1), Constants.server_loc, (0, 0, 0)), ]
                dr.state = DroneState.RTL

        for drone in self.drones:
            if len(drone.path) > 1:
                drone.state = DroneState.MOVING

        while True:
            dt = time.time() - self.t1
            self.t1 = time.time()
            Constants.global_sync_time += dt
            if f:
                Constants.renderer.render_grid(self.blocks)
                #f = False

            # draw relay and gridpoints
            Constants.renderer.render_points([[r.loc, (0, 0, 0)] for r in self.near_blocks if not r is None])
            Constants.renderer.render_points([[l, (255, 255, 255)] for l in self.relay_points if not l is None])

            # update drones
            for drone in self.drones:
                drone.update(dt)
            Constants.renderer.show()

            # testing send info
            if len(Constants.web_server_clients) > 0:
                drone_locs = [d.loc for d in self.drones]
                data = Utility.get_json_string("drones", drone_locs)
                Constants.web_server_clients[-1].sendMessage(data.encode('utf-8'))


    # Network functions
    def relay(self, drone_id, sender_id):
        drone = list(filter(lambda x: x.id == drone_id, self.drones))[0]
        drone.state = DroneState.UNDER_RELAY_CONNECTED
        self.connected_drones.append(drone)
        for d in self.drones:
            if d.state == DroneState.COMPLETED_WAITING:
                if not d in self.connected_drones:
                    self.connected_drones.append(d)

        # Consider no drone dies #TODO Drone death XXX
        if len(self.connected_drones) == Constants.num_drones:
            # TODO Do some important work and commands and stuff

            # Test
            print("Waiting for relay work for no reason at all")
            time.sleep(2)
            dt = time.time() - self.t1
            Constants.global_sync_time += dt
            self.t1 = time.time()
            # recompute relay
            relay_time = Constants.global_sync_time + Constants.relay_time
            try:
                self.near_blocks, self.relay_points = self.generate_relay(relay_time)
                self.process_relay(self.near_blocks, self.relay_points, relay_time)
            except ValueError:
                for dr in self.drones:
                    dr.path = [GridBlock(Vector2D(-1, -1), Constants.server_loc, (0, 0, 0)), ]
                    dr.state = DroneState.RTL
                    self.connected_drones = []
                return

            self.connected_drones = []

            # continue
            for drone in self.drones:
                if len(drone.path) > 1:
                    drone.state = DroneState.MOVING

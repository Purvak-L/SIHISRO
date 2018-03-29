# imports 
import time
from Classes import *
from Drone import *

class Simulator:
    def __init__(self):
        Constants.renderer = MapRenderer(Constants.grid_dimension.x, Constants.grid_dimension.y)

        # First get blocks of image points and drones
        self.blocks = Utility.getGridBlocks()
        self.drones = [Drone(i) for i in range(Constants.num_drones)]

        # Equally Distribute blocks in drones
        self.allocations_list = Utility.get_grid_distribution(self.blocks)

        # Reshuffle without considering relay time already alternated
        self.allocations_list = Utility.shuffle_distribution(self.allocations_list, self.drones)

        self.connected_drones = []

    def generate_relay(self, time):
        # Update allocation times
        for i, allocations in enumerate(self.allocations_list):
            Utility.set_estimated_time([block for block in allocations if block.completed == False], self.drones[i])

        # get nearest grid location for next time
        blocks = []
        drange = Constants.drone_range * 0.9
        for allocations in self.allocations_list:
            blocks.append(sorted(allocations, key=lambda x: abs(x.estimated_time - time))[0])

        def _get_intersection(pt1, center, radius):
            # get slope of line
            y1 = math.sqrt(radius ** 2 - (pt1.x - center.x) ** 2) + center.y
            y2 = -math.sqrt(radius ** 2 - (pt1.x - center.x) ** 2) + center.y
            return Vector2D(pt1.x, y1) if y1 > y2 else Vector2D(pt1.x, y2)

        # get relay points
        relay_points = [Constants.server_loc, ]

        for i, block in enumerate(blocks):
            # check if inside range
            vec = block.loc.sub(relay_points[i])
            mag = vec.abs()
            if mag < drange:
                #push it in y TODO change
                relay_points.append(_get_intersection(block.loc, relay_points[i], drange))
            else:
                vec = Vector2D(vec.x / mag, vec.y / mag)
                relay_points.append(Vector2D(relay_points[i].x + vec.x * drange, 
                                             relay_points[i].y + vec.y * drange))

            # scaling of the relay points TODO
            min_x = min([r.x for r in relay_points])
            max_x = max([r.x for r in relay_points])
            min_y = min([r.y for r in relay_points])
            max_y = max([r.y for r in relay_points])

            scale_x = max_x / Constants.grid_dimension.x
            scale_y = max_y / Constants.grid_dimension.y

            # for i, point in enumerate(relay_points):
            #     relay_points[i] = relay_points[i].mul(Vector2D(scale_x, scale_y))
        
        return blocks, relay_points[1:]

    def process_relay(self, blocks, relay_points, relay_time):
        relay_points = [GridBlock((-2, -2), relay_points[i], (0,0,0)) for i in range(len(relay_points))]
        for i, block in enumerate(blocks):
            index = self.allocations_list[i].index(block)
            index_first_not_completed = [ind for ind, grpt in enumerate(self.allocations_list[i]) 
                                         if grpt.completed == False][0]
            if block.estimated_time > relay_time:
                sublist = self.allocations_list[i][index_first_not_completed: index]
            else:
                sublist = self.allocations_list[i][index_first_not_completed: index + 1]
            sublist.append(relay_points[i])
            trace_list = [(self.drones[j], relay_points[j]) for j in range(i)] 
            self.drones[i].path = sublist
            self.drones[i].relay_trace = list(reversed(trace_list))

    def loop(self):
        Constants.global_sync_time = 0
        t1 = time.time()
        time.sleep(0.1)
        f = True
        # generate relay points
        relay_time = 20
        blocks, relay = self.generate_relay(relay_time)
        self.process_relay(blocks, relay, relay_time)

        for drone in self.drones:
            drone.state = DroneState.MOVING

        while True:
            dt = time.time() - t1
            t1 = time.time()
            Constants.global_sync_time += dt
            #print(ttime)
            if f:
                Constants.renderer.render_grid(self.blocks)
                f = False

            # recompute relay
            # if Constants.global_sync_time > relay_time:# test
            #     relay_time += 20
            #     blocks, relay = self.generate_relay(relay_time)
            #     self.process_relay(blocks, relay, relay_time)

            # draw relay and gridpoints
            Constants.renderer.render_points([[r.loc, (0, 0, 0)] for r in blocks])
            Constants.renderer.render_points([[l, (255, 255, 255)] for l in relay])

            # update drones
            for drone in self.drones:
                drone.update(dt)
            Constants.renderer.show()


    # Network functions
    def relay(self, drone_id):
        drone = list(filter(lambda x: x.id == drone_id, self.drones))[0]
        drone.state = DroneState.UNDER_RELAY_CONNECTED
        self.connected_drones.append(drone)

    
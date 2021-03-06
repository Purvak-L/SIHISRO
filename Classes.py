import math
import cv2


class Vector2D:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

    def __str__(self):
        return "[{0:3} {1:3}]".format(self.x, self.y)

    def dist(self, other):
        return math.sqrt((other.x - self.x) ** 2 + (other.y - self.y) ** 2)
    
    def add(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def sub(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def mul(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.x * other.x, self.y * other.y)
        else:
            return Vector2D(self.x * other, self.y * other)
    
    def abs(self):
        return self.dist(Vector2D())
    
    def norm(self):
        mag = self.abs()
        return Vector2D(self.x/mag, self.y/mag)


class GridBlock:
    def __init__(self, index, loc, color):
        self.index = index
        self.loc = loc
        self.color = color
        self.allocated = False
        self.distance_from_server = self.loc.dist(Constants.server_loc)
        self.completed = False
        self.estimated_time = None
        self.drone_id = None

    def __str__(self):
        return "{0},{1},{2},{3},{4}".format(self.index, self.loc, self.color, self.completed, self.allocated)


class Constants:
    # Drone Constants
    time_of_flight = 200
    time_to_click = 2
    relay_wait_duration = 2
    velocity = 30
    drone_range = 20
    original_num_drones = 5
    num_drones = original_num_drones
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (102, 0, 102), (255, 0, 255),
        (215, 220, 55), (205, 100, 155), (155, 200, 255), (233, 12, 33), (123, 45, 111)]
    height = 17.5 #17.5 # get as param
    fov = 60

    # Map Constants
    coordinates = [Vector2D(0, 0), Vector2D(0, 100), Vector2D(100, 100), Vector2D(100, 0)]
    overlap = .25
    block_width = height * math.tan(fov / 2 * math.pi / 180)
    block_height = height * math.tan(fov / 2 * math.pi / 180)
    grid_dimension = Vector2D(100, 100)

    # Locations
    server_loc = Vector2D(50, 0)


    # Simulator
    simulator = None
    relay_time = 10
    next_relay_time = relay_time
    chat_client = None

    # Renderer
    renderer = None

    # enviroment
    network = None

    # Time
    global_sync_time = 0

    # flag for sending data to website
    web_server_clients = []


class MapRenderer:
    def __init__(self, width, height, path = './img.png'):
        self.map_width = width
        self.map_height = height
        self.path = path
        self.width_ratio = 0.0
        self.height_ratio = 0.0
        self.map_image = None
        self.output = None
        self.grid_layer = None #TODO Optimize drawing
        self.grid_list = []
        cv2.namedWindow("map", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("map", 700, 700)
        #cv2.resizeWindow("map", 1280, 720)

    def _set_map_image(self):
        self.map_image = cv2.imread(self.path)
        # self.map_image = cv2.resize(self.map_image, ())

    def show(self):
        if isinstance(self.output, type(None)):
            return
        cv2.waitKey(1)
        cv2.imshow("map", self.output)
        self.output = None

    def close(self):
        cv2.destroyAllWindows()

    def _draw_rect_on_map(self, x, y, w, h, color, alpha):
        x = x - w/2
        y = y - h/2
        if isinstance(self.output, type(None)):
            self.output = self.map_image.copy()
        overlay = self.output.copy()
        cv2.rectangle(overlay, (int(x * self.width_ratio), int(y * self.height_ratio)),
                      (int((x + w) * self.width_ratio), int((y + h) * self.height_ratio)), color, -1)
        cv2.addWeighted(overlay, alpha, self.output, 1 - alpha, 0, self.output)

    def _draw_rect_on_map_optimized(self, block_list, w, h, alpha):
        if isinstance(self.output, type(None)):
            self.output = self.map_image.copy()
        overlay = self.output.copy()
        for block in block_list:
            x = block.loc.x - w / 2
            y = block.loc.y - h / 2
            cv2.rectangle(overlay, (int(x * self.width_ratio), int(y * self.height_ratio)),
                      (int((x + w) * self.width_ratio), int((y + h) * self.height_ratio)), block.color, -1)
        cv2.addWeighted(overlay, alpha, self.output, 1 - alpha, 0, self.output)
        self.grid_layer = self.output.copy()

    def render_grid(self, grid_blocks, w = Constants.block_width, h = Constants.block_height):

        if isinstance(self.map_image, type(None)):
            self._set_map_image()

        completed_blocks = []
        incomplete_blocks = []

        (height, width, channel) = self.map_image.shape
        self.width_ratio = width / float(self.map_width)
        self.height_ratio = height / float(self.map_height)
        for block in grid_blocks:
            if block.completed:
                completed_blocks.append(block)
            else:
                incomplete_blocks.append(block)

        self._draw_rect_on_map_optimized(completed_blocks, w, h, 0.6)
        self._draw_rect_on_map_optimized(incomplete_blocks, w, h, 0.2)

    def render_points(self, point_list, range_=None, connected=False):
        """
        point_list is List of list, containing x,y, color where color is in the form (0,0,0)
        """
        (height, width, channel) = self.map_image.shape
        self.width_ratio = width / float(self.map_width)
        self.height_ratio = height / float(self.map_height)
        if isinstance(self.output, type(None)):
            self.output = self.map_image.copy()
        for point, color in point_list:
            if connected:
                cv2.circle(self.output, (int(point.x * self.width_ratio), int(point.y * self.height_ratio)), 8,
                           (0, 255, 0), 8)
            cv2.circle(self.output, (int(point.x * self.width_ratio), int(point.y * self.height_ratio)), 5, (0, 0, 0), 4)
            cv2.circle(self.output, (int(point.x * self.width_ratio), int(point.y * self.height_ratio)), 3, color, 3)
            if not isinstance(range_, type(None)):
                cv2.circle(self.output, (int(point.x * self.width_ratio), int(point.y * self.height_ratio)), int(range_
                           * self.width_ratio), (0, 255, 0), 1)

    def draw_connection_line(self, pt1, pt2):
        (height, width, channel) = self.map_image.shape
        self.width_ratio = width / float(self.map_width)
        self.height_ratio = height / float(self.map_height)
        if isinstance(self.output, type(None)):
            self.output = self.map_image.copy()
        cv2.line(self.output, (int(pt1.x * self.width_ratio), int(pt1.y * self.height_ratio)),
                 (int(pt2.x * self.width_ratio), int(pt2.y * self.height_ratio)), (0, 255, 0), 3)

class Utility:
    @staticmethod
    def get_json_string(packet_type, list_ = None, img = None, next_est_relay = 0, drones = None):
        json_ = "{"
        if packet_type is "drones":
            json_ += "\"type\": \"drones\","
            json_ += "\"drones\": ["
            for i, loc in enumerate(list_):
                x = loc.x
                y = loc.y
                z = 200
                if i < len(list_) - 1:
                    str_ = "{{\"id\":\"{0}\",\"est_loc\":[{1},{2},{3}],\"conn_status\":\"connected\"," \
                           "\"timestamp\":1}},".format(i+1, x, y, z)
                else:
                    str_ = "{{\"id\":\"{0}\",\"est_loc\":[{1},{2},{3}],\"conn_status\":\"connected\",\"timestamp\": " \
                           "1234}}".format(i + 1, x, y, z)
                json_ += str_
            json_ += "]}"

        elif packet_type is "grid_data":
            json_ += "\"type\": \"grid_data\","
            json_ += "\"dim\": [{0},{1}],".format(Constants.block_height, Constants.block_width)
            json_ += "\"top_lat_long\": [{0},{1},{2},{3}],".format(0, 0, Constants.grid_dimension.x, Constants.grid_dimension.y)
            json_ += "\"blocks\": ["
            for i, grid_pt in enumerate(list_):
                id = "{0}{1}".format(grid_pt.index[0], grid_pt.index[1])
                status = "not_explored"
                if grid_pt.completed:
                    status = "explored"
                if i < len(list_) - 1:
                    str_ = "{{\"id\":\"{0}\",\"centre\":[{1},{2},{3}],\"status\":\"{4}\",\"drone_id\":{5}}},".format(id, grid_pt.loc.x, grid_pt.loc.y, 0, status, grid_pt.drone_id)
                else:
                    str_ = "{{\"id\":\"{0}\",\"centre\":[{1},{2},{3}],\"status\":\"{4}\",\"drone_id\":{5}}}".format(id, grid_pt.loc.x, grid_pt.loc.y, 0, status, grid_pt.drone_id)
                json_ += str_
            json_ += "]}"
            print(json_)

        elif packet_type is "relay":
            json_ += "\"type\": \"relay\","
            json_ += "\"n_relay\": {0},".format(len(list_))
            json_ += "\"next_relay_est_time\":{0},".format(next_est_relay)
            json_ += "\"relay_status\": \"complete\","
            json_ += "\"relay_points\": ["
            for i, relay_pt in enumerate(list_):
                # id = "{0}{1}".format(relay_pt.index.x,relay_pt.index.y)
                # next_id = "{0}{1}".format(list_[i+1].index.x, list_[i+1].index.y)
                id = 10
                connected = "[{0},{1}]".format(0, 0)
                # if 0 < i < (len(list_) - 1):
                #     last_id = "{0}{1}".format(list_[i-1].index.x, list_[i-1].index.y)
                #     connected = "[{0},{1}]".format(last_id, id)
                # else:
                #     connected = "[{0}]".format(id)

                if i < len(list_) - 1:
                    str_ = "{{\"id\":{0},\"loc\":[{0},{1},{2}],\"affiliated_drone\":{3},\"connected\":{4}," \
                           "\"connected_status\":\"up\"}},".format(id, relay_pt.x, relay_pt.y, 0,
                                                                   drones[i].id, connected)
                else:
                    str_ = "{{\"id\":{0},\"loc\":[{0},{1},{2}],\"affiliated_drone\":{3},\"connected\":{4}," \
                           "\"connected_status\":\"up\"}}".format(id, relay_pt.x, relay_pt.y, 0,
                                                                  drones[i].id, connected)
                json_ += str_
            json_ += "]}"

        elif packet_type == "bg-img":
            json_ += "\"type\": \"bg-img\", \"data\":\""
            # pil_img = Image.fromarray(img)
            # buff = BytesIO()
            # pil_img.save(buff, format="JPEG")
            # new_image_string = base64.b64encode(buff.getvalue()).decode("utf-8")
            new_image_string = img.tostring()
            new_image_string.decode("utf-8")
            json_ += new_image_string
            json_ += "\"}"

        return json_

    @staticmethod
    def getGridBlocks(coordinates = Constants.coordinates, overlap = Constants.overlap, 
            block_width = Constants.block_width, block_height = Constants.block_height):
        # get length and breadth of the rectangle
        l = coordinates[2].x - coordinates[1].x
        b = coordinates[1].y - coordinates[0].y

        # get number of rectangles in l and b as n1 and n2 resp
        n1 = math.ceil((l - block_width * overlap) / (block_width - block_width * overlap))
        n2 = math.ceil((b - block_height * overlap) / (block_height - block_height * overlap))

        # get new overlaps
        d1 = (n1*block_width - l) / (n1 - 1)
        d2 = (n2*block_height - b) / (n2 - 1)

        # print new overlaps
        #print("New overlap in x and y resp is: {}{:5}".format(d1,d2))

        # calculate grids
        grid_points = []

        # rows
        for i in range(0, n2):
            for j in range(0, n1):
                x = coordinates[0].x + block_width / 2 + j * (block_width - d1)
                y = coordinates[0].y + block_height / 2 + i * (block_height - d2)
                # grid_points.append([(j, i), (x, y), 0.5, None])
                block = GridBlock((j, i), Vector2D(x, y), None)
                grid_points.append(block)
        return grid_points

    @staticmethod
    def get_grid_distribution(grid_blocks):
        # find max distance to get new range
        num_drones = Constants.num_drones
        distances = [x.distance_from_server for x in grid_blocks]
        min_dist = min(distances)
        mapping_range = (max(distances) - min_dist) / num_drones

        allocations = [[] for i in range(num_drones)]

        for block in grid_blocks:
            for i in range(num_drones):
                if (block.distance_from_server - min_dist) <= (i + 1) * mapping_range:
                    allocations[i].append(block)
                    block.allocated = True
                    block.color = Constants.colors[i]
                    break
        not_allocated = [block for block in grid_blocks if not block.allocated]

        for block in not_allocated:
            block.allocated = True
            allocations[Constants.num_drones - 1].append(block)

        return allocations

    @staticmethod
    def arrange_grid(allocations):
        transposeIndividual = {}

        for block in allocations:
            # get columns
            indices = block.index
            transposeIndividual.setdefault(indices[0], []).append(block)

        def alternate(transpose):
            alternateGrid = []
            i = 0
            for k, v in transpose.items():
                if (i % 2 == 0):
                    for block in v:
                        alternateGrid.append(block)
                    i += 1
                else:
                    for block in v[::-1]:
                        alternateGrid.append(block)
                    i += 1
            return alternateGrid

        # get alternate
        alternateGrid = alternate(transposeIndividual)
        return alternateGrid

    # considers time from drone current location and finally towards the last point
    @staticmethod
    def set_estimated_time(allocations, drone, last_point = Constants.server_loc):
        time_ = Constants.global_sync_time
        curr_loc = drone.loc
        travel_towards = curr_loc.dist(allocations[0].loc) / Constants.velocity
        travel_back = allocations[-1].loc.dist(last_point) / Constants.velocity

        for block in allocations:
            dist = block.loc.dist(curr_loc)
            time_ += dist / Constants.velocity
            block.estimated_time = time_
            time_ += Constants.time_to_click
            curr_loc = block.loc

        avg_time = time_ / len(allocations)

        drone.estimated_time = time_ + travel_towards + travel_back
        drone.travel_towards = travel_towards
        drone.travel_back = travel_back
        drone.avg_time = avg_time

    @staticmethod
    def shuffle_distribution(allocations_list, drones):

        n_original_allocs = sum([len(s) for s in allocations_list])

        for i, allocations in enumerate(allocations_list):
            Utility.set_estimated_time(allocations, drones[i])
        
        sorted_allocations_list = [sorted(allocations, key=lambda x: x.distance_from_server) 
                                          for allocations in allocations_list]

        for j in range(0, 5): # TODO update to auto
            for i, drone in enumerate(drones):
                #print("Iter {0} Drone {1}".format(j, i))
                #print("Allocated {0}".format([len(al) for al in allocations_list]))
                #print("Sorted {0}".format([len(al) for al in sorted_allocations_list]))
                time_prev = drones[i-1].estimated_time if i-1 >= 0 else float('-inf')
                time_next = drones[i+1].estimated_time if i+1 < Constants.num_drones else float('-inf')
                #print("Time prev {0} Time next {1} current {2}".format(time_prev, time_next, drone.estimated_time))
                if drone.estimated_time < time_prev or drone.estimated_time < time_next:
                    if time_prev > time_next:
                        prev_locs = sorted_allocations_list[i - 1]
                        num_extra_blocks = int((time_prev - drone.estimated_time) / drones[i-1].avg_time)
                        if int(num_extra_blocks / 2) == 0:
                            continue
                        # take 
                        take = prev_locs[-int(num_extra_blocks / 2): ]
                        #print("Drone{0}/{5} taking {1}/{4} from Drone{2} at iter {3}".format(i, len(take), i-1, j,
                        #       len(sorted_allocations_list[i - 1]), len(sorted_allocations_list[i])))
                        sorted_allocations_list[i - 1] = prev_locs[:-int(num_extra_blocks / 2)]
                        curr_drone_locations = sorted_allocations_list[i]
                        curr_drone_locations += take # keep it sorted
                        sorted_allocations_list[i] = curr_drone_locations
                        # update allocations
                        allocations_list[i - 1] = Utility.arrange_grid(sorted_allocations_list[i - 1])
                        Utility.set_estimated_time(allocations_list[i - 1], drones[i - 1])
                        # # update colors
                        # for block in allocations_list[i - 1]:
                        #     block.color = Constants.colors[i - 1]
                    else:
                        next_locs = sorted_allocations_list[i + 1]
                        num_extra_blocks = int((time_next - drone.estimated_time) // drones[i + 1].avg_time)
                        if int(num_extra_blocks / 2) == 0:
                            continue
                        take = next_locs[:int(num_extra_blocks / 2)]
                        #print("Drone{0}/{5} taking {1}/{4} from Drone{2} at iter {3}".format(i, len(take), i + 1, j,
                        #       len(sorted_allocations_list[i + 1]), len(sorted_allocations_list[i])))
                        sorted_allocations_list[i + 1] = next_locs[int(num_extra_blocks / 2):]
                        curr_drone_locations = sorted_allocations_list[i]
                        sorted_allocations_list[i] = curr_drone_locations + take # keep it sorted
                        # update allocations
                        allocations_list[i + 1] = Utility.arrange_grid(sorted_allocations_list[i + 1])
                        Utility.set_estimated_time(allocations_list[i + 1], drones[i + 1])
                        # # update colors
                        # for block in allocations_list[i + 1]:
                        #     block.color = Constants.colors[i + 1]
                    # update current drone allocations
                    allocations_list[i] = Utility.arrange_grid(sorted_allocations_list[i])
                    Utility.set_estimated_time(allocations_list[i], drones[i])
                    n_later = sum([len(s) for s in allocations_list])
                    z = 1
                    # update colors 
                    for block in allocations_list[i]:
                        block.color = Constants.colors[i]

            # Draw
            pt_list_flattened = []
            for allocations in allocations_list:
                pt_list_flattened += allocations

            # render grid
            Constants.renderer.render_grid(pt_list_flattened)
            Constants.renderer.show()

        # set drone id to grid blocks
        # for i, allocations in enumerate(allocations_list):
        #     for block in allocations:
        #         block.drone_id = drones[i].id

        print("Final")
        print("{0}".format([d.estimated_time for d in drones]))
        return allocations_list

    @staticmethod
    def processPoints(blocks, drones = []):
            
        # set locations for each drone
        for i, drone in enumerate(drone_list):
            drone.set_locations(distribution[i+1])

        # shuffle distribution for same time
        shuffle_distribution(drone_list, gridDimension, imgWidth, imgHeight)

        # Print est time for drones
        for drone in drone_list:
            print("Drone{0}, ETA,to,back: {1}".format(drone.id, drone.get_estimated_time()))
        return drone_list


class Commands:

    @staticmethod
    def set_params(coordinates, level, max_height, overlap, num_of_drones, server_range, time_of_flight):
        Constants.num_drones = int(num_of_drones)
        Constants.drone_range = int(server_range)
        Constants.overlap = float(overlap)
        Constants.time_of_flight = int(time_of_flight)

        # calculate coordinates
        for i, coord in enumerate(coordinates):
            coordinates[i] = float(coord)

        Constants.grid_dimension = Vector2D(coordinates[2] - coordinates[0], coordinates[3] - coordinates[1])
        print("{0} {1}".format(Constants.grid_dimension.x, Constants.grid_dimension.y))

        # standardize
        top_left = Vector2D(coordinates[0], coordinates[3])
        top_right = Vector2D(top_left.x + Constants.grid_dimension.x, top_left.y)
        bottom_right = Vector2D(coordinates[2], coordinates[1])
        bottom_left = Vector2D(bottom_right.x - Constants.grid_dimension.x, bottom_right.y)
        Constants.coordinates = [bottom_left, top_left, top_right, bottom_right]
        print(Constants.coordinates)

        # set sim params
        Constants.simulator.start()
        Constants.web_server_clients[-1].sendMessage('{"type":"ready"}'.encode('utf-8'))

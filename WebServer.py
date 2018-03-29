from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor
from random import randint
import base64
import sys
import json
from Classes import *
import _thread

# data structures for main page image data
img_dic = {}
img_dic['type'] = "bg-img"

# data structures for drone stats
drone_loc_json_string = '{ "type": "drones", "drones": [{"id": "1101", "est_loc": [12.1234,11.5678,1], "conn_status": "connected", "timestamp": "020439" }, {"id": "1102", "est_loc": [8.1234,14.5678,1], "conn_status": "connected", "timestamp": "074518" }, {"id": "1103", "est_loc": [9.1234,10.5678,1], "conn_status": "not_connected", "timestamp": "032338" }, {"id": "1104", "est_loc": [4.1234,11.5678,1], "conn_status": "connected", "timestamp": "032338" } ]}'

# data structures for relay points data
relay_json_string = """{"type": "relay", "n_relay": "7", "next_relay_est_time": "23", "relay_status": "complete", 
        "relay_points" : [ 
        {   "id": 1201, 
            "loc": [12.1233,12.4567,-1], 
            "affiliated_drone": 1101, 
            "connected": [-1, 1202], 
            "connected_status": ["up", "down"]  
        }, 
        {   "id": 1202, 
            "loc": [9.1233,8.4567,-1], 
            "affiliated_drone": 1102, 
            "connected": [1201, 1203], 
            "connected_status" : ["up", "down"] 
        }, 
        {   "id": 1203, 
            "loc": [12.1233,12.4567,-1],
            "affiliated_drone": 1103,
            "connected": [1204],
            "connected_status": ["up", "down"]    
        }
    ]
}"""

grid_json_string = """
{
    "type": "grid_data",

    "dim": [40, 50],
    "top_lat_long": [12.1234, 14.5678, 40, 50],
    "blocks": [{
        "id": 152,
        "label": "B1",
        "centre": [30, 40, 200],
        "status": "explored",
        "drone_id": 1101,
        "est_time": 2345
    },
    {
        "id": 152,
        "label": "B1",
        "centre": [30, 40, 200],
        "status": "unexplored",
        "drone_id": 1101,
        "est_time": 2345
    },
    {
        "id": 152,
        "label": "B1",
        "centre": [30, 40, 200],
        "status": "unexplored",
        "drone_id": 1101,
        "est_time": 2345
    },
    {
        "id": 152,
        "label": "A1",
        "centre": [30, 40, 200],
        "status": "explored",
        "drone_id": 1101,
        "est_time": 2345
    },
    {
        "id": 152,
        "label": "A1",
        "centre": [30, 40, 200],
        "status": "explored",
        "drone_id": 1101,
        "est_time": 2345
    },
    {
        "id": 152,
        "label": "A1",
        "centre": [30, 40, 200],
        "status": "unexplored",
        "drone_id": 1101,
        "est_time": 2345
    },
    {
        "id": 152,
        "label": "A1",
        "centre": [30, 40, 200],
        "status": "explored",
        "drone_id": 1101,
        "est_time": 2345
    }]
}
"""


class MyServerProtocol(WebSocketServerProtocol):
    def onConnect(self, request):
        print("Client connecting: {}".format(request.peer))

    # handle messages from dashboard
    def onOpen(self):
        print("WebSocket connection open.")
        Constants.web_server_clients.append(self)

        def sendMapData():
            ENCODING = 'utf-8'
            IMAGE_NAME = 'img.png'

            with open(IMAGE_NAME, 'rb') as open_file:
                byte_content = open_file.read()

            base64_bytes = base64.b64encode(byte_content)

            base64_string = base64_bytes.decode(ENCODING)

            # modify this for changing image data

            img_dic["img-data"] = base64_string
            img_json_string = json.dumps(img_dic)
            # print img_dic
            # self.sendMessage(img_json_string.encode('utf8'))
            # self.sendMessage(drone_loc_json_string.encode('utf8'))
            # self.sendMessage(relay_json_string.encode('utf8'))
            # self.sendMessage(grid_json_string.encode('utf8'))
            # self.sendMessage(test_coord.encode('utf8'))
            self.factory.reactor.callLater(1.0, sendMapData)

        # send image data to dashboard every 5ms
        sendMapData()

    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {} bytes".format(len(payload)))
        else:
            def start_sim():
                Constants.simulator.start()
                Constants.simulator.loop()
            _thread.start_new_thread(start_sim, ())
            data_from_dash = json.loads(payload.decode('utf8'))
            if (data_from_dash['type'] == 'sim_params'):
                print("level: ", data_from_dash['level'])
                print("height: ", data_from_dash['height'])
                print("Overlap(%): ", data_from_dash['overlap'])
                print("no_drones: ", data_from_dash['swarm'][0])
                print("comm_range: ", data_from_dash['swarm'][1])
                print("server_range: ", data_from_dash['swarm'][2])


                # print("Text message received: {}".format(payload.decode('utf8')))
        # echo back message verbatim
        self.sendMessage(payload, isBinary)

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {}".format(reason))
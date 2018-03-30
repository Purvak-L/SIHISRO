# imports
from Classes import *
from Simulator import *
from WebServer import *
from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from twisted.python import log
from twisted.internet import reactor
import sys
import socket

## Main code

# TODO Host Webserver
# start server
def bore():
    print("creating..")
    Constants.simulator = Simulator()
    startClient()
    log.startLogging(sys.stdout)
    factory = WebSocketServerFactory(u"ws://127.0.0.1:50050")
    factory.protocol = MyServerProtocol
    reactor.listenTCP(50050, factory)
    reactor.run()
    # p.join()


def startClient():
    server_address = 'localhost'
    client_socket = socket.socket()
    port = 7555
    client_socket.connect(('localhost', port))
    print('Connected to %s on port %s' % (server_address, port))
    Constants.chat_client = client_socket


if __name__ == '__main__':
    # bore()
    Constants.simulator = Simulator()
    Constants.simulator.start()
    startClient()
    Constants.simulator.loop()
# sim = Simulator()
# sim.loop()

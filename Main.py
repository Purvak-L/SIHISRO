# imports
from Classes import *
from Simulator import *
from WebServer import *


## Main code

# TODO Host Webserver
# start server
def bore():
    print("creating..")
    Constants.simulator = Simulator()
    log.startLogging(sys.stdout)
    factory = WebSocketServerFactory(u"ws://127.0.0.1:8080")
    factory.protocol = MyServerProtocol
    reactor.listenTCP(8080, factory)
    reactor.run()

# bore()
Constants.simulator = Simulator()
Constants.simulator.start()
Constants.simulator.loop()
# Test Create a Simulater Object (For the time Webserver is absent)
#sim = Simulator()
#sim.loop()

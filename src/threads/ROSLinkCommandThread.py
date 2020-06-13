__package__ = 'threads'

import threading

class ROSLinkCommandThread ( threading.Thread ):
    def __init__(self):
        t = threading.Thread(target=self.run)
        t.start()
    def run ( self):
        print "start command thread"
        #pass
        #s.connect(server_address)
        #self.server_add = server_address
        #print 'This thread name is', self.getName()
    def wait_for_command(self):
        #while True:
            msg, address = s.recvfrom(1024)
            r, _, _ = select.select([self.clientSocket], [], [])
            if r:
                data = s.recv(1024)
                print "Got data: ", data
            #print '\nRobot at', self.client, 'is connected\n'
            #t = threading.Thread(target=self.handleMessage, args=(client,))
            print "I received new command.."
            #print "clients_list ", self.clients_list
            #t = threading.Thread(target=self.handleMessage, args=(msg,))
            #t.start()
            self.handleMessage(msg)
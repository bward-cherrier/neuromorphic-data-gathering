# -*- coding: utf-8 -*-
"""
This module can be used to create simple RPC clients and servers.

Some possible usage patterns are as follows:
    
1. Install functions and class instance on a server:
  
def add(a, b):
    return a + b
    
def sub(a, b):
    return a - b
    
class Calc(object):
    def __init__(self, mem):
        self._mem = mem

    def mult(self, a, b):
        return a * b

server = RPCServer('localhost', 17000)
server.register_function(add)
server.register_function(sub)
server.register_instance(Calc(3.141))
server.serve_forever()

2. Create a client and call functions on a server:
    
client = RPCProxy('localhost', 17000)
sum = client.add(2, 3)
diff = client.sub(7, 4)
prod = client.mult(5, 6)

3. Install class on a server (class instance is created after the server is
running, by calling a register_instance(constructor_args) method):

class Calc(object):
    def __init__(self, mem):
        self._mem = mem

    def mult(self, a, b):
        return a * b
        
server = RPCServer('localhost', 17000)
server.register_class(Calc)
server.serve_forever()

...

client = RPCProxy('localhost', 17000)
client.register_instance(3.141)
prod = client.mult(5, 6)

4. Create a client and automatically start a server by running a pre-defined
server script:
    
Assuming that 'server.py' contains the code in the first example above:
    
client = RPCAutoProxy('server.py', 'localhost', 17000)
sum = client.add(2, 3)
diff = client.sub(7, 4)
prod = client.mult(5, 6)

"""

import time
import subprocess
from multiprocessing.connection import Listener, Client
#from threading import Thread

def resolve_dotted_attribute(obj, attr):
    """resolve_dotted_attribute(a, 'b.c.d') => a.b.c.d
    
    Resolves a dotted attribute name to an object.  Raises an AttributeError
    if any attributes in the chain start with a '_'.
    """
    attrs = attr.split('.')
    for i in attrs:
        if i.startswith('_'):
            raise AttributeError('attempt to access private attribute "%s"' % i)
        else:
            obj = getattr(obj, i)
    return obj

class RPCProxy(object):
    """Simple RPC client class.
    """
    def __init__(self, address, port):
        self._connection = Client((address, port), authkey=b'tactip')

    def __del__(self):
        self._connection.close()
  
    def __getattr__(self, name):
        """
        Returns a function that forwards the request to the server.
        """
        def do_rpc(*args, **kwargs):
            self._connection.send((name, args, kwargs))
            result = self._connection.recv()
            if isinstance(result, Exception):
                raise result
            return result
        return do_rpc

class RPCAutoProxy(RPCProxy):
    """RPC auto proxy class automatically starts server on initialisation
    """
    def __init__(self, server_script, address, port):
        command = 'python ' + server_script + ' -a ' + address + ' -p ' + str(port)        
        subprocess.Popen(command, creationflags=subprocess.SW_HIDE, shell=True)
        time.sleep(1)
        RPCProxy.__init__(self, address, port)
        
class RPCServer(object):
    """Simple RPC server class.
    """
    def __init__(self, address, port):
        self._functions = {}
        self._instance = None
        self._listener = Listener((address, port), authkey=b'tactip')
    
    def register_function(self, function, name=None):
        """Registers a function to respond to RPC requests.
        """
        if name is None:
            name = function.__name__
        self._functions[name] = function

    def register_instance(self, instance):
        """Registers an instance to respond to RPC requests.
        """
        self._instance = instance
        
    def register_class(self, class_name):
        """Registers a class to respond to RPC requests (class must be
        instantiated after the server is started, using the register_instance
        function).
        """        
        def register_instance(*args, **kwargs):
            instance = class_name(*args, **kwargs)
            self.register_instance(instance)
        self.register_function(register_instance)  
        
    def _handle_connection(self, connection):
        """Main server loop handles a client connection and dispatches
        requests to registered functions or class instances.
        """
        try:
            while True:
                name, args, kwargs = connection.recv()
                try:
                    func = None
                    try:
                        func = self._functions[name]
                    except KeyError:
                        if self._instance is not None:
                            try:
                                func = resolve_dotted_attribute(self._instance, name)
                            except AttributeError:
                                pass
                    if func is not None:
                        r = func(*args, **kwargs)
                        connection.send(r) 
                    else:
                        raise Exception('function "%s" is not supported' % name)
                except Exception as e:
                    connection.send(e)
        except EOFError:
            pass
        
    def serve_forever(self):
        """Accepts client connection and passes it to main server loop for
        request handling."""
        connection = self._listener.accept()
        self._handle_connection(connection)
        connection.close()

# Single-threaded/multiple-connection version
#        while True:
#            connection = _listener.accept()
#            self._handle_connection(connection)
#            connection.close()
#            
# Multiple-threaded/multiple-connection version
#        while True:
#            connection = _listener.accept()
#            t = Thread(target=self._handle_connection, args=(connection,))
#            t.daemon = True
#            t.start()

def main():
    pass

if __name__ == '__main__':
    main()



# -*- coding: utf-8 -*-
"""
This module is used to create simple subprocess procedure call (SPC) clients
and servers that communicate via message queues.

Some possible usage patterns are as follows:
    
1. Install functions and class instance on a subprocess server, and call via a
proxy:

import multiprocessing as mp

def add(a, b):
    return a + b
    
def sub(a, b):
    return a - b
    
class Calc(object):
    def __init__(self, mem):
        self._mem = mem

    def mult(self, a, b):
        return a * b

requests = mp.Queue()
responses = mp.Queue()
server = SPCServer(requests, responses)
server.register_function(add)
server.register_function(sub)
server.register_instance(Calc(3.141))
server.daemon = True
server.start()
resp = responses.get()
if isinstance(resp, Exception):
    raise resp
if resp != 'ready':
    raise RuntimeError('failed to start SPC server')

def add_proxy(a, b):
    requests.put(('add', (a, b), {}))
    resp = responses.get()
    if isinstance(resp, Exception):
        raise resp
    return resp
    
sum = add_proxy(2, 3)

requests.put(('exit', (), {}))

2. Create a client and call functions on a subprocess server:

client = SPCProxy(Calc, 3.141)
sum = client.add(2, 3)
diff = client.sub(7, 4)
prod = client.mult(5, 6)

3. Install class instance on a subprocess server (instantiation is deferred
until after the subprocess has started):
    
class Calc(object):
    def __init__(self, mem):
        self._mem = mem

    def mult(self, a, b):
        return a * b
        
requests = mp.Queue()
responses = mp.Queue()
server = SPCServer(requests, responses)
server.register_instance_deferred(Calc, 3.141)
server.daemon = True
server.start()
resp = responses.get()
if isinstance(resp, Exception):
    raise resp
if resp != 'ready':
    raise RuntimeError('failed to start SPC server')

def mult_proxy(a, b):
    requests.put(('mult', (a, b), {}))
    resp = responses.get()
    if isinstance(resp, Exception):
        raise resp
    return resp
    
prod = mult_proxy(5, 6)

requests.put(('exit', (), {}))

"""

import multiprocessing as mp 

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

class SPCProxy(object):
    """Simple SPC client class.
    """
    def __init__(self, class_name, *args, **kwargs):
        self._requests = mp.Queue()
        self._responses = mp.Queue()
        self._busy = False
        self._cancelled = False        
        proc = SPCServer(self._requests, self._responses)
#        p.register_instance(class_name(*args, **kwargs))
        proc.register_instance_deferred(class_name, *args, **kwargs)
        proc.daemon = True
        proc.start()
        resp = self._responses.get()
        if isinstance(resp, Exception):
            raise resp
        if resp != 'ready':
            raise RuntimeError('failed to start SPC server')

    def __del__(self):
        # Send poisoned pill to shut down SPC server
        self._requests.put(('exit', (), {}))
        
    def __getattr__(self, name):
        """
        Returns a function that forwards the request to the server.
        """ 
        if name.startswith('async_'):
            def do_spc(*args, **kwargs):
                if self._busy: raise RuntimeError('async function call error')
                self._requests.put((name[6:], args, kwargs))
                self._busy = True           
        else:
            def do_spc(*args, **kwargs):
                if self._busy: raise RuntimeError('sync function call error')
                self._requests.put((name, args, kwargs))
                resp = self._responses.get()
                if isinstance(resp, Exception):
                    raise resp
                return resp
        return do_spc
        
    def async_result(self):
        """Blocks on the result of an asynchronous function call. Any
        exceptions are propagated to the calling code.
        """
        if not self._busy:
            raise RuntimeError('async result error') 
        resp = self._responses.get()
        self._busy = False
        self._cancelled = False
        if isinstance(resp, Exception):
            raise resp
        return resp 
        
    def async_cancel(self):
        """Sends a cancel signal to the remote function.
        """
        if not self._busy or self._cancelled:
            raise RuntimeError('async cancel error') 
        self._requests.put(('cancel', (), {}))
        self._cancelled = True 

    
class SPCServer(mp.Process):
    """Simple SPC server class.
    """
    def __init__(self, requests, responses):
        mp.Process.__init__(self)
        self._requests = requests
        self._responses = responses
        self._functions = {}
        self._instance = None
        self._class_name = None
        self._args = []
        self._kwargs = {}
        
    def register_function(self, function, name=None):
        """Registers a function to respond to requests.
        """
        if name is None:
            name = function.__name__
        self._functions[name] = function

    def register_instance(self, instance):
        """Registers an instance to respond to RPC requests.
        """
        self._instance = instance
        
    def register_instance_deferred(self, class_name, *args, **kwargs):
        """Registers a class instance to respond to RPC requests (class is
        instantiated after the server is started).
        """    
        self._class_name = class_name
        self._args = args
        self._kwargs = kwargs

    def run(self):
        """Request handler dispatches requests to registered functions or
        class instances.
        """
        
        # Helper function for cancelling asynchronous functions
        def cancel():
            if self._requests.empty():
                return False
            else:
                name, args, kwargs = self._requests.get()
                if name == 'cancel':
                    return True
                else:
                    raise ValueError('Incorrect cancel command')
        
        # Instantiate class if required
        if self._instance is None and self._class_name is not None:
            try:
                self._instance = self._class_name(*self._args, **self._kwargs)
            except Exception as e:
                self._responses.put(e)
                
        # Indicate that server is ready to handle requests 
        self._responses.put('ready')

        # Handle requests
        while True:
            name, args, kwargs = self._requests.get()
            try:
                if name == 'exit':
                    break 
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
                    try:
                        r = func(cancel=cancel, *args, **kwargs)
                    except TypeError:
                        r = func(*args, **kwargs)
                    r = True if r is None else r
                    self._responses.put(r)
                else:
                    raise Exception('function "%s" is not supported' % name)
            except Exception as e:
                self._responses.put(e)


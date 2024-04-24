# -*- coding: utf-8 -*-
"""
Created on Thu Jun 21 10:44:47 2018

@author: jl15313
"""

import Pyro4


def start_pyro_server(object_class, object_id, expose_class=False):
    """
    Instantiates object of specified class on daemon and registers it
    with name server.
    """
    Pyro4.config.SERIALIZERS_ACCEPTED = set(['pickle'])
    with Pyro4.Daemon() as daemon:
        ns = Pyro4.locateNS()
        if expose_class:
            object_class = Pyro4.expose(object_class)
        uri = daemon.register(object_class)
        ns.register(object_id, uri)
        print("Object '{}' instantiated on daemon and registered with name server".format(object_id))
        print("Use CTRL+C or close window to terminate daemon")
        daemon.requestLoop()


def main():
    pass


if __name__ == '__main__':
    main()
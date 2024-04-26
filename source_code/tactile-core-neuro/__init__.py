import logging

# Set default logging handler to avoid "No handler found" warnings.

try: # Python 2.7+
    from logging import NullHandler
except ImportError:
    class NullHandler(logging.Handler):
        def emit(self, record):
            pass

logging.getLogger(__name__).addHandler(NullHandler())

#logging.basicConfig(filename="tactile.log",
#                    level=logging.DEBUG,
#                    format='%(levelname)s: %(asctime)s %(message)s',
#                    datefmt='%m/%d/%Y %I:%M:%S')
from argparse import ArgumentParser
from Pyro5.api import expose, Daemon, locate_ns, oneway
from core.sensor.tactile_sensor_neuro import NeuroTac
import subprocess
import time

@expose
class NeuroTacService(object):
    # Initialize sensor variables
    def __init__(self, save_events_video:bool=False, save_acc_video:bool=False, display:bool=False)-> None:
        self.sensor = NeuroTac(save_events_video=save_events_video, save_acc_video=save_acc_video, display=display)
        print(f"Connected to {self.sensor.camera_type}")

    def __enter__(self):
        return self

    def __exit__(self, type, value, tb)-> None:
        self.close()

    @property
    def camera_type(self):
        return self.sensor.camera_type

    @property
    def camera(self):
        return self.sensor.camera

    @property
    def n_data_points(self):
        return self.sensor.n_data_points

    @property
    def events_on(self):
        return self.sensor.events_on

    @property
    def events_off(self):
        return self.sensor.events_off
   
    @property
    def frames(self):
        return self.sensor.frames

    @property
    def save_events_video(self):
        return self.save_events_video
   
    @save_events_video.setter
    def save_events_video(self, save_events_video):
        self.sensor.save_events_video = save_events_video

    @property
    def save_acc_video(self):
        return self.save_acc_video
   
    @save_acc_video.setter
    def save_acc_video(self, save_acc_video):
        self.sensor.save_acc_video = save_acc_video

    @property
    def events_on_filename(self):
        return self.events_on_filename
   
    @events_on_filename.setter
    def events_on_filename(self, events_on_filename):
        self.sensor.events_on_filename = events_on_filename
   
    @property
    def events_off_filename(self):
        return self.events_off_filename
   
    @events_off_filename.setter
    def events_off_filename(self, events_off_filename):
        self.sensor.events_off_filename = events_off_filename
   
    @property
    def events_video_filename(self):
        return self.events_video_filename
   
    @events_video_filename.setter
    def events_video_filename(self, events_video_filename):
        self.sensor.events_video_filename = events_video_filename
   
    @property
    def acc_video_filename(self):
        return self.acc_video_filename
   
    @acc_video_filename.setter
    def acc_video_filename(self, acc_video_filename):
        self.sensor.acc_video_filename = acc_video_filename
   
    @property
    def display(self):
        return self.display
   
    @display.setter
    def display(self, display):
        self.sensor.display = display

    # Sensor attributes not needed to access and therefore not exposed
    # self.starttime
    # self.event_store
    # self.noise_filter
    # self.thread_run

    def reset_variables(self):
        self.sensor.reset_variables()

    def start_logging(self):
        print('Started recording')
        self.sensor.thread_run = True
   
    @oneway
    def stop_logging(self):
        self.sensor.thread_run = False
        print('Stopped recording')  

    @oneway
    def get_events(self):
        self.sensor.get_events()

    def get_frames(self):
        self.sensor.get_frames()

    def value_cleanup(self):
        self.sensor.value_cleanup()

    def set_filenames(self, events_on_file = None, events_off_file = None, events_video_file = None, acc_video_file = None, frames = None) -> None:
        self.sensor.set_filenames(events_on_file=events_on_file, events_off_file=events_off_file, events_video_file=events_video_file, acc_video_file=acc_video_file, frames=frames)

    def save_events_on(self, path=None):
        self.sensor.save_events_on(path)

    def save_events_off(self, path=None):
        self.sensor.save_events_off(path)

    def save_frames(self, path=None):
        self.sensor.save_frames(path)
        
    def close(self):
        return


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-r", "--robot-ip", type=str,
                        default="172.16.0.2", help="robot IP address")
    parser.add_argument("-n", "--service-name", type=str,
                        default="neurotac_service_1", help="service name")
    parser.add_argument("-i", "--host-ip", type=str,
                        default="127.0.0.1", help="host IP address")
    parser.add_argument("-p", "--host-port", type=int,
                        default=9092, help="host port number")
    args = vars(parser.parse_args())
    robot_ip = args["robot_ip"]
    service_name = args["service_name"]
    host_ip = args["host_ip"]
    host_port = args["host_port"]

    # Run background bash script that opens pyro namespace
    # This process is closed upon script exit
    print("Starting namespace server...")
    subprocess.Popen(["pyro5-ns"])
    time.sleep(2)

    with Daemon(host=host_ip, port=host_port) as daemon, locate_ns() as ns:
        print(f"Starting service {service_name} ...")
        service = NeuroTacService(save_events_video=True)
        service_uri = daemon.register(service)
        ns.register(service_name, service_uri)
        print(
            f"Service {service_name} running (press CTRL-C to terminate) ...")
        daemon.requestLoop()
        

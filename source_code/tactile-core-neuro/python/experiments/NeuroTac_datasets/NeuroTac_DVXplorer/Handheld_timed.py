# Import only necessary libraries
import os
import time
import json
import threading
import tkinter as tk
from tkinter import ttk

from core.sensor.tactile_sensor_neuro import NeuroTac

sensor_type = 'NeuroTac_DVXplorer'  # NeuroTac version

class DataCollector:
    def __init__(self, sensor_type):
        self.sensor_type = sensor_type
        self.sensor = NeuroTac(save_events_video=False, save_acc_video=False, display=False)
        self.trial_idx = 0
        self.obj_idx = 0
        self.total_time = 5
        self.root = None
        self.progress_bar = None
        self.progress_var = None
        self.remaining_label = None
        self.obj_label = None
        self.t = None

    def create_gui(self):
            self.root = tk.Tk()
            self.root.title("NeuroTac Data")

            frame = ttk.Frame(self.root)
            frame.grid(row=0, column=0, padx=10, pady=10)

            self.progress_var = tk.DoubleVar()  # Create progress_var as an instance variable
            self.progress_bar = ttk.Progressbar(frame, variable=self.progress_var, length=200)
            self.progress_bar.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

            self.remaining_label = tk.Label(frame, text=f"Remaining: {int(self.total_time)} seconds")
            self.remaining_label.grid(row=1, column=1, padx=10, pady=10)

            start_button = ttk.Button(frame, text="Start Recording", command=self.start_recording)
            start_button.grid(row=1, column=0, padx=10, pady=10)

            obj_button = ttk.Button(frame, text="Next Object", command=self.next_object)
            obj_button.grid(row=2, column=0, padx=10, pady=10)

            self.obj_label = tk.Label(frame, text=f"Object: {int(self.obj_idx)}  Trial: {int(self.trial_idx)}")
            self.obj_label.grid(row=2, column=1, padx=10, pady=10)

            self.update_sensor_variables()

            self.root.mainloop()

    def update_sensor_variables(self):
        # Your start recording logic here
        self.sensor.reset_variables()
        events_on_file = os.path.join(self.events_dir,'taps_object_' + str(self.obj_idx) + '_trial_' +str(self.trial_idx) + '_events_on')
        events_off_file = os.path.join(self.events_dir,'taps_object_' + str(self.obj_idx) + '_trial_' +str(self.trial_idx) + '_events_off')
        events_video = os.path.join(self.video_dir,'event_stream_object_' + str(self.obj_idx) + '_trial_' +str(self.trial_idx) + '.mp4')
        acc_video = os.path.join(self.video_dir,'accumulator_object_' + str(self.obj_idx) + '_trial_' +str(self.trial_idx) + '.mp4')                    
        self.sensor.set_filenames(events_on_file = events_on_file, events_off_file = events_off_file, events_video_file = events_video, acc_video_file = acc_video)

    def start_recording(self):      
        # Start sensor recording
        self.sensor.start_logging()

        # self.t = threading.Thread(target=self.sensor.get_events, args = ())
        # self.t.start()
        
        self.start_time = time.time()
        self.update_progress()
        # self.end_recording()

    def update_progress(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        remaining_time = self.total_time - elapsed_time

        if elapsed_time >= self.total_time:
            self.progress_var.set(100)
            self.remaining_label.config(text="Completed")
            self.end_recording()
            return

        # for i in range(5):
        #     time.sleep(0.95)
        #     print(str(i+1))
        # self.remaining_label.config(text="Completed")
        progress = (elapsed_time / self.total_time) * 100

        self.progress_var.set(int(progress))
        self.remaining_label.config(text=f"Remaining: {int(remaining_time)} seconds")
        self.root.after(100, self.update_progress)

    def end_recording(self):
        # Your end recording logic here
            # Stop sensor recording
        self.sensor.stop_logging()
        # self.t.join()
        
        # Collate proper timestamp values in ms.
        self.sensor.value_cleanup()

        # Save data
        self.sensor.save_events_on()
        self.sensor.save_events_off()

        # print("saved data")
        self.trial_idx += 1
        self.obj_label.config(text=f"Object: {int(self.obj_idx)} Trial: {int(self.trial_idx)} ")
        self.update_sensor_variables()

    def next_object(self):
        self.obj_idx +=1
        self.trial_idx = 0
        self.obj_label.config(text=f"Object: {int(self.obj_idx)} Trial: {int(self.trial_idx)} ")
        self.update_sensor_variables()

    def main(self):
        collect_dir_name = os.path.join(os.path.basename(__file__)[:-3], os.path.basename(__file__)[:-3] + '_' + time.strftime('%m%d%H%M'))
        collect_dir = os.path.join(os.environ['DATAPATH'], sensor_type, collect_dir_name)
        self.video_dir = os.path.join(collect_dir, 'videos')
        self.events_dir = os.path.join(collect_dir, 'events')

        os.makedirs(collect_dir, exist_ok=True)
        os.makedirs(self.video_dir, exist_ok=True)
        os.makedirs(self.events_dir, exist_ok=True)

        meta = self.make_meta()
        with open(os.path.join(collect_dir, 'meta.json'), 'w') as f:
            json.dump(meta, f)

        self.create_gui()

    def make_meta(self, tap_time=10, n_objects=2, n_trials=2):
        meta = locals().copy()
        meta['sensor'] = self.sensor_type
        del meta['self']
        return meta

if __name__ == '__main__':
    collector = DataCollector(sensor_type)
    collector.main()

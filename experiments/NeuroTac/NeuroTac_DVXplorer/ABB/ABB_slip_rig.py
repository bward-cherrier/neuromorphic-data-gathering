import os, time, json, collections
import pickle as pkl
from cri.robot import SyncRobot, AsyncRobot
from cri.controller import ABBController

from vsp.video_stream import CvVideoDisplay, CvVideoOutputFile
from vsp.processor import AsyncProcessor
import sys
sys.path.append("python")
from core.sensor.tactile_capture_timestamp import CaptureTimestamp, MyCameraStreamProcessorMT
from core.sensor.tactile_sensor_neuro import NeuroTac_DAVIS240

def make_meta(meta_file,          
              robot_tcp = [0, 0, 89, 0, 0, 0], # Size of the TacTip (tcp = tool center point)
              base_frame = [0, 0, 0, 0, 0, 0], # Origin of the base frame is at centre of the ABB robot base
              home_pose = [400, 0, 340, 180, 0, 180], # Starting point of the robot when switched on
              linear_speed = 10, # Robot's speed for linear movements
              angular_speed = 5, # Robot's speed for rotation movements
              slide_speeds: [0.1,0.2,0.3,0.4], # Speeds at which the ABB arm moves back
              slide_depths: [5], # Depth of contact with sliding surface
              n_trials = 1, # Number of taps at each pose
              ):
  data_dir = os.path.dirname(meta_file)

  meta = locals().copy()
  del meta['data_dir']
  return meta

meta_dict = {
    'sensor_caps': 'FP'}

def make_robot():
  return AsyncRobot(SyncRobot(ABBController(ip='192.168.125.1'), port=5000))

def make_sensor():  
  return NeuroTac_DVXPlorer(port=7777)

def make_camera(device_path="/dev/video0", display=CvVideoDisplay(name='preview'), frame_size=(640, 480), fps=70.0, cut=None):
    return AsyncProcessor(MyCameraStreamProcessorMT(
        camera=CaptureTimestamp(device_path=device_path, frame_size=frame_size, fourcc='mp4v',
                                num_buffers=2, is_color=True, cut=cut),
        display=display,
        writer=CvVideoOutputFile(filename=os.path.join(video_dir, 'demo.mp4'), fps=fps,
                                 is_color=True, fourcc_code='mp4v', frame_size=frame_size),
    ))

def collect_data(neurotac_data_dir, video_data_dir, tap_move, obj_poses, home_pose, base_frame, work_frame,robot_tcp,linear_speed,angular_speed,n_trials,**kwargs):

# Check depth isn't going to damage tip
if 10 + meta_dict['slide_depths'][0] > 18:
    raise Exception("Risk of damaging tactip...quitting")

num_samples = meta_dict['n_trias'] * len(meta_dict['slide_depths']) * len(meta_dict['slide_speeds'])

with make_robot() as robot, make_sensor() as sensor, make_camera(device_path="/dev/video2", display=None) as out_cam:

    # Set robot speeds & TCP
    robot.tcp = meta_dict['robot_tcp']
    robot.linear_speed = meta_dict['linear_speed']
    robot.angular_speed = meta_dict['angular_speed']
    
    # Avoid singularities if not starting at usual home    
    if robot.joint_angles[4] < 0.1:
        robot.move_linear((400,0,300,robot.pose[3],robot.pose[4],robot.pose[5]))
        robot.move_joints((0,20.1,0.8,0,69.1,0))

    # Move robot to horizontal position
    robot.move_linear(meta_dict['home_pos'])  # Move home
    robot.move_linear((263.4, -281.4, 220, 180, 0, 180))
    robot.move_joints((-45.9,69.2,7.8,-47.8,-81.0,-15.0))
    time.sleep(1)

    # Start camera
    pt2 = out_cam.process(num_frames=5)
    
    # Move to origin of work frame
    meta_dict['work_frame'] = (robot.pose[0], robot.pose[1], robot.pose[2], robot.coord_frame[3], robot.coord_frame[4], robot.coord_frame[5])
    time.sleep(1)
    robot.coord_frame = meta_dict['work_frame']  # Change coord frame
    print("Moving to origin of work frame ...")

    time.sleep(1)
    # data = collections.deque()
    ts_dict = {}; acc_dict = {}
    for di, d in enumerate(meta_dict['slide_depths']):
        for si, s in enumerate(meta_dict['slide_speeds']):
            for r in range(meta_dict['n_trials']):
                print("depth: %.2f, speed: %.2f, run: %d" % (d, s, r))
                robot.move_linear((0, 0, 0, 0, 90, 0))
                print("touching object")
                robot.move_linear((10 + d, 0, 0, 0, 90, 0))  # Contact object
                time.sleep(1)
                robot.linear_speed = 4
                print("lifting object")
                robot.move_linear((10 + d, 0, 65, 0, 90, 0))#106, 90, 0))  # Lift up
                time.sleep(2)
                sensor.async_process(outfile=os.path.join(neurotac_data_dir, 'd' + str(d) + '_s' + str(s) + '_r' + str(
                    r) + '.mp4'))  # Start pin rec
                out_cam.async_process(outfile=os.path.join(video_data_dir, 'd' + str(d) + '_s' + str(s) + '_r' + str(
                    r) + '_out_cam.mp4'), num_frames=3000)  # Start exterior cam rec
                time.sleep(1)
                robot.linear_speed = s
                print("dropping object")
                robot.move_linear((10 + d - 4, 0, 65, 0, 90, 0))
                sensor.async_cancel()
                out_cam.async_cancel()
                time.sleep(1)
                _, ts = sensor.async_result()
                _, ts_out = out_cam.async_result()

                ts_dict.update({'d' + str(d) + '_s' + str(s) + '_r' + str(r): ts})
                ts_dict.update({'d' + str(d) + '_s' + str(s) + '_r' + str(r) +'_out_cam': ts_out})
                ts = None
                ts_out = None

                robot.linear_speed = meta_dict['robot_speed']
                robot.move_linear((0, 0, 65, 0, 90, 0))


    print("Moving home ...")
    robot.coord_frame = meta_dict['base_frame']  # Change to base frame
    # avoids singularities
    robot.move_linear((400,0,300,robot.pose[3],robot.pose[4],robot.pose[5]))
    robot.move_joints((0,20.1,0.8,0,69.1,0))
    # move home
    robot.move_linear(meta_dict['home_pos'])  # Move home

    # save data
    with open(ts_data, 'wb') as t:
        pkl.dump(ts_dict, t)

# Create metadata JSON file
video_target_file = os.path.join(collect_dir, 'targets_video.csv')
ts_data = os.path.join(collect_dir, 'ts_data_uncut.pkl')


    # Save metadata
    os.makedirs(data_dir)
    os.makedirs(video_dir)
    with open(os.path.join(collect_dir, 'meta.json'), 'w') as f:
        json.dump(meta_dict, f)


def main():

  # Make general folder for data storage and save metadata
  data_dir = os.path.join(os.environ['DATAPATH'], 'NeuroTac_DVXplorer', os.path.basename(__file__)[:-3], os.path.basename(__file__)[:-3] +'_'+time.strftime('%m%d%H%M'))
  meta_file = os.path.join(data_dir, 'meta.json')
  meta = make_meta(meta_file) 
  os.makedirs(os.path.dirname(meta_file))
  with open(meta_file, 'w') as f: 
    json.dump(meta, f)   

  # Make folders to store neurotac and camera data
  neurotac_data_dir = os.path.join(data_dir, 'neurotac_data')
  video_data_dir = os.path.join(data_dir, 'video_data')
  os.makedirs(neurotac_data_dir)
  os.makedirs(video_data_dir)

  # Collect data
  collect_data(neurotac_data_dir, video_data_dir,**meta)

if __name__ == '__main__':
    main()


    """ CAMERA SETTINGS
    Bri:34 Cont:10 Sat:0 Hue:0 Gamma:100 Gain:16 WB:4600 Shar:2 Backlight:1 Exp:156
    Bri:-49 Cont:36 Sat:0 Hue:0 Gamma:100 Gain:16 WB:4600 Shar:2 Backlight:1 Exp:156 - New
    """
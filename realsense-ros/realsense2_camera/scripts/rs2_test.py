# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import signal
import shlex
from rs2_listener import CWaitForMessage
import rclpy
from rclpy.node import Node
from importRosbag.importRosbag import importRosbag
import numpy as np
import tf2_ros
import itertools
import subprocess
import time

global tf_timeout
tf_timeout = 5

def ImuGetData(rec_filename, topic):
    # res['value'] = first value of topic.
    # res['max_diff'] = max difference between returned value and all other values of topic in recording.

    res = dict()
    res['value'] = None
    res['max_diff'] = [0,0,0]
    data = importRosbag(rec_filename, importTopics=[topic], log='ERROR', disable_bar=True)[topic]
    res['value'] = data['acc'][0,:]
    res['max_diff'] = data['acc'].max(0) - data['acc'].min(0)
    return res

def AccelGetData(rec_filename):
    return ImuGetData(rec_filename, '/device_0/sensor_2/Accel_0/imu/data')

def AccelGetDataDeviceStandStraight(rec_filename):
    gt_data = AccelGetData(rec_filename)
    gt_data['ros_value'] = np.array([0.63839424, 0.05380408, 9.85343552])
    gt_data['ros_max_diff'] = np.array([0.06940174, 0.04032778, 0.05982018])
    return gt_data

def ImuTest(data, gt_data):
    # check that the imu data received is the same as in the recording. 
    # check that in the rotated imu received the g-accelartation is pointing up according to ROS standards.
    try:
        v_data = np.array([data['value'][0].x, data['value'][0].y, data['value'][0].z])
        v_gt_data = gt_data['value']
        diff = v_data - v_gt_data
        max_diff = abs(diff).max()
        msg = 'original accel: Expect max diff of %.3f. Got %.3f.' % (gt_data['max_diff'].max(), max_diff)
        print (msg)
        if max_diff > gt_data['max_diff'].max():
            return False, msg

        v_data = np.array(data['ros_value']).mean(0)
        v_gt_data = gt_data['ros_value']
        diff = v_data - v_gt_data
        max_diff = abs(diff).max()
        msg = 'rotated to ROS: Expect max diff of %.3f. Got %.3f.' % (gt_data['ros_max_diff'].max(), max_diff)
        print (msg)
        if max_diff > gt_data['ros_max_diff'].max():
            return False, msg
    except Exception as e:
        msg = '%s' % e
        print ('Test Failed: %s' % msg)
        return False, msg
    return True, ''

def ImageGetData(rec_filename, topic):
    all_avg = []
    ok_percent = []
    res = dict()

    data = importRosbag(rec_filename, importTopics=[topic], log='ERROR', disable_bar=True)[topic]
    for pyimg in data['frames']:
        ok_number = (pyimg != 0).sum()
        ok_percent.append(float(ok_number) / (pyimg.shape[0] * pyimg.shape[1]))
        all_avg.append(pyimg.sum() / ok_number)

    all_avg = np.array(all_avg)

    channels = pyimg.shape[2] if len(pyimg.shape) > 2 else 1
    res['num_channels'] = channels
    res['shape'] = pyimg.shape
    res['avg'] = all_avg.mean()
    res['ok_percent'] = {'value': (np.array(ok_percent).mean()) / channels, 'epsilon': 0.01}
    res['epsilon'] = max(all_avg.max() - res['avg'], res['avg'] - all_avg.min())
    res['reported_size'] = [pyimg.shape[1], pyimg.shape[0], pyimg.shape[1]*pyimg.dtype.itemsize*channels]

    return res


def ImageColorGetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_1/Color_0/image/data')


def ImageDepthGetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_0/Depth_0/image/data')


def ImageDepthInColorShapeGetData(rec_filename):
    gt_data = ImageDepthGetData(rec_filename)
    color_data = ImageColorGetData(rec_filename)
    gt_data['shape'] = color_data['shape'][:2]
    gt_data['reported_size'] = color_data['reported_size']
    gt_data['reported_size'][2] = gt_data['reported_size'][0]*2
    gt_data['ok_percent']['epsilon'] *= 3
    return gt_data

def ImageDepthGetData_decimation(rec_filename):
    gt_data = ImageDepthGetData(rec_filename)
    gt_data['shape'] = [x/2 for x in gt_data['shape']]
    gt_data['reported_size'] = [x/2 for x in gt_data['reported_size']]
    gt_data['epsilon'] *= 3
    return gt_data

def ImageColorTest(data, gt_data):
    # check that all data['num_channels'] are the same as gt_data['num_channels'] and that avg value of all
    # images are within epsilon of gt_data['avg']
    try:
        if ('num_channels' not in data):
            msg = 'No data received'
            return False, msg
        channels = list(set(data['num_channels']))
        msg = 'Expect %d channels. Got %d channels.' % (gt_data['num_channels'], channels[0])
        print (msg)
        if len(channels) > 1 or channels[0] != gt_data['num_channels']:
            return False, msg
        msg = 'Expected all received images to be the same shape. Got %s' % str(set(data['shape']))
        print (msg)
        if len(set(data['shape'])) > 1:
            return False, msg
        msg = 'Expected shape to be %s. Got %s' % (gt_data['shape'], list(set(data['shape']))[0])
        print (msg)
        if (np.array(list(set(data['shape']))[0]) != np.array(gt_data['shape'])).any():
            return False, msg
        msg = 'Expected header [width, height, step] to be %s. Got %s' % (gt_data['reported_size'], list(set(data['reported_size']))[0])
        print (msg)
        if (np.array(list(set(data['reported_size']))[0]) != np.array(gt_data['reported_size'])).any():
            return False, msg
        msg = 'Expect average of %.3f (+-%.3f). Got average of %.3f.' % (gt_data['avg'].mean(), gt_data['epsilon'], np.array(data['avg']).mean())
        print (msg)
        if abs(np.array(data['avg']).mean() - gt_data['avg'].mean()) > gt_data['epsilon']:
            return False, msg

        msg = 'Expect no holes percent > %.3f. Got %.3f.' % (gt_data['ok_percent']['value']-gt_data['ok_percent']['epsilon'], np.array(data['ok_percent']).mean())
        print (msg)
        if np.array(data['ok_percent']).mean() < gt_data['ok_percent']['value']-gt_data['ok_percent']['epsilon']:
            return False, msg

    except Exception as e:
        msg = '%s' % e
        print ('Test Failed: %s' % msg)
        return False, msg
    return True, ''


def ImageColorTest_3epsilon(data, gt_data):
    gt_data['epsilon'] *= 3
    return ImageColorTest(data, gt_data)

def NotImageColorTest(data, gt_data):
    res = ImageColorTest(data, gt_data)
    return (not res[0], res[1])

def PointCloudTest(data, gt_data):
    width = np.array(data['width']).mean()
    height = np.array(data['height']).mean()
    msg = 'Expect image size %d(+-%d), %d. Got %d, %d.' % (gt_data['width'][0], gt_data['width'][1], gt_data['height'][0], width, height)
    print (msg)
    if abs(width - gt_data['width'][0]) > gt_data['width'][1] or height != gt_data['height'][0]:
        return False, msg
    mean_pos = np.array([xx[:3] for xx in data['avg']]).mean(0)
    msg = 'Expect average position of %s (+-%.3f). Got average of %s.' % (gt_data['avg'][0][:3], gt_data['epsilon'][0], mean_pos)
    print (msg)
    if abs(mean_pos - gt_data['avg'][0][:3]).max() > gt_data['epsilon'][0]:
        return False, msg
    mean_col = np.array([xx[3:] for xx in data['avg']]).mean(0)
    msg = 'Expect average color of %s (+-%.3f). Got average of %s.' % (gt_data['avg'][0][3:], gt_data['epsilon'][1], mean_col)
    print (msg)
    if abs(mean_col - gt_data['avg'][0][3:]).max() > gt_data['epsilon'][1]:
        return False, msg

    return True, ''


def staticTFTest(data, gt_data):
    for couple in gt_data.keys():
        if data[couple] is None:
            msg = 'Tf is None for couple %s' % '->'.join(couple)
            return False, msg
        temp = data[couple].translation
        np_trans = np.array([temp.x, temp.y, temp.z])
        temp = data[couple].rotation
        np_rot = np.array([temp.x, temp.y, temp.z, temp.w])
        if any(abs(np_trans - gt_data[couple][0]) > 1e-5) or \
           any(abs(np_rot - gt_data[couple][1]) > 1e-5):
           msg = 'Tf is changed for couple %s' % '->'.join(couple)
           return False, msg
    return True, ''

test_types = {'vis_avg': {'listener_theme': 'colorStream',
                          'data_func': ImageColorGetData,
                          'test_func': ImageColorTest},
              'depth_avg': {'listener_theme': 'depthStream',
                            'data_func': ImageDepthGetData,
                            'test_func': ImageColorTest},
              'no_file': {'listener_theme': 'colorStream',
                          'data_func': lambda x: None,
                          'test_func': NotImageColorTest},
              'pointscloud_avg': {'listener_theme': 'pointscloud',
                          'data_func': lambda x: {'width': [660353, 2300], 'height': [1], 'avg': [np.array([ 1.28251814, -0.15839984, 4.82235184, 80, 160, 240])], 'epsilon': [0.04, 5]},
                          'test_func': PointCloudTest},
              'align_depth_ir1': {'listener_theme': 'alignedDepthInfra1',
                                  'data_func': ImageDepthGetData,
                                  'test_func': ImageColorTest},
              'align_depth_color': {'listener_theme': 'alignedDepthColor',
                                   'data_func': ImageDepthInColorShapeGetData,
                                   'test_func': ImageColorTest_3epsilon},
              'depth_avg_decimation': {'listener_theme': 'depthStream',
                                   'data_func': ImageDepthGetData_decimation,
                                   'test_func': ImageColorTest},
              'align_depth_ir1_decimation': {'listener_theme': 'alignedDepthInfra1',
                                  'data_func': ImageDepthGetData,
                                  'test_func': ImageColorTest},
              'static_tf': {'listener_theme': 'static_tf',
                                  'data_func': lambda x: {('camera_link', 'camera_color_frame'): ([-0.00010158783697988838, 0.014841210097074509, -0.00022671300393994898], [-0.0008337442995980382, 0.0010442184284329414, -0.0009920650627464056, 0.9999986290931702]), 
                                                          ('camera_link', 'camera_depth_frame'): ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), 
                                                          ('camera_link', 'camera_infra1_frame'): ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), 
                                                          ('camera_depth_frame', 'camera_infra1_frame'): ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), 
                                                          ('camera_depth_frame', 'camera_color_frame'): ([-0.00010158783697988838, 0.014841210097074509, -0.00022671300393994898], [-0.0008337442995980382, 0.0010442184284329414, -0.0009920650627464056, 0.9999986290931702]), 
                                                          ('camera_infra1_frame', 'camera_color_frame'): ([-0.00010158783697988838, 0.014841210097074509, -0.00022671300393994898], [-0.0008337442995980382, 0.0010442184284329414, -0.0009920650627464056, 0.9999986290931702])}
                                                            ,
                                  'test_func': staticTFTest},
              'accel_up':   {'listener_theme': 'accelStream',
                                  'data_func': AccelGetDataDeviceStandStraight,
                                  'test_func': ImuTest},
              }


def run_test(test, listener_res):
    # gather ground truth with test_types[test['type']]['data_func'] and recording from test['rosbag_filename']
    # return results from test_types[test['type']]['test_func']
    test_type = test_types[test['type']]
    gt_data = test_type['data_func'](test['params']['rosbag_filename'])
    return test_type['test_func'](listener_res[test_type['listener_theme']], gt_data)


def print_results(results):
    title = 'TEST RESULTS'
    headers = ['index', 'test name', 'score', 'message']
    col_0_width = len(headers[0]) + 1
    col_1_width = max([len(headers[1])] + [len(test[0]) for test in results]) + 1
    col_2_width = max([len(headers[2]), len('OK'), len('FAILED')]) + 1
    col_3_width = max([len(headers[3])] + [len(test[1][1]) for test in results]) + 1
    total_width = col_0_width + col_1_width + col_2_width + col_3_width
    print
    print (('{:^%ds}'%total_width).format(title))
    print ('-'*total_width)
    print (('{:<%ds}{:<%ds}{:>%ds} : {:<%ds}' % (col_0_width, col_1_width, col_2_width, col_3_width)).format(*headers))
    print ('-'*(col_0_width-1) + ' '*1 + '-'*(col_1_width-1) + ' '*2 + '-'*(col_2_width-1) + ' '*3 + '-'*(col_3_width-1))
    print ('\n'.join([('{:<%dd}{:<%ds}{:>%ds} : {:<s}' % (col_0_width, col_1_width, col_2_width)).format(idx, test[0], 'OK' if test[1][0] else 'FAILED', test[1][1]) for idx, test in enumerate(results)]))
    print


def get_tfs(coupled_frame_ids):
    res = dict()
    tfBuffer = tf2_ros.Buffer()
    node = Node('tf_listener')
    listener = tf2_ros.TransformListener(tfBuffer, node)
    rclpy.spin_once(node)
    for couple in coupled_frame_ids:
        from_id, to_id = couple
        if (tfBuffer.can_transform(from_id, to_id, rclpy.time.Time(), rclpy.time.Duration(nanoseconds=3e6))):
            res[couple] = tfBuffer.lookup_transform(from_id, to_id, rclpy.time.Time(), rclpy.time.Duration(nanoseconds=1e6)).transform
        else:
            res[couple] = None
    return res

def kill_realsense2_camera_node():
    cmd = "kill -s INT $(ps aux | grep '[r]ealsense2_camera_node' | awk '{print $2}')"
    os.system(cmd)

def run_tests(tests):
    msg_params = {'timeout_secs': 5}
    results = []
    params_strs = set([test['params_str'] for test in tests])
    for params_str in params_strs:
        rclpy.init()
        rec_tests = [test for test in tests if test['params_str'] == params_str]
        themes = [test_types[test['type']]['listener_theme'] for test in rec_tests]
        msg_retriever = CWaitForMessage(msg_params)
        print ('*'*30)
        print ('Running the following tests: %s' % ('\n' + '\n'.join([test['name'] for test in rec_tests])))
        print ('*'*30)
        num_of_startups = 5
        is_node_up = False
        for run_no in range(num_of_startups):
            print 
            print ('*'*8 + ' Starting ROS ' + '*'*8)
            print ('running node (%d/%d)' % (run_no, num_of_startups))
            cmd_params = ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py'] + params_str.split(' ')
            print ('running command: ' + ' '.join(cmd_params))
            p_wrapper = subprocess.Popen(cmd_params, stdout=None, stderr=None)
            time.sleep(2)
            service_list = subprocess.check_output(['ros2', 'node', 'list']).decode("utf-8")
            is_node_up = '/camera/camera' in service_list
            if is_node_up:
                print ('Node is UP')
                break

            print ('Node is NOT UP')
            print ('*'*8 + ' Killing ROS ' + '*'*9)
            try:
                p_wrapper.terminate()
                p_wrapper.wait(timeout=2)
            except subprocess.TimeoutExpired:
                kill_realsense2_camera_node()
            print ('DONE')

        if is_node_up:
            listener_res = msg_retriever.wait_for_messages(themes)
            if 'static_tf' in [test['type'] for test in rec_tests]:
                print ('Gathering static transforms')
                frame_ids = ['camera_link', 'camera_depth_frame', 'camera_infra1_frame', 'camera_infra2_frame', 'camera_color_frame']
                coupled_frame_ids = [xx for xx in itertools.combinations(frame_ids, 2)]
                listener_res['static_tf'] = get_tfs(coupled_frame_ids)

            print ('*'*8 + ' Killing ROS ' + '*'*9)
            kill_realsense2_camera_node()
            p_wrapper.wait()
            print ('*'*8 + ' Killed ' + '*'*9)
        else:
            listener_res = dict([[theme_name, {}] for theme_name in themes])

        rclpy.shutdown()
        print ('*'*30)
        print ('DONE run')
        print ('*'*30)

        for test in rec_tests:
            try:
                res = run_test(test, listener_res)
            except Exception as e:
                print ('Test %s Failed: %s' % (test['name'], e))
                res = False, '%s' % e
            results.append([test['name'], res])

    return results

def main():
    outdoors_filename = './records/outdoors_1color.bag'
    all_tests = [{'name': 'non_existent_file', 'type': 'no_file', 'params': {'rosbag_filename': '/home/non_existent_file.txt'}},
                 {'name': 'vis_avg_2', 'type': 'vis_avg', 'params': {'rosbag_filename': outdoors_filename}},
                 {'name': 'depth_avg_1', 'type': 'depth_avg', 'params': {'rosbag_filename': outdoors_filename}},
                 #{'name': 'points_cloud_1', 'type': 'pointscloud_avg', 'params': {'rosbag_filename': outdoors_filename, 'pointcloud.enable': 'true'}},
                 {'name': 'depth_w_cloud_1', 'type': 'depth_avg', 'params': {'rosbag_filename': outdoors_filename, 'pointcloud.enable': 'true'}},
                 {'name': 'align_depth_color_1', 'type': 'align_depth_color', 'params': {'rosbag_filename': outdoors_filename, 'align_depth.enable':'true'}},
                 {'name': 'align_depth_ir1_1', 'type': 'align_depth_ir1', 'params': {'rosbag_filename': outdoors_filename, 'align_depth.enable': 'true', 'enable_infra1':'true', 'enable_infra2':'true'}},
                 {'name': 'depth_avg_decimation_1', 'type': 'depth_avg_decimation', 'params': {'rosbag_filename': outdoors_filename, 'decimation_filter.enable':'true'}},
                 {'name': 'align_depth_ir1_decimation_1', 'type': 'align_depth_ir1_decimation', 'params': {'rosbag_filename': outdoors_filename, 'align_depth.enable':'true', 'decimation_filter.enable':'true'}},
                 {'name': 'static_tf_1', 'type': 'static_tf', 'params': {'rosbag_filename': outdoors_filename, 'enable_infra1':'true', 'enable_infra2':'true'}}, 
                 {'name': 'accel_up_1', 'type': 'accel_up', 'params': {'rosbag_filename': './records/D435i_Depth_and_IMU_Stands_still.bag', 'enable_accel': 'true', 'accel_fps': '0.0'}},
        ]

    # Normalize parameters:
    for test in all_tests:
        test['params']['rosbag_filename'] = os.path.abspath(test['params']['rosbag_filename'])
        test['params']['color_width'] = '0'
        test['params']['color_height'] = '0'
        test['params']['depth_width'] = '0'
        test['params']['depth_height'] = '0'
        test['params']['infra_width'] = '0'
        test['params']['infra_height'] = '0'
        test['params_str'] = ' '.join([key + ':=' + test['params'][key] for key in sorted(test['params'].keys())])

    if len(sys.argv) < 2 or '--help' in sys.argv or '/?' in sys.argv:
        print ('USAGE:')
        print ('------')
        print ('rs2_test.py --all | <test_name> [<test_name> [...]]')
        print
        print ('Available tests are:')
        print ('\n'.join([test['name'] for test in all_tests]))
        exit(-1)

    if '--all' in sys.argv[1:]:
        tests_to_run = all_tests
    else:
        tests_to_run = [test for test in all_tests if test['name'] in sys.argv[1:]]

    results = run_tests(tests_to_run)
    print_results(results)

    res = int(all([result[1][0] for result in results])) - 1
    print ('exit (%d)' % res)
    exit(res)

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 11 17:25:42 2022

@author: admin
"""


import torch, os, cv2, sys
from model.model import parsingNet
from utils.common import merge_config
import scipy.special
import numpy as np
import torchvision.transforms as transforms
from data.constant import culane_row_anchor, tusimple_row_anchor

### Tracker
cur_file_dir = os.path.dirname(os.path.realpath(__file__))
### RingroadMap
sys.path.append(os.path.join(cur_file_dir, 'utils'))
from utils.tf_transforms import euler_from_quaternion

###ROS Things
sys.path.append('/usr/lib/python2.7/dist-packages')
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry

from lane_detection_msgs.msg import CNNLaneDetectionMsg

image_half_rate_flag = True
img_msg_test = None
def img_callback(img_msg):
    print('get one image')
    global heading_rate_old, velocity_old, timestamp_old, odom_info, image_half_rate_flag
    global img_msg_test
    img_msg_test = img_msg
    image_half_rate_flag = not image_half_rate_flag
    if image_half_rate_flag:
        start_time = rospy.get_rostime().to_sec()
        phi_imu = 0
        delta_heading = 0
        delta_distance = 0
        
        if odom_info is not None:
            q = odom_info.pose.pose.orientation
            phi_imu = np.rad2deg(euler_from_quaternion([q.x, q.y, q.z, q.w])[1])
            heading_rate_new = odom_info.twist.twist.angular.z
            velocity_new = (odom_info.twist.twist.linear.x**2 + odom_info.twist.twist.linear.y**2)**0.5
            timestamp_new = img_msg.header.stamp.secs + img_msg.header.stamp.nsecs/1e9
            
        if heading_rate_old is not None:
            delta_time = min(abs(timestamp_new - timestamp_old),0.2)
            delta_heading = (heading_rate_old + heading_rate_new) * delta_time / 2
            delta_distance = (velocity_new + velocity_old) * delta_time / 2 #TODO: check speed unit (m/s)

        start_time_ = rospy.get_rostime().to_sec()
        # image = cvbridge.compressed_imgmsg_to_cv2( img_msg, desired_encoding='rgb8')
        if is_compressed_image:
            image = cv2.imdecode(np.frombuffer(img_msg.data, np.uint8), cv2.IMREAD_COLOR)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # image = cvbridge.compressed_imgmsg_to_cv2( img_msg, desired_encoding='rgb8')
        else:
            #image = cvbridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
            image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # resized_image = cv2.resize(image, (800, 288))
        preprocessed_image = img_transforms(cv2.resize(image, (800, 288)))
        preprocessed_image = preprocessed_image[None]
        image_gpu = preprocessed_image.cuda()
        print('preprocess time: %.4f ms'%( (rospy.get_rostime().to_sec()-start_time_)*1e3) )

        start_time_ = rospy.get_rostime().to_sec()
        with torch.no_grad():
            out = net(image_gpu)
        print('cnn inference time: %.4f ms'%( (rospy.get_rostime().to_sec()-start_time_)*1e3) )
        
        out_j = out[0].data.cpu().numpy()
        out_j = out_j[:, ::-1, :]
        prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)
        idx = np.arange(cfg.griding_num) + 1
        idx = idx.reshape(-1, 1, 1)
        loc = np.sum(prob * idx, axis=0)
        no_marker_flag = np.argmax(out_j, axis=0)
        loc[no_marker_flag == cfg.griding_num] = 0
        #TODO: reject the detection whose max prob is not large enough
        max_prob = np.max(prob, axis=0)
        loc[max_prob < 0.25] = 0
        out_j = loc
        
        # out_j = loc[:,[1,2]] ###TODO: only two lanes
        cnn_time = rospy.get_rostime().to_sec()
        
        cnn_out_msg = CNNLaneDetectionMsg()
        if including_image:
            cnn_out_msg.front_camera_image = cvbridge.cv2_to_imgmsg(image)
        cnn_out_msg.start_time = start_time
        cnn_out_msg.cnn_time = cnn_time
        cnn_out_msg.phi_imu = phi_imu
        cnn_out_msg.delta_distance = delta_distance
        cnn_out_msg.delta_heading = delta_heading
        cnn_out_msg.out_cnn = out_j.reshape(-1)
        cnn_out_msg.out_cnn_height = out_j.shape[0]
        cnn_out_msg.out_cnn_width = out_j.shape[1]
        
        cnn_out_pub.publish(cnn_out_msg)
        
        # updater.update(out_j, delta_distance, delta_heading, phi_imu)  #need change
        if odom_info is not None:
            heading_rate_old = heading_rate_new
            velocity_old = velocity_new
            timestamp_old = timestamp_new
    
    
def nav_callback(nav_):
    global odom_info
    odom_info = nav_


if __name__ == "__main__":
    torch.backends.cudnn.benchmark = True

    is_compressed_image = True
    including_image = True

    args, cfg = merge_config(config_path=os.path.join(cur_file_dir,'configs/tusimple.py'))
    cfg.test_model = os.path.join(cur_file_dir,'tusimple_18.pth')
    cfg.dataset = 'RingroadTusimple'

    assert cfg.backbone in ['18','34','50','101','152','50next','101next','50wide','101wide']

    if cfg.dataset == 'CULane':
        cls_num_per_lane = 18
    elif cfg.dataset == 'Tusimple':
        cls_num_per_lane = 56
    elif cfg.dataset == 'RingroadTusimple':
        cls_num_per_lane = 56
    elif cfg.dataset == 'RingroadCULane':
        cls_num_per_lane = 18
    else:
        raise NotImplementedError

    net = parsingNet(pretrained = False, backbone=cfg.backbone,cls_dim = (cfg.griding_num+1,cls_num_per_lane,4),
                    use_aux=False).cuda() # we dont need auxiliary segmentation in testing

    state_dict = torch.load(cfg.test_model, map_location='cpu')['model']
    compatible_state_dict = {}
    for k, v in state_dict.items():
        if 'module.' in k:
            compatible_state_dict[k[7:]] = v
        else:
            compatible_state_dict[k] = v

    net.load_state_dict(compatible_state_dict, strict=False)
    net.eval()
    print('Neural Network loaded!')
    
    #### Start Ros Communication:
    cvbridge = CvBridge()
    rospy.init_node("Camera_Lane_Detection_Node")
    print("initial detection node")

    img_transforms = transforms.Compose([
        transforms.ToPILImage(),
        # transforms.Resize((288, 800)),
        transforms.ToTensor(),
        transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
    ])

    odom_info = None
    heading_rate_old = None
    velocity_old = None
    timestamp_old = None
    
    nav_sub = rospy.Subscriber("/odom", Odometry, nav_callback, queue_size=1)
    if is_compressed_image:
        img_sub = rospy.Subscriber("/pylon_camera_node_center/image_rect/compressed", CompressedImage, img_callback, queue_size=1, buff_size=2**24)
    else:
        img_sub = rospy.Subscriber("/pylon_camera_node_center/image_rect", Image, img_callback, queue_size=1, buff_size=2**24)
    cnn_out_pub = rospy.Publisher("/lane_detection_result/CNN", CNNLaneDetectionMsg, queue_size=1)
    rospy.spin()


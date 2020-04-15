#!/usr/bin/python

import os
import rospy
import rospkg
import tf2_ros
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest, StartTrajectoryResponse
from cartographer_ros_msgs.srv import GetTrajectoryStates, GetTrajectoryStatesRequest, GetTrajectoryStatesResponse
from cartographer_ros_msgs.srv import FinishTrajectory, FinishTrajectoryRequest, FinishTrajectoryResponse
from cartographer_ros_msgs.msg import TrajectoryStates
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose

class CartoInterface(object):
    def __init__(self):
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_cb)
        self.srv_start_traj = rospy.ServiceProxy('/start_trajectory', StartTrajectory)
        self.srv_finish_traj = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)
        self.srv_get_traj_status = rospy.ServiceProxy('/get_trajectory_states', GetTrajectoryStates)

        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.init_pose = Pose()
        self.req = StartTrajectoryRequest()
        rospack = rospkg.RosPack()
        carto_path = rospack.get_path('cartographer_ros')
        self.req.configuration_directory = carto_path + "/configuration_files"
        self.req.configuration_basename = "backpack_3d.lua"
        self.req.use_initial_pose = True

        self.current_traj_id = 0
        self.found_live_traj = False
    
    def get_z_pose(self):
        pose_found = False
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not pose_found:
            try:
                trans = self.tf2_buffer.lookup_transform('map', 'base_link', rospy.Time.now())
                pose_found = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
        z_pose = trans.transform.translation.z
        return z_pose
        
    def pose_cb(self,msg):
        print("pose cb")
        self.init_pose = msg.pose.pose        
        self.get_traj_state_client()
        if self.found_live_traj :
            self.init_pose.position.z = self.get_z_pose()
            self.finish_traj()
        self.start_traj_client()
    
    def start_traj_client(self):
        print("start traj client")
        rospy.wait_for_service('/start_trajectory')
        self.req.initial_pose = self.init_pose
        self.req.relative_to_trajectory_id = 0 #self.current_traj_id
        try:
            ret = self.srv_start_traj(self.req)
        except rospy.ServiceException as e:
            print(e)

    def get_traj_state_client(self):
        print("get traj")
        rospy.wait_for_service('/get_trajectory_states')
        try:
            ret = self.srv_get_traj_status()
        except rospy.ServiceException as e:
            print(e)
        traj_ids = ret.trajectory_states.trajectory_id
        traj_states = ret.trajectory_states.trajectory_state
        index = 0
        for idx in traj_states:
            val = map(ord,idx)
            if not val[0]:
                print("found : ", index)
                self.found_live_traj = True
                break
            index += 1
        if self.found_live_traj: 
            self.current_traj_id = int(traj_ids[index])

    def finish_traj(self):
        print("finish traj :", self.current_traj_id)
        rospy.wait_for_service('/finish_trajectory')
        try:
            ret = self.srv_finish_traj(self.current_traj_id)
        except rospy.ServiceException as e:
            print(e)

def main():
    rospy.init_node('initialize_trajectory')
    #print("path : ",os.path.abspath(os.getcwd()))
    ci = CartoInterface()
    rospy.spin()

if __name__ == '__main__':
    main()

#!/usr/bin/env python
import rospy
import os, sys, inspect
import signal
import numpy as np
from crazy_ros.srv import UpdateParams
from crazy_ros.msg import GenericLogData
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from time import sleep, time
import yaml
from math import sin, cos, pi
import tf
import scipy as sp
import itertools
import json
import ast

# Loads the crazyflie library module from the /modules directory
curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
libdir = os.path.join(os.path.dirname(curdir),'modules')
sys.path.insert(1,libdir)
from crazylib import Trajectory

class sparseMatrix3D(object):

    def __init__(self, nx=500, ny=500, nz=500):
        self.nx = nx
        self.ny = ny
        self.nz = nz
        self.weight = []
        for ii in range(nx):
            self.weight.append(np.zeros((ny,nz)))

    def add_weight(self, ix, iy, iz):
        self.weight[ix][iy,iz] += 1
    
    def tranform_to_pl2(self, roomContstraints, frame_id):
        iterator = []
        delta = np.array([0.0,0.0,0.0])
        bounds = [self.nx, self.ny, self.nz]
        for ii in range(3):
            delta[ii] = (roomContstraints['max'][ii] - roomContstraints['min'][ii])/float(bounds[ii])
        for ix in range(self.nx):
            rows,cols = self.weight[ix].nonzero()
            for iy,iz in zip(rows,cols):
                iterator.append([ix * delta[0] + roomContstraints['min'][ii],
                                 iy * delta[1] + roomContstraints['min'][ii],
                                 iz * delta[2] + roomContstraints['min'][ii]])
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return pc2.create_cloud_xyz32(header, iterator)
        
class Visualization(object):

    def __init__(self):
        # Linear path from the TSP GA / PA
        self.linearPoints = None                                  # Array with points with position/priority/time (from user)
        self.linearPath = None                                    # Array with points with position/priority/time (from GA)
        
        # QP generated path and 
        self.path = Path()                                        # Actual trajectory (Path object)
        self.reference_path = Path()                              # Reference trajectory (Path object)
        self.frame_id = "/world"                                  # ID used in rviz (String)

        # Room visualization and data compression
        self.quadRotation = None                                  # most recently measured quadcopter rotation (np.array 3x3)
        self.quadOffset = None                                    # most recent camera offset (np.array 3x1)
        self.quadPCL = None                                       # most recent camera pcl    (PointCloud2 object)
        self.quadPCLStatus = False                                # True if the PCL is retreived from the kinect (Boolean)
        self.roomResolution = np.array([500,400,150])             # Room resolution
        self.roomContstraints = {'max':[10,6,4],             # Room maximum and minimum dimensions,
                                 'min':[-1.0,-1.0,-1.0]}             # all data points outside will be ignored
        self.roomData = sparseMatrix3D(self.roomResolution[0],    # sparse LIL-object for storing data
                                       self.roomResolution[1],
                                       self.roomResolution[2])
        # Subscribers
        self.traj_file     = rospy.Subscriber('/visualization/trajectory_file', String, self.visualize_trajectory)
        self.pcl2_load     = rospy.Subscriber('/visualization/pointcloud_load', String, self.pointcloud_load)
        self.pcl2_save     = rospy.Subscriber('/visualization/pointcloud_save', String, self.pointcloud_save)
        self.pcl2_active   = rospy.Subscriber('/visualization/pointcloud_active', String, self.pointcloud_active)
        self.meas_pose_sub = rospy.Subscriber('/crazyflie/log_pos', GenericLogData, self.handle_measured_pose)
        self.ref_pose_sub  = rospy.Subscriber('/crazyflie/log_ref', GenericLogData, self.handle_reference_pose)
        self.camera_pcl_sub= rospy.Subscriber('/camera/depth/points', PointCloud2, self.handle_camera_pcl)

        # Publishers
        self.ref_traj_pub    = rospy.Publisher('/visualization/reference_trajectory', Path, queue_size = 10)
        self.ref_pose_pub    = rospy.Publisher('/visualization/reference_pose', PoseStamped, queue_size = 10)
        self.ref_mark_pub    = rospy.Publisher('/visualization/reference_position', Marker, queue_size = 10)
        self.ref_markarr_pub = rospy.Publisher('/visualization/reference_markerarray', MarkerArray, queue_size = 10, latch=True)
        self.meas_traj_pub   = rospy.Publisher('/visualization/measured_trajectory', Path, queue_size = 10)
        self.meas_pose_pub   = rospy.Publisher('/visualization/measured_pose', PoseStamped, queue_size = 10)
        self.meas_mark_pub   = rospy.Publisher('/visualization/measured_position', Marker, queue_size = 10)
        self.anchor_pub      = rospy.Publisher('/visualization/anchors', MarkerArray, queue_size = 10, latch=True)
        self.room_pcl        = rospy.Publisher('/visualization/pcl', PointCloud2, queue_size = 1)
        self.room_pcltemp     = rospy.Publisher('/visualization/pcltemp', PointCloud2, queue_size = 1)
         
        # Loads a trajectory object with a dummy trajectory file
        # TODO: remove the need for setting a trajectory file here as it is never used
        curdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        self.trajdir = os.path.join(os.path.dirname(curdir),'trajectories')
        self.mapdir = os.path.join(os.path.dirname(curdir),'maps')
        self.trajectory = Trajectory('Trajectory2.mat', self.trajdir, 1, 1)
        
        # Visualizes anchors
        self.visualize_anchors()

    ######################### Anchor visualization ############################
    
    def visualize_anchors(self):
        # Read values from yaml file
        for arg in rospy.myargv(argv=sys.argv):
            if os.path.basename(arg) == 'anchor_pos.yaml':
                try:
                    with open(arg, 'r') as stream:
                        anchors = yaml.load(stream)    
                except yaml.YAMLError:
                    raise Exception('Could not load data in "anchor_pos.yaml",\n'+
                                    'please check for syntax errors in the file.')
                except IOError:
                    raise IOError('Could not locate "anchor_pos.yaml" in /config,\n'+
                                  'please verify that the file exists')
        
        markerArray = MarkerArray()
        for key in anchors.keys():
            if not key == 'n_anchors':
                try:
                    m = self._create_marker(anchors[key], key, int(key[6]))
                    m.color.r, m.color.g, m.color.b, m.color.a = 0, 0, 0, 0.8
                    markerArray.markers.append(m)
                except:
                    raise Exception('Could not create all of the markers for anchor visualization.\n'+
                                    'check that the anchors are correctly defined in anchor_pos.yaml')
        self.anchor_pub.publish(markerArray)

    ######################## Linear path visualization #########################

    def visualize_trajectory(self, msg):
        
        # Loads and visualizes trajectory object
        filename = msg.data
        try:
            self.trajectory.load(filename)
        except Exception as inst:
            print inst.args[0] + '\nTrajectory loading failed!\n'
            return
        
        # Here we distinguish between two cases, the first where a set of points
        # are defined in R3 constituting a linear trajectory, in which case the
        # control points are plotted. If a preferred path vector exists, this
        # is visualized as well. In the second case, we simply have a QP
        # trajectory without control points, which is the visualized as a single
        # path object. With this structure, we may only have one of two types of
        # trajectories and we can never have dual trajectories. In addition, we
        # may only transfer trajectories which ar loaded and hence visualized in
        # RVIZ.
        trajectory_path = Path()
        trajectory_markers = MarkerArray()
        if self.trajectory.isLinear:
            points = self.trajectory.Pmat
            pvec = self.trajectory.pathvector
            for ii in range(points.shape[0]):
                # Updates the job positions in the linear path 
                unique_marker_id = 50+ii
                position = points[pvec[ii],:]
                m = self._create_marker(position, 'measurement', unique_marker_id)
                m.color.r, m.color.g, m.color.b, m.color.a = 0, 0.8, 0, 0.8
                trajectory_markers.markers.append(m)
                
                # Updates the path object itself 
                pose = self._create_pose( p=points[pvec[ii],:], q=[0,0,0,1] )
                self._update_path(trajectory_path, pose)
            self.ref_markarr_pub.publish( trajectory_markers )
            
        elif self.trajectory.isQuadratic:
            M = 50
            time_total = max( self.trajectory.times ) - min( self.trajectory.times )
            ts = min( self.trajectory.times )
            dt = float(time_total) / ( M - 1 )
            for ii in range(M):
                values = self.trajectory.evaluate_trajectory( ts + ii * dt )
                pose = self._create_pose( p=values[0:3], q=[0,0,0,1] )
                self._update_path(trajectory_path, pose)
        self.ref_traj_pub.publish( trajectory_path )
        
    ################## Reference QP trajectory visualization ###################
    
    def visualize_reference_trajectory(self, msg):
        # Tries to load the file and throws an error if the specified
        # file is formatted incorrecly by passing the thrown exception

        filename = msg.data
        try:
            self.trajectory.load(filename)
        except Exception as inst:
            print inst.args[0] + '\nTrajectory loading failed!\n'
            return
        
        # Evaluates trajectory in M points and updates the reference trajectory path
        trajectory_path = Path()
        M = 50
        time_total = max( self.trajectory.times ) - min( self.trajectory.times )
        ts = min( self.trajectory.times )
        dt = float(time_total) / ( M - 1 )
        for ii in range(M):
            values = self.trajectory.evaluate_trajectory( ts + ii * dt )
            pose = self._create_pose( p=values[0:3], q=[0,0,0,1] )
            self._update_path(trajectory_path, pose)
        self.ref_traj_pub.publish( trajectory_path )

    #################### Kinect point cloud visualization #####################
    
    def pointcloud_active(self, msg):
        # Callback for activating/disabeling the pointcloud functionality 
        if msg.data == 'True':
            self.quadPCLStatus = True
        else:
            self.quadPCLStatus = False

    def pointcloud_save(self, msg):
        # TODO: Callback for saving a PCL structure
        print "Saving sparse matrix data...\r"
        # Converts the room data object to a spasrse matrix and then saves 
        sparseData = []
        for ii in range(self.roomData.nx):
            sparseData.append(sp.sparse.lil_matrix(self.roomData.weight[ii]))
        sparseMat = np.array(sparseData)
        
        data = {"nx": self.roomData.nx, 
                "ny": self.roomData.ny,
                "nz": self.roomData.nz,
                "max": self.roomContstraints['max'],
                "min": self.roomContstraints['min'],
                'weight':[str(element).replace(" ","") for element in sparseMat]}

        #filename = msg.data
        filename = msg.data
        path = os.path.join(self.mapdir, filename)
        with open(path, 'w') as mapfile:
            json.dump(data, mapfile, separators=(',', ':'),sort_keys=True, indent=4)
        print " Complete!"

    def pointcloud_load(self, msg):
        # TODO: Callback for loading a PCL structure
        print "Loading sparse matrix data...\r"
        filename = msg.data
        recreated_matrix  = []
        path = os.path.join(self.mapdir, filename)
        try:
            with open(path, 'r') as mapfile:
                data = json.load(mapfile)
            mapfile.close()
        except:
            print (("Error in visualization node, could not locate %s in the\n"+
                    "directory %s") % (msg.data, self.mapdir))
            return
        count = 0
        room = sparseMatrix3D(nx=data['nx'], ny=data['ny'], nz=data['nz'])
        # Recreate weight matrix
        for ii in range(data['nx']):
            temp_mat = np.zeros((data['ny'],data['nz']))
            for pixel in data['weight'][ii].split('\n'):
                line = pixel.split('\t')
                try:
                    index = ast.literal_eval(line[0])
                    temp_mat[index] = int(float(line[1]))
                    count +=1
                except:
                    pass
            room.weight[ii] = temp_mat
        pcl = room.tranform_to_pl2(self.roomContstraints, self.frame_id)
        self.room_pcl.publish(pcl)
        print count
        print 'Complete"'

    def handle_camera_pcl(self, msg):
        # Updates the most recent pcl measurement
        self.quadPCL = msg
    
    def update_pcl_data(self):
        # Only update PCL structure if there exist new estimate of the rotation, offset or PCL
        if (not self.quadRotation == None and
            not self.quadOffset == None and
            not self.quadPCL == None and 
            self.quadPCLStatus == True):

            quadpcl = self.quadPCL
            # Includes new measurements in the PCL with outlier removal
            factor = np.array([0.0,0.0,0.0])
            for ii in range(3):
                factor[ii] = self.roomResolution[ii] / (self.roomContstraints['max'][ii] - self.roomContstraints['min'][ii])

            # Rotates image into the correct coordinate system
            R1 = np.array([[1.0,0.0,0.0],[0.0,-1.0,0.0],[0.0,0.0,-1.0]]) # Transforms the image to reside in the yz-plane
            R2 = np.array([[1.0,0.0,0.0],[0.0,0.0,-1.0],[0.0,1.0,0.0]])  # Rotates the image
            R3 = np.array([[0.0,1.0,0.0],[-1.0,0.0,0.0],[0.0,0.0,1.0]])  # Rotates the image
            
            automatic = False
            try:
                if automatic:
                    cameraRotation = np.dot(self.quadRotation, np.dot(R3,np.dot(R2,R1)))
                    cameraOffset = self.quadOffset
                else:
                    dataPos = raw_input('Enter x y z offset [m]: ')
                    cameraOffset = np.array([float(num) for num in dataPos.split()])
                    
                    dataAng = raw_input('Enter r p y offset [deg]: ')
                    rpy = [float(ang) for ang in dataAng.split()]
                    R = self._get_RPY_2_rotmat(rpy)
                    cameraRotation = np.dot(R, np.dot(R3,np.dot(R2,R1)))
            except:
               print 'whoops'
               return

            # Create temporary point cloud
            cont = True
            while cont:
                iterator = []
                for p in pc2.read_points(quadpcl, field_names = ("x", "y", "z"), skip_nans=True):
                    point = np.dot(cameraRotation, np.array(p)) + cameraOffset
                    iterator.append(point.tolist())
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = self.frame_id
                pointCloud = pc2.create_cloud_xyz32(header, iterator)
                self.room_pcltemp.publish(pointCloud)
                try:
                    dataAdjust = raw_input('Enter x y z correction [m]: ')
                    if len(dataAdjust.split())== 3:
                        print 'change'
                        cameraOffset = cameraOffset + np.array([float(num) for num in dataAdjust.split()])
                    elif dataAdjust.split()[0] == 'c':
                        print 'continue'
                        cont = False
                    else:
                        print 'return'
                        return
                except:
                   print 'whoops'
                   return
                
                
            for p in pc2.read_points(quadpcl, field_names = ("x", "y", "z"), skip_nans=True):
                # Transforms point to the global coordinate system
                point = np.dot(cameraRotation, np.array(p)) + cameraOffset
                ix = int(round((point[0] - self.roomContstraints['min'][0]) * factor[0]))
                iy = int(round((point[1] - self.roomContstraints['min'][1]) * factor[1]))
                iz = int(round((point[2] - self.roomContstraints['min'][2]) * factor[2]))
                if (ix > 0 and iy > 0 and iz > 0 and ix < self.roomResolution[0] and
                    iy < self.roomResolution[1] and iz < self.roomResolution[2]):
                    
                    self.roomData.add_weight(ix, iy, iz)     
            pointCloud = self.roomData.tranform_to_pl2(self.roomContstraints, self.frame_id)
            self.pointcloud_save('room2211b.txt')
            self.room_pcl.publish(pointCloud)

            # Supplements the weight matrix data
            debug = False
            if debug:
                iterator = []
                for p in pc2.read_points(self.quadPCL, field_names = ("x", "y", "z"), skip_nans=True):
                    point = np.dot(cameraRotation, np.array(p)) + cameraOffset
                    iterator.append(point.tolist())
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = self.frame_id
                pointCloud = pc2.create_cloud_xyz32(header, iterator)
            if False:
                for p in pc2.read_points(self.quadPCL, field_names = ("x", "y", "z"), skip_nans=True):
                    # Transforms point to the global coordinate system
                    point = np.dot(cameraRotation, np.array(p)) + cameraOffset
                    ix = int(round(point[0] * factor[0]))
                    iy = int(round(point[1] * factor[1]))
                    iz = int(round(point[2] * factor[2]))
                    if (ix > 0 and iy > 0 and iz > 0 and ix < self.roomResolution[0] and
                        iy < self.roomResolution[1] and iz < self.roomResolution[2]):
                        self.roomData.add_weight(ix, iy, iz)     
                pointCloud = self.roomData.tranform_to_pl2(self.roomContstraints, self.frame_id)
                self.pointcloud_save('room221b.txt')
            #self.room_pcl.publish(pointCloud)


    #################### Data callbacks and help functions ####################

    def handle_measured_pose(self, data):
        # This callback creates a parker and pose corresponding to the
        # most recent measurement as computed in the firmware, publishes
        # the point and orientation for visualization in ROS and updates
        # the corresponding PATH object showing the trajectory of the
        # M (int) most recent poses.
        
        # Parameter defining how many points are allowed in the path
        M = 100
        
        # Extracts data and converts angles to radians
        position = data.values[0:3]
        quaternion = self._get_RPY_2_quaternion(data.values[3:6])
        
        # Stores the most recent measured rotation matrix
        self.quadRotation = self._get_RPY_2_rotmat(data.values[3:6])
        self.quadOffset = np.array(position)

        # Computes the pose and marker
        pose = self._create_pose(position, quaternion)
        unique_marker_id = 98;
        marker = self._create_marker(position, 'measurement', unique_marker_id)
        marker.color.r, marker.color.g, marker.color.b = 1, 0, 0

        # Updates the path object
        self.path = self._update_path(self.path, pose)
        
        # Sets the maximum number of poses allowed to be stored in the path object
        # as a precaution. This method assumes that we only update the path vector
        # handle_measured_pose.
        if len(self.path.poses) > M:
            self.path.poses.pop(0)

        # Publishes the path object, pose and marker
        self.meas_pose_pub.publish(pose)
        self.meas_traj_pub.publish(self.path)
        self.meas_mark_pub.publish(marker)

    def handle_reference_pose(self, data):
        # Computes the pose from the generic log data of the reference
        # as ertreived from the crazyflie and plots the result in rviz
        # Extracts data and converts angles to radians
        position = data.values[0:3]
        quaternion = self._get_RPY_2_quaternion(data.values[3:6])

        # Computes the pose and marker
        pose = self._create_pose(position, quaternion)
        unique_marker_id = 99;
        marker = self._create_marker(position, 'measurement', unique_marker_id)
        marker.color.r, marker.color.g, marker.color.b = 0, 0, 1

        # Publishes the path object, pose and marker
        self.ref_pose_pub.publish(pose)
        self.ref_mark_pub.publish(marker)

    def _create_marker(self, p, n, num):
        # Creates a marker with unit quaternion for visualization
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = p[0], p[1], p[2]
        m.pose.orientation.x, m.pose.orientation.y = 0, 0
        m.pose.orientation.z, m.pose.orientation.w = 0, 1
        m.scale.x, m.scale.y, m.scale.z = 0.1, 0.1, 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = 0, 0, 1, 0.8
        m.id, m.type = num, 2
        m.lifetime = rospy.Duration() # never autodelete
        return m
    
    def _create_pose(self, p, q):
        # Creates a pose object from a positional vector and quaternion
        pose = PoseStamped()
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.Time.now()
        return pose

    def _update_path(self, path, pose):
        # Update a desired path with a time stamped pose object
        pose.header.seq = path.header.seq + 1
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        pose.header.stamp = path.header.stamp
        path.poses.append(pose)
        return path

    def _get_RPY_2_quaternion(self, values):
        # This function takes angles on a roll pitch yaw (RPY) format
        # in degrees and computes the ros compliant quaternion representing
        # the same rotation

        # Convert to radians and divide by 2 for qtarenion transform
        rd2 = ( values[0] * pi / 180 ) / 2
        pd2 = ( values[1] * pi / 180 ) / 2
        yd2 = ( values[2] * pi / 180 ) / 2
        
        # Compute quaternion (note the order in which it is returned)
        qw = cos(rd2)*cos(pd2)*cos(yd2) + sin(rd2)*sin(pd2)*sin(yd2)
        qx = sin(rd2)*cos(pd2)*cos(yd2) - cos(rd2)*sin(pd2)*sin(yd2)
        qy = cos(rd2)*sin(pd2)*cos(yd2) + sin(rd2)*cos(pd2)*sin(yd2)
        qz = cos(rd2)*cos(pd2)*sin(yd2) - sin(rd2)*sin(pd2)*cos(yd2)
        return [qx, qy, qz, qw]

    def _get_RPY_2_rotmat(self, values):
        # This function takes angles on a roll pitch yaw (RPY) format
        # in degrees and computes the ros compliant quaternion representing
        # the same rotation
        r, p, y = (values[0] * pi / 180.0), (values[1] * pi / 180.0), (values[2] * pi / 180.0)
        c3, c2, c1 = cos(r), cos(p), cos(y)
        s3, s2, s1 = sin(r), sin(p), sin(y)
        R = np.array([[c1*c2, c1*s2*s3-s1*c3, c1*s2*c3+s1*s3],
                      [s1*c2, s1*s2*s3+c1*c3, s1*s2*c3-c1*s3],
                      [  -s2,          c2*s3,          c2*c3]])
        return R

def signal_handler(signal, frame):
    print 'Shutting down master node nicely!'
    sys.exit(0)

def main():
    rospy.init_node('visualizationNode')
    visualization = Visualization()
    signal.signal(signal.SIGINT, signal_handler)
    
    # TODO: remove, now used to make the code run en no estimation is available
    visualization.quadRotation = np.array([[1,0,0],[0,1,0],[0,0,1]])
    visualization.quadOffset = np.array([0,0,0])
    while True:
        # Only visualize and use PCL if the mapping mode is active
        visualization.update_pcl_data()
        sleep(1)
        
    
if __name__ == '__main__':
    main()

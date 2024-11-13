

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, SetBoolResponse

class HuskyHighlevelController:
    def __init__(self):
        # 初始化NodeHandle
        #self.node_handle = rospy.get_param("~")
        
        # 从参数服务器获取控制增益
        #self.ctrl_p = rospy.get_param("~controller_gain", 0.0)
        
        # 获取订阅话题和队列大小
        #topic = rospy.get_param("~subscriber_topic", "")
        #queue_size = rospy.get_param("~queue_size", 10)
        '''
        if not topic:
            rospy.logerr("Could not find subscriber params!")
            rospy.signal_shutdown("Missing subscriber parameters")
        '''
        
        # 创建订阅者
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # 创建发布者
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.viz_pub = rospy.Publisher("visualization_marker", Marker, queue_size=0)
        self.pub = rospy.Publisher("/revised_scan", LaserScan, queue_size = 10)
        
        # 服务服务器
        self.stop_srv = rospy.Service("/start_stop", SetBool, self.start_stop)
        
        # 初始化Twist消息
        self.msg = Twist()
        
        self.flag = 0
        
        # 初始化pillar位置
        self.pillar_pos = [0.0, 0.0]
        
        # 初始化marker
        self.marker = Marker()
        self.init_pillar_marker()
        
        # 设置初始速度
        self.set_vel(0, "forward")
        
        rospy.loginfo("Husky highlevel controller node launched!")
    
    def set_vel(self, vel, dof):
        """设置Husky的速度"""
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel
    
    def drive_husky(self, dist):
        """发布速度命令"""
        self.set_vel( dist, 'forward')
        self.vel_pub.publish(self.msg)
        
    
    def adjust_heading(self, ang):
        """通过P控制调整机器人朝向"""
        ''' 以0度面向方向为前方，度数小于0向右转，大于0向左转'''
        #print("here")
        if ang<math.pi:
        	self.set_vel(ang,"ang")
        if ang>=math.pi:
        	diff=ang-360
        	self.set_vel(diff,"ang")
        #diff = -ang
        #self.set_vel( diff, "ang")
        self.vel_pub.publish(self.msg)
        print(self.msg)
    
    def viz_pillar(self):
        """在RViz中可视化柱子"""
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.viz_pub.publish(self.marker)
    
    def laser_callback(self, msg):
        """激光扫描数据的回调函数，定位柱子并调整朝向"""
        #print(len(msg.ranges)) len is 2019 from 0-360
        count = 0
        #flag = 0
        for i in range(15):
        	if msg.ranges[i] <= 0.15:
        		count = count + 1
        	if msg.ranges[359 - i] <= 0.15:
        		count = count + 1
        #print(count)
        if count > 10:
        	self.set_vel(0.0, "forward")
        	self.vel_pub.publish(self.msg)
        	#self.flag = 1
        	print("first")
        
        print(f"flag: {self.flag}")
        if self.flag == 0:
        	max_diff_index=self.find_max_difference_positions(msg.ranges)
        
        	if max_diff_index[0][0]<max_diff_index[1][0]:
            		index=[max_diff_index[0][0],max_diff_index[0][1],max_diff_index[1][0],max_diff_index[1][1]]
        	else:
            		index=[max_diff_index[1][0],max_diff_index[1][1],max_diff_index[0][0],max_diff_index[0][1]]

        	if index[2]-index[1]<180:
            		dest_index=int((index[1]+index[2])/2)
        	else:
            		dest_index=int((180+(index[1]+index[2])/2)%359)
        #dest_index=self.find_difference_positions(msg.ranges)   #找左右有突变的方法
        	#if max_diff_index[0][0]<max_diff_index[0][1]:
        	#	dest_index=max_diff_index[0][0]
        	#else:
        	#	dest_index=max_diff_index[0][1]
        	ang = msg.angle_min + msg.angle_increment *dest_index
        	dist = self.find_min_non_zero([msg.ranges[dest_index],msg.ranges[index[0]],msg.ranges[index[1]],msg.ranges[index[2]],msg.ranges[index[3]]])
        	#dist=min(msg.ranges[dest_index],msg.ranges[index[0]],msg.ranges[index[1]],msg.ranges[index[2]],msg.ranges[index[3]])
        	#dist=msg.ranges[dest_index]   
        	rospy.loginfo(f"dest_index is {dest_index}, max_diff_index is {index[0]},{index[1]},{index[2]},{index[3]}")
        	rospy.loginfo(f"Pillar is {dist}m away at {ang * 180 / math.pi} degrees")
        	self.pillar_pos[0] = dist * math.cos(ang)
        	self.pillar_pos[1] = dist * math.sin(ang)
        	rospy.loginfo(f"Pillar's coordinate to Husky is [{self.pillar_pos[0]}, {self.pillar_pos[1]}]") 
        	# 调整朝向并驱动Husky
        	if 8 <= (ang*180/ math.pi) and (ang*180/math.pi) <= 352 and dist<4 and dist>0.2:
        	#print("herehere")
        		self.adjust_heading(ang)
        	elif (350 <= (ang*180/ math.pi) or (ang*180/math.pi) <= 10) :
        		if dist>0.4:
        		#self.adjust_heading(0)
        			self.drive_husky(dist)
        		elif dist<=0.4 and dist>0.15 :
        			self.drive_husky(0.5*dist)
        	if dist< 0.2:
        		self.flag = 1
        		print("second")
        		self.drive_husky(0)
        		self.adjust_heading(0)
			
        
        
		# 可视化柱子
		#self.viz_pillar()
	    
    def start_stop(self, request):
        """启动/停止机器人"""
        if request.data:  # 启动
            response_msg = "Start Husky"
            self.set_vel(3.0, "forward")
        else:  # 停止
            response_msg = "Stop Husky"
            self.set_vel(0.0, "forward")
        
        rospy.loginfo(response_msg)
        return SetBoolResponse(success=True, message=response_msg)
    
    def init_pillar_marker(self):
        """初始化RViz中的柱子marker"""
        self.marker.header.frame_id = "base_laser"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "pillar"
        self.marker.id = 1
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 2
        self.marker.color.a = 1.0  # 设置透明度
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        
    def find_difference_positions(self,arr):
    	for i in range(len(arr)-1):
    		left=False
    		right=False
    		for k in range(6):
    			if abs(arr[i-k]-arr[i-k-1])>0.3 and arr[i-k]!=0 and arr[i-k-1]!=0:
    				left=True
    			if abs(arr[i+k]-arr[i+k+1])>0.3 and arr[i+k]!=0 and arr[i+k+1]!=0:
    				right=True
    			if left and right and arr[i]!=0:
    				return i
    	return 0
        
    def find_max_difference_positions(self, arr):
    	# 检查数组长度是否大于等于2
    	if len(arr) < 2:
        	raise ValueError("数组长度必须大于等于2")

    # 初始化两个最大差值和对应的位置对
    	max_diff = abs(arr[1] - arr[0])  # 第一大差值
    	second_max_diff = 0  # 第二大差值
    	max_diff_positions = (0, 1)  # 第一大差值的位置对
    	second_max_diff_positions = (0, 1)  # 第二大差值的位置对

    # 遍历数组，寻找最大和第二大的差值
    	for i in range(len(arr) - 1):
        	diff = abs(arr[i + 1] - arr[i])
        	if diff > max_diff and arr[i]!=0 and arr[i+1]!=0:
            # 更新第一大差值和第二大差值
            		second_max_diff = max_diff
            		second_max_diff_positions = max_diff_positions
            		max_diff = diff
            		max_diff_positions = (i, i + 1)
        	elif diff > second_max_diff and arr[i]!=0 and arr[i+1]!=0:
            # 更新第二大差值
            		second_max_diff = diff
            		second_max_diff_positions = (i, i + 1)

    # 处理最后一个元素和第一个元素的差异
    	diff = abs(arr[-1] - arr[0])
    	if diff > max_diff and arr[-1]!=0 and arr[0]!=0:
        	second_max_diff = max_diff
        	second_max_diff_positions = max_diff_positions
        	max_diff = diff
        	max_diff_positions = (len(arr) - 1, 0)
    	elif diff > second_max_diff and arr[-1]!=0 and arr[0]!=0:
        	second_max_diff = diff
        	second_max_diff_positions = (len(arr) - 1, 0)

    	return max_diff_positions, second_max_diff_positions
    	
    def find_min_non_zero(self, numbers):
    	# 去掉列表中的零
    	non_zero_numbers = [num for num in numbers if num != 0]
    	# 如果去掉零之后的列表为空，说明没有非零数
    	if not non_zero_numbers:
    		return None # 或者可以抛出一个异常，取决于你的需求
    	# 返回非零数中的最小值
    	return min(non_zero_numbers)



if __name__ == '__main__':
    rospy.init_node('husky_highlevel_controller', anonymous=True)
    controller = HuskyHighlevelController()
    rospy.spin()

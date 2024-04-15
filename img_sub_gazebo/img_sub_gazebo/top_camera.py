import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import rclpy.publisher
from sensor_msgs.msg import Image
# from turtlebot3_msgs.msg import Astar
# from turtlebot3_msgs.srv import Astarservice
import cv2 as cv
import numpy as np
from heapq import *
import math
import time
from rclpy.qos import QoSProfile


class top_camera(Node):

    def __init__(self):
        super().__init__('astar_path_planning')
        self.current_frame = 0
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',   #this is where you subscribe to a topic, both pub & sub have same topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        #self.motionpublisher = self.create_publisher(Astar,'motioncontrol',10)
        self.service = self.create_service(Astarservice,'astar',self.service_callback)
        #self.motionsubscription = self.create_subscription(Astar,'motioncontrol',self.motionsubscription_callback,10)
    def service_callback(self,request,response):
        print(request)
        #response = 'Recevived request %d' % response
        
        if request.value == "request A*":
            print(1)
            response = astarservice(response)
            return response
        print(2)
        return "Invalid"

        
    def motionsubscription_callback(self,data):
        print(data)

    def listener_callback(self, data):
        self.current_frame = self.br.imgmsg_to_cv2(data)
        # cv.imshow('camera',self.current_frame)
        # cv.waitKey(1)
def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    obstacle_value = 255
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:
        
        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return tuple(data) + tuple([start])

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == obstacle_value:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False
#ros2 can't send np array due to data type wrappers, ie  np.array([1,2],np.int32), the numbers 1 & 2 has an int32 data wrapper
def angle360(path):
    xyarray = np.array(path)    #convert to array for maths operations
    angles = np.empty(len(path)-1)  #create empty array to store angles, the number of angles is 1 less than the path
    xylength = np.empty(xyarray.shape,np.float32) #create empty array to store angles
    standardized_rad = 1/math.pi #convert 0 ~ pi to 0 ~ 1
    ind =0
    magnitude = []
    angle = []
    coordinate=[]
    coordinate.append(path[-1])
    for i in range(0,len(xyarray)-1):
        xylength[i] = xyarray[-i-2] - xyarray[-i-1] #find the length of x and y
        angles[i] = math.atan2(-xylength[i,0],xylength[i,1])*standardized_rad  #find the angle of x,y relative to positive x-axis (+y returns -angle and vice versa)
        
        if angles[i] != angles[i-1] and i>0:    #if current angle not same as previous angle save previous angle in a list
            magnitude.append(i -ind)
            angle.append(float(angles[i-1]))
            coordinate.append(path[-i-1])
            ind = i
    
    angle.append(float(angles[-1]))
    coordinate = np.reshape(coordinate,-1).tolist() #reshape it to a 1D because I am too lazy to figure how to send a 2D array via ros2 and remove data wrapper
    return angle, magnitude, coordinate

def astarservice(response):
    #rclpy.init(args=args)
    camera = top_camera()
    start = (12,25)
    goal = (3,3)
    x = 0
    #camera.create_service()
    while 1:
        rclpy.spin_once(camera)

        
        maze_gray = cv.cvtColor(camera.current_frame,cv.COLOR_BGR2GRAY)
        ret, maze_bw = cv.threshold(maze_gray,127,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
        indices = maze_bw.nonzero()
        min = np.min(indices,axis=1)
        max = np.max(indices,axis=1)
        maze_bw_slice = maze_bw[min[0]:max[0],min[1]:max[1]]
        maze_30 = cv.resize(maze_bw_slice,(30,30),interpolation=cv.INTER_NEAREST_EXACT)
        
        if x == 0:
            start_time = time.time()
            path = astar(maze_30,start,goal)
            angle,magnitude,coordinate = angle360(path)
            stop_time = time.time()
            fps = str(int(1/(stop_time - start_time)))
            print(fps,(stop_time - start_time))
            x= 1
        
        
        response.astar.angle = angle
        response.astar.magnitude = magnitude

        response.astar.coordinate = coordinate
        #camera.motionpublisher.publish(ast)
        

        


        return response


def main(args=None):
    rclpy.init(args=args)
    camera = top_camera()

    #minimal_subscriber = MinimalSubscriber()
    while 1:

        rclpy.spin_once(camera)
        cv.imshow('camera',camera.current_frame)
        cv.resizeWindow('camera',400,400)
        cv.waitKey(1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)



    camera.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
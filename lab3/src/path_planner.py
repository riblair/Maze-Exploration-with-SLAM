#!/usr/bin/env python3

import math
import rospy
from priority_queue import PriorityQueue
from typing import Tuple as tuple
from typing import List as list
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PathPlanner:
    
    def __init__(self):
        rospy.init_node("path_planner")

        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service('plan_path', GetPlan, self.plan_path)

        self.cspace = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=10)
        self.occgrid = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
        self.visited = rospy.Publisher("/visited", GridCells, queue_size=10)
        self.finalPath = rospy.Publisher("/FinalPath", GridCells, queue_size=10)

        ## Initialize the request counter
        # TODO

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        return (mapdata.info.width)*p[1]+p[0]



    @staticmethod
    def euclidean_distance(mapdata: OccupancyGrid, p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        return math.dist(p1,p2)
        


    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        oPose = mapdata.info.origin.position
        oOrien = mapdata.info.origin.orientation
        res = mapdata.info.resolution

        # this is the offset in the local frame.
        xOff = res*p[0]
        yOff = res*p[1]

        theta = math.atan2(yOff,xOff) # angle of pose in local frame
        (__, __, phi) = euler_from_quaternion([oOrien.x, oOrien.y, oOrien.z, oOrien.w]) # angle of local frame from world frame


        # world space transform
        point = Point()

        point.x = oPose.x + xOff + res/2
        point.y = oPose.y + yOff + res/2
        point.z = oPose.z

        return point


    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        oPose = mapdata.info.origin.position
        res = mapdata.info.resolution

        # Tracy's Solution

        # xOff = res*p[0]
        # yOff = res*p[1]
        # return wp / mapdata.info.resolution

        # Jais solution 
        # position = mapdata.info.origin.position
        # resolution = mapdata.info.resolution

        # height=mapdata.info.height
        # width=mapdata.info.width

        # if wp.x<width*resolution and wp.y<height*resolution:
        #     gridX=round(float(wp.x)/resolution)
        #     gridY=round(float(wp.y)/resolution)
        # grid=[gridX,gridY]
        
        # return grid

        xOff = wp.x-oPose.x 
        yOff = wp.y-oPose.y 

        angle = math.atan2(yOff,xOff)
        print("(x,y) = (" + str(round(xOff,3)) + "," + str(round(yOff,3)) + ") ang = " + str(round(angle,3)))

        xP = math.trunc((xOff) / res)
        yP = math.trunc((yOff) / res)
        
        print("(x,y) = (" + str(round(xOff,3)) + ", " + str(round(yOff,3)) + ") ang = " + 
              str(round(angle,3)) + " (x,y) = " + str(xP) + ", " + str(yP) + ")")
        return (xP, yP)

        
    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> list[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        wpath = []
        lastPos = PathPlanner.grid_to_world(mapdata, path[1])
        for i in range(len(path)):
            if (i+1 < len(path)):
                lastPos = PathPlanner.grid_to_world(mapdata, path[i+1])                
            pos = PoseStamped()
            pos.pose.position = PathPlanner.grid_to_world(mapdata, path[i])
            xDiff = lastPos.x - pos.pose.position.x 
            yDiff = lastPos.y - pos.pose.position.y 
            qArray = quaternion_from_euler(0, 0, math.atan2(yDiff,xDiff)) 
            pos.pose.orientation.x = qArray[0]
            pos.pose.orientation.y = qArray[1]
            pos.pose.orientation.z = qArray[2]
            pos.pose.orientation.w = qArray[3]
            wpath.append(pos)
        return wpath
    
    

    @staticmethod
    def is_cell_walkable(mapdata:OccupancyGrid, p: tuple[int, int]) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        # Jais solution
        # x = p[0]
        # y = p[1]
        # index = PathPlanner.grid_to_index(mapdata, p)
        # width = mapdata.info.width
        # height = mapdata.info.height
        # if x < width and x >= 0:
        #     if y < height and y >= 0:
        #         if mapdata.data[index] < 100:
        #             return True
        # else:
        #     return False
        
        index = PathPlanner.grid_to_index(mapdata,p)
        cellOcc = mapdata.data[index] #finds occ in p
        bounds = False
        free = False

        if p < (mapdata.info.width-1, mapdata.info.height-1):
            bounds = True

        if cellOcc < 75: # 75 is for expanded cells, 100 is for pre-existing walls
            free = True

        return bounds and free

               

    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        xOff = [0,0,-1,1]
        yOff = xOff[::-1]

        list = []

        for i in range (0,len(xOff)):
            cell = (p[0] + xOff[i],p[1] + yOff[i])
            if PathPlanner.is_cell_walkable(mapdata, cell) :
                list.append(cell)
        return list
    
    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        list = []
        for i in range(-1, 2) :
            for j in range(-1, 2):
                if i == j and i == 0 :
                    continue
                cell = (p[0]+i, p[1]+j)
                if PathPlanner.is_cell_walkable(mapdata, cell) :
                    list.append(cell)
        return list
    
    
    @staticmethod
    def request_map() -> OccupancyGrid:
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo("Requesting the map")
        oc = rospy.ServiceProxy('/static_map', GetMap)
        return oc().map

        

    def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """

        """ If desired, We could make a better 'Kernal' approach where instead of a square check, 
            we construct a kernal, and iterate through that kernal instead. 
            This way we could make the kernal an arbitrary shape (like circular)
        """

        # Kernal would be a list of tuples where each tuple is the x and y offset from center cell. 
        # Still "expensive" compared to state machine optimized approach ("only check new cells")

        rospy.loginfo("Calculating C-Space")

        # given an occupancy grid, make a new occupancy grid with filled spaces
        ## Go through each cell in the occupancy grid
        clist = [] #collects data of c-space
        occ = []
        alpha = 0.5
        
        for i in range(0, len(mapdata.data)): #iterates through each point
            fill = False
            x = int(i % (mapdata.info.width))
            y = int(i / (mapdata.info.height))
            if not PathPlanner.is_cell_walkable(mapdata, (x,y)):
                clist.append(100)
                occ.append(PathPlanner.grid_to_world(mapdata, (x,y)))
                continue

            ## Inflate the obstacles where necessary
            for j in range(-padding, padding + 1):
                for k in range (-padding, padding + 1): #looks through neighbors within padding
                    index = PathPlanner.grid_to_index(mapdata, [x-j,y-k])
                    if (index <0 or index >= len(mapdata.data)):
                        continue
                    if(mapdata.data[index] == 100):
                        fill = True
                        j = padding + 1
                        k = padding + 1

            if not fill:
                clist.append(0)
            else:
                clist.append(75)
                occ.append(PathPlanner.grid_to_world(mapdata, (x,y)))

        ## Create a OccupancyGrid message and publish it
        occgrid = OccupancyGrid()
        occgrid.header = mapdata.header
        occgrid.info = mapdata.info
        occgrid.data = clist
        self.occgrid.publish(occgrid)

        ## Create a GridCells message and publish it
        cspace = GridCells()
        cspace.header = mapdata.header
        cspace.cell_height = mapdata.info.resolution * alpha
        cspace.cell_width = mapdata.info.resolution * alpha
        cspace.cells = occ
        self.cspace.publish(cspace)
        
        ## Return the C-space
        return occgrid


    def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = dict()
        total_cost = dict()
        came_from[start] = None
        total_cost[start] = 0
        path=[]

        aStarGC=GridCells()
        aStarGC.header = mapdata.header
        aStarGC.cell_width = mapdata.info.resolution / 1.5
        aStarGC.cell_height = mapdata.info.resolution / 1.5
        aStarGC.cells = []
        # self.finalPath.publish(aStarGC) # sometimes causes next call to publish to just not work 

        aStarWaveFrontGC=GridCells()
        aStarWaveFrontGC.header = mapdata.header
        aStarWaveFrontGC.cell_width = mapdata.info.resolution / 2
        aStarWaveFrontGC.cell_height = mapdata.info.resolution / 2
        aStarWaveFrontGC.cells = []
        #while queue still contains positions
        while not frontier.empty():
            current = frontier.get()
            aStarWaveFrontGC.cells.append(PathPlanner.grid_to_world(mapdata,current))
            self.visited.publish(aStarWaveFrontGC)

            if current == goal:
                while current is not None:                    
                    aStarGC.cells.append(PathPlanner.grid_to_world(mapdata,current))
                    path.append(current)
                    current = came_from[current]

                for cell in aStarGC.cells :
                    while cell in aStarWaveFrontGC.cells :
                        aStarWaveFrontGC.cells.remove(cell)
                self.visited.publish(aStarWaveFrontGC) 
                self.finalPath.publish(aStarGC)  

                return path[::-1]

            neighbors = PathPlanner.neighbors_of_8(mapdata,current)
            for next in neighbors :
                # print("hueristic %f" % PathPlanner.euclidean_distance(mapdata, goal, next))
                new_cost = total_cost[current] + PathPlanner.euclidean_distance(mapdata, current, next)
                if next not in total_cost or new_cost < total_cost[next]:
                    
                    total_cost[next] = new_cost
                    frontier.put(next, new_cost + PathPlanner.euclidean_distance(mapdata, goal, next))
                    came_from[next] = current
        return path[::-1]


    
    @staticmethod
    def optimize_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """

        # find direction from previous node in path, 
        # if next node AND next next node have opposite direction, remove next node and continue.
        ### EXTRA CREDIT
        pathCopy = [path[0]]
        i = 1 # start at second node

        while i < (len(path)-1) :
            print(path[i])
            prevDirection = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            currDirection = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])

            if not prevDirection == currDirection :
                pathCopy.append(path[i])
            i = i+1
        # find direction from previous node in path, 
        # if next node and prev have same direction, remove next node and continue.
        ### EXTRA CREDIT
        pathCopy.append(path[len(path)-1])
        # rospy.loginfo("Optimizing path")
        return pathCopy

        

    def path_to_message(self, mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        
        rospy.loginfo("Returning a Path message")
        finalPath = Path()

        finalPath.header = mapdata.header
        finalPath.poses = PathPlanner.path_to_poses(mapdata,path)

        return finalPath


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)

        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        # return "a"

        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)

        ## Return a Path message
        print("akjsdajshd")
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    PathPlanner().run()

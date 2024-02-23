#!/usr/bin/env python3

import math
import rospy

from lastlab.srv import GetGridCells, GetPlan2
from priority_queue import PriorityQueue
from typing import Tuple as tuple
from typing import List as list
from nav_msgs.srv import GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

CSPACE_THRESH = 75 
CSPACE_PADDING = 5
RESOLUTION = 0.015

class PathPlanner:
    
    def __init__(self):
        rospy.init_node("path_planner")

        rospy.Service('plan_path', GetPlan2, self.plan_path)
        rospy.Service('get_Frontier', GetGridCells, self.findFrontier)
        self.front = rospy.Publisher("/frontier", GridCells, queue_size=10)

        self.cspace = rospy.Publisher("/path_planner/cspace", GridCells, queue_size=10)

        # rospy.wait_for_service('/dynamic_map') # we dont use this anywhere so why bother 
        rospy.sleep(1.0) # Sleep to allow roscore to do some housekeeping
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
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
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
        res = mapdata.info.resolution

        # this is the offset in the local frame.
        xOff = res*p[0]
        yOff = res*p[1]
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

        xOff = wp.x-oPose.x 
        yOff = wp.y-oPose.y 
        
        xP = math.trunc((xOff) / res)
        yP = math.trunc((yOff) / res)

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
        index = PathPlanner.grid_to_index(mapdata,p)
        if(not (index < len(mapdata.data) and index > 0)) :
            return False
        cellOcc = mapdata.data[index] #finds occ in p
        bounds = False
        free = False

        if p < (mapdata.info.width-1, mapdata.info.height-1):
            bounds = True

        if cellOcc < CSPACE_THRESH and cellOcc >= 0: # 75 is for expanded cells, 100 is for pre-existing walls -1 for unknown
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
        # neighbors = [Point()] * 8
        # walkNeighbors = []

        # for i in range(8): # 9 is 8 neighbors plus p
        #     # finds the x point of neighbors
        #     (neighbors[i]).x = p[0] + (((i + 2) % 3) - 1)

        #     # finds the y point of neighbors
        #     (neighbors[i]).y = p[1] + int((i / 3) - 1)

        #     if PathPlanner.is_cell_walkable(mapdata, (neighbors[i].x, neighbors[i].y)):
        #         walkNeighbors.append(i)

        # return walkNeighbors
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
        oc = rospy.ServiceProxy('/dynamic_map', GetMap)
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
        occ = [] #collects occupancy data for cells
        clist = [] #collects data of c-space
        # alpha = 0.5
        width = mapdata.info.width
        height = mapdata.info.width
        dataLength = len(mapdata.data)

        for i in range(0, dataLength): #iterates through each point
            fill = False
            x = int(i % width)
            y = int(i / width)
            # cellIndex = PathPlanner.grid_to_index(mapdata, (x,y))
            cellVal = mapdata.data[i]
            if cellVal < 0: # unknown cell case
                occ.append(-1)
                continue
            elif cellVal > int(CSPACE_THRESH-1): # filled case
                occ.append(cellVal)
                clist.append(PathPlanner.grid_to_world(mapdata, (x,y)))
                continue
            ## Inflate the obstacles where necessary
            j = -padding
            k = -padding
            while not fill and j < padding +1:
                cellY = y + j
                while not fill and k < padding +1:
                    if(abs(j) == CSPACE_PADDING and abs(k) == CSPACE_PADDING) :
                        k = k + 1
                        continue
                    cellX = x + k
                    index = PathPlanner.grid_to_index(mapdata, [cellX,cellY])
                    if(not (index < 0 or index >= dataLength) and mapdata.data[index] > CSPACE_THRESH):
                        fill = True
                    k = k+1
                j = j+1
                k = -padding

            if not fill:
                occ.append(0)
            else:
                occ.append(CSPACE_THRESH)
                clist.append(PathPlanner.grid_to_world(mapdata, (x,y)))

        ## Create a OccupancyGrid message and publish it
        occgrid = OccupancyGrid()
        occgrid.header = mapdata.header
        occgrid.info = mapdata.info
        occgrid.data = occ

        # Create a GridCells message and publish it
        cspace = GridCells()
        cspace.header = mapdata.header
        cspace.cell_height = mapdata.info.resolution / 2.0
        cspace.cell_width = mapdata.info.resolution / 2.0
        cspace.cells = clist
        self.cspace.publish(cspace)
        
        ## Return the C-space
        rospy.loginfo("Done Calc C space")
        return occgrid
    
    def findFrontier(self, msg) -> (GridCells, OccupancyGrid):
        #if we have a open cell that neighbors an unknown cell, we have a frontier cell

        mapdata = self.calc_cspace(msg.inMap, CSPACE_PADDING)
        width = mapdata.info.width
        dataLength = len(mapdata.data)
        frontList = [] 

        for i in range(0, dataLength): #iterates through each point
            frontFound = False
            x = int(i % width)
            y = int(i / width)
            cellVal = mapdata.data[i]
            if cellVal < 0 or cellVal > CSPACE_THRESH-1:  # if cell is unknown or filled we skip
                continue
            j = -1
            k = -1
            while not frontFound and j < 2:
                cellY = y + j
                while not frontFound and k < 2:
                    cellX = x + k
                    index = PathPlanner.grid_to_index(mapdata, [cellX,cellY])
                    if mapdata.data[index] == -1:
                        frontFound = True
                        frontList.append(PathPlanner.grid_to_world(mapdata, (x,y)))
                    k = k+1
                j = j+1
                k = -1

        gc = GridCells()
        gc.cell_height = mapdata.info.resolution / 2
        gc.cell_width = mapdata.info.resolution / 2
        gc.header = mapdata.header
        gc.cells = frontList
        
        self.front.publish(gc)
        return gc, mapdata

    def a_star(self, mapdata: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = dict()
        total_cost = dict()
        came_from[start] = start 
        total_cost[start] = 0
        path=[]

        aStarGC=GridCells()
        aStarGC.header = mapdata.header
        aStarGC.cell_width = mapdata.info.resolution / 1.5
        aStarGC.cell_height = mapdata.info.resolution / 1.5
        aStarGC.cells = []
 
        #while queue still contains positions
        while not frontier.empty():
            current = frontier.get()
            if current == goal: # we found the goal, now work backwards to get the path
                while current is not start:                    
                    aStarGC.cells.append(PathPlanner.grid_to_world(mapdata,current))
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
            # we didnt find goal, search neighbors
            neighbors = PathPlanner.neighbors_of_8(mapdata,current)
            prevCell = came_from[current]
            prevDirection = (current[0] - prevCell[0], current[1] - prevCell[1])

            for next in neighbors :
                new_cost = total_cost[current] + PathPlanner.euclidean_distance(current, next)
                if next not in total_cost or new_cost < total_cost[next]:
                    directionPenalty = 0.2 # 0.1 worked well
                    
                    nextDirection = (next[0]-current[0], next[1]-current[1])
                    if nextDirection == prevDirection :
                        directionPenalty = 0

                    cspacePenalty = self.calc_CspacePenalty(mapdata, next)

                    total_cost[next] = new_cost+cspacePenalty + directionPenalty
                    frontier.put(next, new_cost + PathPlanner.euclidean_distance(goal, next) + directionPenalty + cspacePenalty)
                    came_from[next] = current
        return path[::-1]
    
    def calc_CspacePenalty(self, mapdata, cell) -> float:
        # we want to disincentivize cells that are near a lot of occupied cells
        numWalkable = 0.0
        for i in range (-2, 3) :
            cellY = cell[1] + i
            for j in range(-2, 3) :
                cellX = cell[0] + j
                if PathPlanner.is_cell_walkable(mapdata, (cellX, cellY)) :
                    numWalkable = numWalkable + 1
        penalty = 0.09*(24.0-numWalkable)
        return penalty

    @staticmethod
    def optimize_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        # find direction from previous node in path, 
        # if next node AND next next node have opposite direction, remove next node and continue.
        pathCopy = [path[0]]
        i = 1 # start at second node

        while i < (len(path)-1) :
            prevDirection = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            currDirection = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])

            if not prevDirection == currDirection :
                pathCopy.append(path[i])
            i = i+1
        # find direction from previous node in path, 
        # if next node and prev have same direction, remove next node and continue.
        pathCopy.append(path[len(path)-1])

        for p in range(0, len(pathCopy)-1):
            dist = PathPlanner.euclidean_distance(pathCopy[p],pathCopy[p+1]) * RESOLUTION
            if (dist >= 1):
                midX = int((pathCopy[p+1][0]-pathCopy[p][0])/2 + pathCopy[p][0])
                midY = int((pathCopy[p+1][1]-pathCopy[p][1])/2 + pathCopy[p][1])
                midPoint = (midX, midY)
                pathCopy.insert(p+1,midPoint)
        return pathCopy

        

    def path_to_message(self, mapdata: OccupancyGrid, path: list[tuple[int, int]]) -> Path:
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        
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
        mapdata = msg.map
        if mapdata is None:
            print("MAPDATA ERROR")
            return Path()
        
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(mapdata, start, goal)
        if len(path) == 0 :
            print("NO PATH FOUND!")
            return Path()
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)

        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        rospy.spin()

        
if __name__ == '__main__':
    PathPlanner().run()

#!/usr/bin/env python3

import rospy
import roslaunch

from lastlab.srv import GetGridCells, GetPlan2
import path_planner
from path_planner import PathPlanner
from typing import Tuple as tuple
from typing import List as list
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from priority_queue import PriorityQueue
from pathlib import Path as filePath

NAV_TOLERANCE = 0.215
RECALC_CONST = 20

# TODO {Always} tune gmapping params to better fit our proj

class FinalProj:

    def __init__(self):
        rospy.init_node("final")
        self.centers = rospy.Publisher("/centers", GridCells, queue_size=3)
        self.navPath = rospy.Publisher("/navPath", Path, queue_size=3)

        self.finalPathPub = rospy.Publisher("/FinalPath", GridCells, queue_size=10)
        self.stopPub = rospy.Publisher("/STOP", String, queue_size=10)
        

        # init service proxies
        self.pathP = rospy.ServiceProxy('plan_path', GetPlan2)
        self.frontierFinder = rospy.ServiceProxy('get_Frontier', GetGridCells)

        self.currentPointSub = rospy.Subscriber("/currentPosition", Point, self.getPositionDetails)

        self.mapSub = rospy.Subscriber("/map", OccupancyGrid, self.updateMap)
        self.currentPoint = Point()
        self.currMap = OccupancyGrid()
        self.CSpaceMap = OccupancyGrid() # most up to date map WITH cspace
        self.currNavGoal = Point()

        self.initPos = Point()
        
        rospy.sleep(2.0) # wait a lil longer for map to update fully
        rospy.loginfo("final node ready")

    def updateMap(self, msg) :
        self.currMap = msg

    def run(self):
        # wont start unless both path_planner and lab2 are up and running
        rospy.wait_for_service('get_Frontier') 
        rospy.wait_for_service('plan_path')
        
        # gets our current position b4 moving anywhere. We will nav back here for phase 2.
        self.initPos = self.currentPoint
        initCell = PathPlanner.world_to_grid(self.currMap, self.initPos)
 
        print("init pose ", self.initPos)
        print("init grid ", initCell)
        
        recalcCounter = 0

        # launches gmapping
        # rospy.init_node('en_Mapping', anonymous=True)
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # gmapping = roslaunch.parent.ROSLaunchParent(uuid, ["~/launch/gmapping.launch"])
        # gmapping.launch()
        # process = launch.launch(turtlebot3_gmapping)

        # while process.is_alive() :
        #     rospy.sleep(1)  

        # rospy.loginfo("gmapping on")

        # our nav routine runs until we cannot find a frontier (navRoutine returns false in this case)
        while(1) : 
            if not self.navRoutine() :
                self.stopPub.publish("STOP") # we should stop moving and recalc our path.
                break
            continueNav = True
            # once we have a path, we continue to follow it until A) we reach the cell B) we find the cell to be non-navigable
            while continueNav:
                rospy.sleep(0.1) # dont want to be hogging all of the CPU

                # check A) are we close enough to the goal cell
                navPos = PathPlanner.grid_to_world(self.currMap, self.currNavGoal)

                currPos = self.currentPoint
                currCell = PathPlanner.world_to_grid(self.currMap, currPos)

                currDist = PathPlanner.euclidean_distance((currPos.x,currPos.y), (navPos.x, navPos.y))
                print("currDist", round(currDist,3))
                if(currDist < NAV_TOLERANCE) :
                    continueNav = False
                    self.stopPub.publish("STOP")
                
                if recalcCounter > RECALC_CONST :
                    try :
                        self.CSpaceMap = self.frontierFinder(self.currMap).map
                    except(rospy.exceptions.TransportInitError) : 
                        rospy.sleep(1)
                        rospy.loginfo("Oops try again ")
                        self.CSpaceMap = self.frontierFinder(self.currMap).map
                    
                    recalcCounter = 0

                    if not self.isPathOK(self.CSpaceMap) :
                        continueNav = False
                        self.stopPub.publish("STOP")
                else :
                    recalcCounter = recalcCounter + 1
                
                # now lets check if we are in still in the C-Space
                if not PathPlanner.is_cell_walkable(self.CSpaceMap, currCell) :
                    print("we might not be in cspace")
                    # we are lenient, and check our neighbors too. 
                    neighbors = PathPlanner.neighbors_of_8(self.CSpaceMap, currCell)
                    if len(neighbors) : 
                        print("false alarm")
                    else :# If none of them are in the cspace, then we should flag ourselves
                        print("correcting")
                        self.stopPub.publish("STOP")
                        self.correctionRoutine(self.CSpaceMap, currPos, currCell)
                        continueNav = False
        # once phase 1 is done, we do phase 2
        self.PhaseTwo()


    def navRoutine(self) -> bool: 
        rospy.loginfo("Starting Nav Routine")
        try :
            front = self.frontierFinder(self.currMap)
        except(rospy.exceptions.TransportInitError) : 
            rospy.sleep(1)
            rospy.loginfo("Oops try again ")
            front = self.frontierFinder(self.currMap)
                    

        # self.currMap = front.map
        self.CSpaceMap = front.map # mybe need this?

        rospy.loginfo("Frontier Cells found: " + str(len(front.frontier.cells)))
        if len(front.frontier.cells) == 0:
            return False
        
        currPos = self.currentPoint
        currCell = PathPlanner.world_to_grid(front.map, currPos)
        if not PathPlanner.is_cell_walkable(front.map, currCell) :
            self.correctionRoutine(self.CSpaceMap, currPos, currCell)

        #Seperates frontier data into lists of connected cells
        bfs = self.bfsForFrontier(front.frontier, front.map)

        #finds the center of each centroid
        centriodFind = self.centeroid(front.frontier, front.map, bfs)

        finalPath = self.bestFrontier(front.frontier, front.map, centriodFind)

        # now we nav to the shortest path
        # IF THERE ARE NO PATHS, we also return false
        if(len(finalPath.poses) == 0):
            return False
        self.sendNavGoal(front.map, finalPath)
        
        self.currNavGoal = PathPlanner.world_to_grid(front.map, finalPath.poses[-1].pose.position)
        return True
        
    
    def bestFrontier(self, frontier: GridCells, mapdata: OccupancyGrid, centroids: list[tuple[int,int]]) -> Path:
        # this function does an A* to each centroid and returns the shortest path,
        start = PoseStamped()

        currPos = self.currentPoint
        start.pose.position = currPos
        finalPath = Path()
        currPath = Path()
        finalPathSize = 1000000 # large num as placeholder for 

        for c in range(0,len(centroids)):
            goal = PoseStamped()

            goal.pose.position = PathPlanner.grid_to_world(mapdata,centroids[c])

            currPath = self.pathP(start, goal, 0.1, mapdata).plan 
            currentPathSize = len(currPath.poses)

            if currentPathSize == 0:
                continue

            if currentPathSize < finalPathSize:
                finalPathSize = currentPathSize
                finalPath = currPath
        return finalPath
    
    def publishPath(self, mapdata: OccupancyGrid, finalPath):

        pathCells = []
        for i in range (0, len(finalPath.poses)):
            pathCells.append(finalPath.poses[i].pose.position)

        finalPathGC = GridCells()
        finalPathGC.cell_height = mapdata.info.resolution / 1.2
        finalPathGC.cell_width = mapdata.info.resolution / 1.2
        finalPathGC.header = mapdata.header
        finalPathGC.cells = pathCells
        self.finalPathPub.publish(finalPathGC)

    def centeroid(self, frontier: GridCells, mapdata: OccupancyGrid, frontierGroups: list[list[tuple[int,int]]]) -> list[tuple[int,int]]:
        initCentroids = []
        for group in frontierGroups:
            xAvg = 0
            yAvg = 0
            for cell in group:
                xAvg = cell[0]+xAvg
                yAvg = cell[1]+yAvg

            initCentroids.append((int(xAvg/len(group)),int(yAvg/len(group))))

        # inital Centriods may not be in frontier, so we convert them to the nearest frontier cell IF NOT walkable.
        finalDist = 100000
        centCells = []
        finalCentriods = []
        for cent in initCentroids:
            if (PathPlanner.is_cell_walkable(mapdata,cent) == False):
                for f in frontier.cells:
                    distance = PathPlanner.euclidean_distance(cent, PathPlanner.world_to_grid(mapdata,f))
                    if (distance < finalDist):
                        finalDist = distance
                        newCent = PathPlanner.world_to_grid(mapdata,f)
                cent = newCent
            finalCentriods.append(cent) # appends either updated cell or old cell if walkable.
            updatedCentroid = PathPlanner.grid_to_world(mapdata, cent)
            centCells.append(updatedCentroid)
        gc = GridCells()
        gc.cell_height = mapdata.info.resolution / 1.4
        gc.cell_width = mapdata.info.resolution / 1.4
        gc.header = mapdata.header
        gc.cells = centCells
        self.centers.publish(gc)
        return finalCentriods
    
    def bfsForFrontier(self, frontier: GridCells, mapdata: OccupancyGrid) -> list[list[tuple[int,int]]]:
        allFrontier = []
        for g in frontier.cells:
            allFrontier.append(PathPlanner.world_to_grid(mapdata,g))

        frontierQueue = PriorityQueue()
        visited = set()                                                                                                                                          

        groups = []
        subGroup = []
        
        for f in range(0,len(allFrontier)):
            startCell = allFrontier[f]
            if startCell in visited:
                continue
            subGroup = []
            frontierQueue.put(startCell, 0)
            subGroup.append(startCell)
            while not frontierQueue.empty():
                cell = frontierQueue.get()
                visited.add(cell)
                neighbors = PathPlanner.neighbors_of_8(mapdata,cell)
                subGroup.append(cell)

                for n in neighbors:
                    if n not in visited and n in allFrontier:
                        frontierQueue.put(n,1)
            groups.append(subGroup)
        return groups

    def sendNavGoal(self, map: OccupancyGrid, finalPath: Path) :

        self.currPath = finalPath
        self.publishPath(map, finalPath)
        self.navPath.publish(finalPath)

    def findClosestOpenCell(self, mapdata, startCell) -> tuple[int, int]:
        # Method finds the closest open cell from our current cell and returns its gridcell location
        # self.currMap = self.frontierFinder().map

        startIndex = PathPlanner.grid_to_index(mapdata, startCell)
        if(mapdata.data[startIndex] == 0) :
            print("StartCell is in C-Space")
            return startCell

        cellQueue = PriorityQueue()
        cellQueue.put(startCell, 0)
        visited = []
        total_cost = dict()
        total_cost[startCell] = 0

        while not cellQueue.empty() :
            currCell = cellQueue.get()
            visited.append(currCell)
            currIndex = PathPlanner.grid_to_index(mapdata, currCell)
            if(mapdata.data[currIndex] == 0) :
                print("found an open cell: ", currCell)
                return currCell
            
            # need to get all neighbors regardless of walkability.
            neighbors = []
            for i in range(-1, 2) :
                for j in range(-1, 2):
                    if i == j and i == 0 :
                        continue
                    neighbors.append((currCell[0]+i, currCell[1]+j))
            for nCell in neighbors:
                nCellIndex = PathPlanner.grid_to_index(mapdata, nCell)
                nVal = mapdata.data[nCellIndex] 
                # print("nVal", nVal)
                if(nVal == 0) :
                    return nCell
                elif (nVal < 100 and nVal > 0 and nCell not in visited) :
                    total_cost[nCell] = total_cost[currCell] + 1
                    cellQueue.put(nCell, total_cost[nCell]+nVal) # open cells should be highest priority

        return None

    def correctionRoutine(self, mapdata, currPos, currCell) :
        openCell = self.findClosestOpenCell(mapdata, currCell)
        if(openCell == None) :
            print("NO OPEN CELL FOUND!")
            return currCell
        
        #get the cardinal direction of the openCell 
        # endDirection = (openCell[0] - currCell[0], openCell[1] - currCell[1])
        # endDirNorm = (endDirection[0] / abs(endDirection[0]), endDirection[1] / abs(endDirection[1]))
        # dirMod = 3
        # endCell = (int(currCell[0] + endDirection[0] + endDirNorm[0]*dirMod), int(currCell[1] + endDirection[1]+ endDirNorm[1]*dirMod))

        # while self.currMap.data[PathPlanner.grid_to_index(self.currMap, endCell)] != 0 and dirMod > 0 :
        #     print("next cell over is bad trying again")
        #     dirMod = dirMod - 1
        #     endCell = (int(currCell[0] + endDirection[0] + endDirNorm[0]*dirMod), int(currCell[1] + endDirection[1]+ endDirNorm[1]*dirMod))
        
        startFixPose = PoseStamped()

        startFixPose.pose.position = currPos

        endFixPose = PoseStamped()

        endFixPose.pose.position =  PathPlanner.grid_to_world(mapdata, openCell)


        fixPath = Path()
        fixPath.poses = [startFixPose, endFixPose]
        self.sendNavGoal(mapdata, fixPath)
        # once we send the goal, we should wait for our robot to finish this path, and then continue its nav
        try :
            (rospy.wait_for_message("/resume", String, timeout=rospy.Duration.from_sec(10.))) # add a timeout clause?
        except (rospy.exceptions.ROSException):
            print("Timed out")
            self.stopPub.publish("STOP")

    def PhaseTwo(self) :
        # Save the map!
        package = 'map_server'
        executable = 'map_saver'
        node = roslaunch.core.Node(package, executable)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        
        try :
            self.CSpaceMap = self.frontierFinder(self.currMap).map
        except(rospy.exceptions.TransportInitError) : 
            rospy.sleep(1)
            rospy.loginfo("Oops try again ")
            self.CSpaceMap = self.frontierFinder(self.currMap).map
                    

        startPose = PoseStamped()
        goalPose = PoseStamped()

        currPos = self.currentPoint
        currCell = PathPlanner.world_to_grid(self.CSpaceMap, currPos)

        startPose.pose.position = currPos
        
        goalPose.pose.position = self.initPos

        if not PathPlanner.is_cell_walkable(self.CSpaceMap, currCell) :
            print("we might not be in cspace")
            neighbors = PathPlanner.neighbors_of_8(self.CSpaceMap, currCell)
            if len(neighbors) :
                print("false alarm")
            else :
                print("correcting")
                self.stopPub.publish("STOP")
                self.correctionRoutine(self.CSpaceMap, currPos, currCell)
        # self.correctionRoutine(startPose.pose.position, currCell)
        rospy.loginfo("Requesting path plan from start to goal")

        print("Current Position: ", startPose.pose.position,
              "Goal Postion: ", goalPose.pose.position)
        print("Current cell: ", currCell,
              "Goal Cell: ", PathPlanner.world_to_grid(self.currMap,goalPose.pose.position))
        gp2 = self.pathP(startPose, goalPose, 0.1, self.CSpaceMap).plan

        self.sendNavGoal(self.CSpaceMap, gp2)
        print("Naving back to start!")

        rospy.wait_for_message("/resume", String) # add a timeout clause?
        self.stopPub.publish("STOP")

        process.stop()
        filePath("/home/riley/.ros/map.pgm").rename("/home/riley/catkin_ws/src/RBE3002_B24_Team17/lastlab/maps/Realmap.pgm")
        filePath("/home/riley/.ros/map.yaml").rename("/home/riley/catkin_ws/src/RBE3002_B24_Team17/lastlab/maps/Realmap.yaml")

    def getPositionDetails(self, msg) :
        self.currentPoint = msg

    # assumes path and currmap are up to date
    def isPathOK(self, mapdata) :
        for node in self.currPath.poses :
            nodeCell = PathPlanner.world_to_grid(mapdata, node.pose.position)
            nodeIndex = PathPlanner.grid_to_index(mapdata, nodeCell)
            nodeVal = mapdata.data[nodeIndex]

            if(nodeVal > path_planner.CSPACE_THRESH-1) :
                print("Path NOT OK")
                return False
            
        return True



if __name__ ==  '__main__':
    FinalProj().run()
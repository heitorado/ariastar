from __future__ import division
from AriaPy import *
import sys
import numpy
import math as m
from heapq import *

debug = 1 # Se verdadeiro, imprime o array do mapa.

#
# Parametros do aria
#
numpy.set_printoptions(threshold=numpy.inf)

Aria_init()
parser = ArArgumentParser(sys.argv)
parser.loadDefaultArguments()
robot = ArRobot()
conn = ArRobotConnector(parser, robot)
sonar = ArSonarDevice()
if not conn.connectRobot():
  print "Could not connect to robot, exiting"
  Aria_exit(1)

if not Aria_parseArgs():
  Aria_logOptions()
  Aria_exit(1)

##########
# Mapa
#########

print "Initializing array"

endGoal = {'x': 12000, 'y': 7000}

tileSize = 510 # 51cm
mapSize = int(45 * 1000 / tileSize) # 45m de largura maxima
mapOffset = (mapSize / 2)
explored = numpy.zeros(shape=(mapSize,mapSize))

print "Done"

#Xmap = Xreal / Tsize + Offset
#Xreal = (Xmap - Offset) * Tsize (+/- Offset)

# Insere uma "parede"
def exploredInsert(x, y):
    explored[int(x / tileSize + mapOffset)][int(y / tileSize + mapOffset)] = 1

#Recupera uma coordenada do array para real
def getRealCoords(x, y):
    return ((x - mapOffset) * tileSize, (y - mapOffset) * tileSize)

def getArrayCoords(x, y): # O mesmo de cima mais arredondado
    return (int(round(x / tileSize + mapOffset)), int(round(y / tileSize + mapOffset)))

robot.addRangeDevice(sonar)
robot.runAsync(True)

##########
# Acoes
##########

recover = ArActionStallRecover()
robot.addAction(recover, 100)

gotoPoseAction = ArActionGotoStraight("goto")
robot.addAction(gotoPoseAction, 50)

stopAction = ArActionStop ("stop")
robot.addAction(stopAction, 40)


robot.enableMotors()

##
##
##

timer = ArTime()
timer.setToNow()

#
# "Hey now you are an A*..."
#

path = []
goal = getArrayCoords(endGoal['x'], endGoal['y']);

def calcDistance(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def heuristics(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])

    minval = min(dx, dy)
    maxval = max(dx, dy)

    diagonalDistance = minval
    straightDistance = maxval - minval

    return m.sqrt(2) * diagonalDistance + straightDistance

def aStar(start):

    nearby = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    closedSet = set()
    openSet = []

    cameFrom = {}
    gScore = {start: 0}
    fScore = {start: heuristics(start, goal)}

    heappush(openSet, (fScore[start], start))

    while openSet:

        current = heappop(openSet)[1]

        if current == goal:
            totalPath = []
            while current in cameFrom:
                totalPath.append(current)
                current = cameFrom[current]
            return totalPath

        closedSet.add(current)
        for i, j in nearby:
            neighbor = current[0] + i, current[1] + j
            tentativeGScore = gScore[current] + heuristics(current, neighbor)
            if 0 <= neighbor[0] < explored.shape[0]:
                if 0 <= neighbor[1] < explored.shape[1]:
                    if explored[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in closedSet and tentativeGScore >= gScore.get(neighbor, 0):
                continue

            if  tentativeGScore < gScore.get(neighbor, 0) or neighbor not in [i[1]for i in openSet]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentativeGScore
                fScore[neighbor] = tentativeGScore + heuristics(neighbor, goal)
                heappush(openSet, (fScore[neighbor], neighbor))

    return False

#
# Main
#

reroute = True

while Aria.getRunning():
    robot.lock()

    poses = sonar.getCurrentBufferAsVector()
    for p in poses:
        try:
            exploredInsert(p.x, p.y)
        except Exception, e:
            print "Error trying to insert x: %d, y: %d -> x: %d, y: %d" % (p.x, p.y, int(p.x / tileSize + mapOffset), int(y / tileSize + mapOffset))
            print e

    if (reroute or gotoPoseAction.haveAchievedGoal()):
        reroute = False
        myPos = robot.getPose()
        path = aStar(getArrayCoords(myPos.x, myPos.y))

        # Se chegarmos no tile final
        if not path:
            ArLog.log(ArLog.Normal, "GOOOOOOAL");
            print explored
            robot.unlock()
            break

        # Caso contrario adiciona-se o proximo passo
        nextTile = path[-1]
        nextPos = getRealCoords(nextTile[0], nextTile[1]);
        nextPosEase = (nextPos[0] + (myPos.x % tileSize), nextPos[1] + (myPos.x % tileSize))
        gotoPoseAction.setGoal(ArPose(nextPos[0], nextPos[1]));

        ArLog.log(ArLog.Normal, "Going to next goal at %.0f %.0f" % (gotoPoseAction.getGoal().getX(), gotoPoseAction.getGoal().getY()) );

    # Imprimir array do mapa
    if (debug and timer.mSecSince() >= 5000):
        print explored
        print "\n\n\n"
        timer.setToNow()

    distance = robot.findDistanceTo(ArPose(nextPos[0], nextPos[1]))

    # Se aproximar da tile ou essa for revelada a parede calcular denovo
    if distance < tileSize / 2 or explored[nextTile[0]][nextTile[1]] == 1:
        gotoPoseAction.cancelGoal()
        reroute = True

    robot.unlock()
    ArUtil.sleep(100)

Aria_exit(0)

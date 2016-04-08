import glob
import json
from enum import Enum

class Route:
    def __init__(self, lotSize, lotLayout, instructionsArray):
        self.lotSize = lotSize
        self.lotLayout  = lotLayout
        self.instructionsArray = instructionsArray

class GridState:
    def __init__(self, onNode, ndStat, rVertical, rMove):
        self.onNode = onNode
        self.ndStat = ndStat
        self.rVertical = rVertical
        self.rMove = rMove

class onNode(Enum):
    em = 0
    C0 = 1
    C1 = 2
    C2 = 3
    NO = 4

class ndStat(Enum):
    none = 0
    R_r = 1
    R_m = 2
    R_v = 3
    C0R_r = 4
    COR_m = 5
    C1R_r = 6
    C1R_m = 7
    C2R_r = 8
    C2R_m = 9
    NO = 10


class rVertical(Enum):
    lift = 0
    l1 = 1
    l2 = 2
    l3 = 3
    l4 = 4
    drop = 5
    NO = 6

class rMove(Enum):
    accE = 0
    mvE0 = 1
    accN = 2
    mvN1 = 3
    mvN0 = 4
    accW = 5
    mvW0 = 6
    accS = 7
    mvS1 = 8
    mvS0 = 9
    w0_accE = 10
    w0_mvE1 = 11
    w0_mvE0 = 12
    w0_accN = 13
    w0_mvN1 = 14
    w0_mvN2 = 15
    w0_mvN3 = 16
    w0_mvN0 = 17
    w0_accW = 18
    w0_mvW1 = 19
    w0_mvW0 = 20
    w0_accS = 21
    w0_mvS1 = 22
    w0_mvS2 = 23
    w0_mvS3 = 24
    w0_mvS0 = 25
    w1_accE = 26
    w1_mvE1 = 27
    w1_mvE0 = 28
    w1_accN = 29
    w1_mvN1 = 30
    w1_mvN2 = 31
    w1_mvN3 = 32
    w1_mvN0 = 33
    w1_accW = 34
    w1_mvW1 = 35
    w1_mvW0 = 36
    w1_accS = 37
    w1_mvS1 = 38
    w1_mvS2 = 39
    w1_mvS3 = 40
    w1_mvS0 = 41
    w2_accE = 42
    w2_mvE1 = 43
    w2_mvE0 = 44
    w2_accN = 45
    w2_mvN1 = 46
    w2_mvN2 = 47
    w2_mvN3 = 48
    w2_mvN0 = 49
    w2_accW = 50
    w2_mvW1 = 51
    w2_mvW0 = 52
    w2_accS = 53
    w2_mvS1 = 54
    w2_mvS2 = 55
    w2_mvS3 = 56
    w2_mvS0 = 57
    NO = 58

class gridPieces(Enum):
    wallNW = 0
    wallN = 1
    wallNE = 2
    wallE = 3
    wallSE = 4
    wallS = 5
    wallSW = 6
    wallW = 7
    noWall = 8

def getParkingLot(lotSize, layoutString):
    layouts = [layoutString[i:i+lotSize[1]] for i in range(0, len(layoutString), lotSize[1])]
    decodedLevels = []
    for level in layouts:
        decodedLevel = []
        for char in level:
            if char == 'D':
                decodedLevel.append(gridPieces(0))
            elif char == 'H':
                decodedLevel.append(gridPieces(1))
            elif char == 'G':
                decodedLevel.append(gridPieces(2))
            elif char == 'O':
                decodedLevel.append(gridPieces(3))
            elif char == 'M':
                decodedLevel.append(gridPieces(4))
            elif char == 'N':
                decodedLevel.append(gridPieces(5))
            elif char == 'J':
                decodedLevel.append(gridPieces(6))
            elif char == 'L':
                decodedLevel.append(gridPieces(7))
            elif char == 'P':
                decodedLevel.append(gridPieces(8))
            else:
                print("Character " + char + " not in library!!!")
        decodedLevels.append(decodedLevel)
    return decodedLevels

def getRoute(filename):

    with open(filename, "r") as input:
        array = []
        for line in input:
            array.append(line)

    lotSize = list(map(int, array[4].rstrip().split(" ")))
    parkingLotLayout = getParkingLot(lotSize,array[5].rstrip())
    instructionsLength = int(array[6])
    instructionsArray  = array[7:(instructionsLength + 7)]
    decodedInstructions = []
    for instrLines in instructionsArray:
        decodedInstructions.append(gridStates(instrLines.rstrip()))
    return Route(lotSize, parkingLotLayout, decodedInstructions)

def decodeGrid(grid):
    onNodeValue = onNode(ord(grid[0]) - ord('c'))
    ndStatValue = ndStat(ord(grid[1]) - ord('A'))
    rVerticalValue = rVertical(ord(grid[2]) - ord('v'))
    rMoveValue = rMove(ord(grid[3]) - ord('!'))
    return GridState(onNodeValue, ndStatValue, rVerticalValue, rMoveValue)

def gridStates (instructionString):
    instructions = [instructionString[i:i+4] for i in range(0, len(instructionString), 4)]
    instructionsDecoded = []
    for instr in instructions:
        instructionsDecoded.append(decodeGrid(instr))
    return instructionsDecoded

def printGrid(route, level):
    instructions = route.instructionsArray[level]
    parkingLotString = ""
    for i in range (0, route.lotSize[0]):
        instr = ""
        for j in range(0, route.lotSize[1]):
            instr += "|"
            instr += instructions[i*10 + j].onNode.name
            instr += "|"
            instr += instructions[i*10 + j].ndStat.name
            instr += "|"
            instr += instructions[i*10 + j].rVertical.name
            instr += "|"
            instr += instructions[i*10 + j].rMove.name
            instr += "\t"
        instr += "\n"
        parkingLotString += instr
    parkingLotString += "\n\n"
    return (parkingLotString)

def writeInstructions(route):
    f = open('instructions.txt', 'w')
    for i in range (0, len(route.instructionsArray)):
        f.write(printGrid(route, i))
    f.close()

def getCoordinates(route, level, instructionsMaker = False):
    instructions = route.instructionsArray[level]
    locationArray = []
    lotWidth = route.lotSize[1]
    carID = 0
    robotID = 0

    for i in range(0, len(instructions)):
        if (instructions[i].onNode == onNode.C0 or instructions[i].onNode == onNode.C1 or instructions[i].onNode == onNode.C2):
            placeStr = str(i)
            if len(placeStr) == 1:
                level = 0
                place = i
            else:
                level = int(placeStr[0])
                place = int(placeStr[1])

            if instructionsMaker == True:
                locationArray.append([instructions[i].onNode.name, level, place, "C" + str(carID),instructions[i]])
            else:
                locationArray.append([instructions[i].onNode.name, level, place, "C" + str(carID)])
            carID += 1
        if (instructions[i].ndStat != ndStat.none and instructions[i].ndStat != ndStat.NO):
            placeStr = str(i)
            if len(placeStr) == 1:
                level = 0
                place = i
            else:
                level = int(placeStr[0])
                place = int(placeStr[1])

            if instructionsMaker == True:
                locationArray.append(["R", level, place, "R" + str(robotID), instructions[i]])
            else:
                locationArray.append(["R", level, place, "R" + str(robotID)])

            robotID += 1

    return locationArray

def getParkingLayout(route):
    lotWidth = route.lotSize[1]
    lotHeight = route.lotSize[0]
    layoutArray = []
    machineArray = getCoordinates(route, 0)

    #Only for testing, soon there should be other types of grids needed
    for i in range(0, lotHeight):
        level = []
        for j in range(0, lotWidth):
            level.append("P")
        layoutArray.append(level)
    return (json.dumps({'width' : lotWidth, 'height' : lotHeight, 'layout' : layoutArray, 'machines' : machineArray}, separators=(',',':')))

def getRouteList():
    routes = glob.glob("../../Examples/*.robroute")
    return routes

def getJSONRouteList():
    routes = getRouteList()
    routeArray = []
    for i in range(0, len(routes)):
        routeArray.append({'name' : "Route " + str(i+1), 'value' : str(i)})
    return (json.dumps(routeArray, separators=(',',':')))

def getInstructions(route):
    initialMachines = getCoordinates(route, 0, True)
    instructionsArray = []
    for i in range(1,len(route.instructionsArray)):
        levelInstructions = []
        nextState = getCoordinates(route, i, True)
        for j in range (0, len(nextState)):
            if nextState[j][0] == 'C0' or nextState[j][0] == 'C1' or nextState[j][0] == 'C2':
                if nextState[j][4].rMove != rMove.NO:
                    move = nextState[j][4].rMove
                    # Horizontal movement of car + robot
                    if move == rMove.w0_accN or move == rMove.w0_mvN0 or move == rMove.w0_mvN1 or move == rMove.w0_mvN2 or move == rMove.w0_mvN3 \
                        or move == rMove.w1_accN or move == rMove.w1_mvN0 or move == rMove.w1_mvN1 or move == rMove.w1_mvN2 or move == rMove.w1_mvN3 \
                        or move == rMove.w2_accN or move == rMove.w2_mvN0 or move == rMove.w2_mvN1 or move == rMove.w2_mvN2 or move == rMove.w2_mvN3:
                        x = nextState[j][1]
                        y = nextState[j][2]
                        nextState[j][3] = getCarID(initialMachines, x, y, 'C')
                        levelInstructions.append([nextState[j][3], 'N', 4])
                    elif move == rMove.w0_accS or move == rMove.w0_mvS0 or move == rMove.w0_mvS1 or move == rMove.w0_mvS2 or move == rMove.w0_mvS3 \
                        or move == rMove.w1_accS or move == rMove.w1_mvS0 or move == rMove.w1_mvS1 or move == rMove.w1_mvS2 or move == rMove.w1_mvS3 \
                        or move == rMove.w2_accS or move == rMove.w2_mvS0 or move == rMove.w2_mvS1 or move == rMove.w2_mvS2 or move == rMove.w2_mvS3:
                        x = nextState[j][1]
                        y = nextState[j][2]
                        nextState[j][3] = getCarID(initialMachines, x, y, 'C')
                        levelInstructions.append([nextState[j][3], 'S', 4])
                    elif move == rMove.w0_accE or move == rMove.w0_mvE0 or move == rMove.w0_mvE1  \
                        or move == rMove.w1_accE or move == rMove.w1_mvE0 or move == rMove.w1_mvE1 \
                        or move == rMove.w2_accE or move == rMove.w2_mvE0 or move == rMove.w2_mvE1:
                        x = nextState[j][1]
                        y = nextState[j][2]
                        nextState[j][3] = getCarID(initialMachines, x, y, 'C')
                        levelInstructions.append([nextState[j][3], 'E', 2])
                    elif move == rMove.w0_accW or move == rMove.w0_mvW0 or move == rMove.w0_mvW1  \
                        or move == rMove.w1_accW or move == rMove.w1_mvW0 or move == rMove.w1_mvW1  \
                        or move == rMove.w2_accW or move == rMove.w2_mvW0 or move == rMove.w2_mvW1 :
                        x = nextState[j][1]
                        y = nextState[j][2]
                        nextState[j][3] = getCarID(initialMachines, x, y, 'C')
                        levelInstructions.append([nextState[j][3], 'E', 2])
            if nextState[j][0] == 'R':
                if nextState[j][4].rMove != rMove.NO:
                    move = nextState[j][4].rMove
                # Horizontal movement of robot
                    if move == rMove.accE or move == rMove.mvE0:
                        x = nextState[j][1]
                        y = nextState[j][2]
                        nextState[j][3] = getCarID(initialMachines, x, y, 'R')
                        levelInstructions.append([nextState[j][3],'E',1])
                    elif move == rMove.accW or move == rMove.mvW0:
                        x = nextState[j][1]
                        y = nextState[j][2]
                        nextState[j][3] = getCarID(initialMachines, x, y, 'R')
                        levelInstructions.append([nextState[j][3],'W',1])
                    elif move == rMove.accN or move == rMove.mvN0:
                        x = nextState[j][1]
                        y = nextState[j][2]
                        nextState[j][3] = getCarID(initialMachines, x, y, 'R')
                        levelInstructions.append([nextState[j][3],'N',2])
                    elif move == rMove.accS or move == rMove.mvS0:
                        x = nextState[j][1]
                        y = nextState[j][2]
                        nextState[j][3] = getCarID(initialMachines, x, y, 'R')
                        levelInstructions.append([nextState[j][3],'S',2])
        instructionsArray.append(levelInstructions)
    return instructionsArray


def getCarID (machines, x, y, type):
    for machine in machines:
        if machine[1] == x and machine[2] == y:
            if type == 'C' and machine[0] != 'R':
                return machine[3]
            elif type == 'R' and machine[0] == 'R':
                return machine[3]

route = getRoute(getRouteList()[0])

print(getInstructions(route))


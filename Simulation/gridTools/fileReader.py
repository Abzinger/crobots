import glob
import json
from enum import Enum


class Route:
    def __init__(self, lot_size, lot_layout, instructions_array):
        self.lot_size = lot_size
        self.lot_layout = lot_layout
        self.instructions_array = instructions_array

# Grid with 4 properties
# onNode - what kind of car is on the grid
# ndStat - What is robot doing? Is it moving, is it ready etc.
# rVertical - is robot moving vertically (lifting the car up or dropping it)
# rMove - is the machine(s) on the grid moving.


class GridState:
    def __init__(self, on_node, nd_stat, r_vertical, r_move):
        self.on_node = on_node
        self.nd_stat = nd_stat
        self.r_vertical = r_vertical
        self.r_move = r_move


# Different onNode states
# onNode - what kind of car is on the grid
class OnNode(Enum):
    em = 0
    C0 = 1
    C1 = 2
    C2 = 3
    NO = 4


# Different ndStat states
# ndStat - What is robot doing? Is it moving, is it ready, is it moving with car etc.
class NodeStatus(Enum):
    none = 0
    R_r = 1
    R_m = 2
    R_v = 3
    C0R_r = 4
    C0R_m = 5
    C1R_r = 6
    C1R_m = 7
    C2R_r = 8
    C2R_m = 9
    NO = 10


# Different rVertical states
# rVertical - is robot moving vertically (lifting the car up or dropping it)
class RobotVertical(Enum):
    lift = 0
    l1 = 1
    l2 = 2
    l3 = 3
    l4 = 4
    drop = 5
    NO = 6


# Different rMove states
# rMove - is the machine(s) on the grid moving.
class RobotMove(Enum):
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


# Different pieces of grid
# noWall - robot can drive from this grid to every direction
# wall* - robot cannot drive to direction *
class GridPieces(Enum):
    wallNW = 0
    wallN = 1
    wallNE = 2
    wallE = 3
    wallSE = 4
    wallS = 5
    wallSW = 6
    wallW = 7
    noWall = 8


# getParkingLot takes the size of the lot and the layout string from the file and will convert it to two-dimensional
# table that will later be used to track the machines' movement.
def get_parking_lot(lot_size, layout_string):
    layouts = [layout_string[i:i + lot_size[1]] for i in range(0, len(layout_string), lot_size[1])]
    decoded_levels = []
    for level in layouts:
        decoded_level = []
        for char in level:
            if char == 'D':
                decoded_level.append(GridPieces(0))
            elif char == 'H':
                decoded_level.append(GridPieces(1))
            elif char == 'G':
                decoded_level.append(GridPieces(2))
            elif char == 'O':
                decoded_level.append(GridPieces(3))
            elif char == 'M':
                decoded_level.append(GridPieces(4))
            elif char == 'N':
                decoded_level.append(GridPieces(5))
            elif char == 'J':
                decoded_level.append(GridPieces(6))
            elif char == 'L':
                decoded_level.append(GridPieces(7))
            elif char == 'P':
                decoded_level.append(GridPieces(8))
            else:
                print("Character " + char + " not in library!!!")
        decoded_levels.append(decoded_level)
    return decoded_levels


# getRoute will get the file name of the .robroute (parking lot instruction set) as an input
# As an output Route class object will be returned that is the basis for all the movements.
def get_route(file_name):
    with open(file_name, "r") as robot_route_file:
        array = []
        for line in robot_route_file:
            array.append(line)

    lot_size = list(map(int, array[4].rstrip().split(" ")))
    parking_lot_layout = get_parking_lot(lot_size, array[5].rstrip())
    instructions_length = int(array[6])
    instructions_array = array[7:(instructions_length + 7)]
    decoded_instructions = []
    for instruction_lines in instructions_array:
        decoded_instructions.append(grid_states(instruction_lines.rstrip()))
    return Route(lot_size, parking_lot_layout, decoded_instructions)


# decodeGrid will decode one piece of parking layout and will return the states and other values of that grid as a
# GridState object.
def decode_grid(grid):
    on_node_value = OnNode(ord(grid[0]) - ord('c'))
    node_status_value = NodeStatus(ord(grid[1]) - ord('A'))
    if node_status_value == NodeStatus.none:
        node_status_value = NodeStatus.NO
    robot_vertical_value = RobotVertical(ord(grid[2]) - ord('v'))
    robot_movement_value = RobotMove(ord(grid[3]) - ord('!'))
    return GridState(on_node_value, node_status_value, robot_vertical_value, robot_movement_value)


# gridStates is a helper function that takes the instruction string from the file as an input and
# will return instructions as an array of GridState objects.
def grid_states(instruction_string):
    instructions = [instruction_string[i:i + 4] for i in range(0, len(instruction_string), 4)]
    decoded_instructions = []
    for instr in instructions:
        decoded_instructions.append(decode_grid(instr))
    return decoded_instructions


# getCoordinates takes the route object as an input and will return the coordinates of all the machines
# on one instruction level as an array of arrays.
def get_coordinates(route, level, making_instructions=False):
    instructions = route.instructions_array[level]
    location_array = []
    lot_width = route.lot_size[1]
    car_id = 0
    robot_id = 0

    for i in range(0, len(instructions)):
        if (instructions[i].on_node == OnNode.C0 or
                instructions[i].on_node == OnNode.C1 or
                instructions[i].on_node == OnNode.C2 or
                instructions[i].nd_stat == NodeStatus.C0R_r or
                instructions[i].nd_stat == NodeStatus.C0R_m or
                instructions[i].nd_stat == NodeStatus.C1R_r or
                instructions[i].nd_stat == NodeStatus.C1R_m or
                instructions[i].nd_stat == NodeStatus.C2R_r or
                instructions[i].nd_stat == NodeStatus.C2R_m):

                level = i // lot_width
                place = i - (level * lot_width)

                if making_instructions:
                    location_array.append([instructions[i].on_node.name, level, place,
                                           "C" + str(car_id), instructions[i]])
                else:
                    location_array.append([instructions[i].on_node.name, level, place, "C" + str(car_id)])
                car_id += 1

        if instructions[i].nd_stat != NodeStatus.none and instructions[i].nd_stat != NodeStatus.NO:

            level = i // lot_width
            place = i - (level * lot_width)

            if making_instructions:
                location_array.append(["R", level, place, "R" + str(robot_id), instructions[i]])
            else:
                location_array.append(["R", level, place, "R" + str(robot_id)])

            robot_id += 1
    return location_array


# getParkingLayout will return the layout and the initial machine positioning as a JSON object - used by client.
def get_parking_layout(route, first=True):
    lot_width = route.lot_size[1]
    lot_height = route.lot_size[0]
    layout_array = []
    if first:
        machine_array = get_coordinates(route, 0)
    else:
        machine_array = get_coordinates(route, len(route.instructions_array) - 1)

    for i in range(0, lot_height):
        level = []
        for j in range(0, lot_width):
            level.append("P")
        layout_array.append(level)
    return (json.dumps({'width': lot_width, 'height': lot_height, 'layout': layout_array,
                        'machines': machine_array}, separators=(',', ':')))


# returns list of routes (.robroute files in Examples directory)
def get_route_list():
    routes = glob.glob("../Examples/*.robroute")
    return routes


# returns the list of routes as a JSON objects
def get_json_route_list():
    routes = get_route_list()
    route_array = []
    for i in range(0, len(routes)):
        route_array.append({'name': "Route " + str(i+1), 'value': str(i)})
    return json.dumps(route_array, separators=(',', ':'))


# Crucial part of the program - getInstructions will transform the instructions from file to implicit instructions
# where a machine will be given ID and a list of instructions  - for example C1 (car ID 1), 'W', 2 would mean that
# car object with ID 1 should at this step move west with the speed of 2 (number means how many steps it takes to move
# from a grid to adjacent grid.
def get_instructions(route):
    initial_machines = get_coordinates(route, 0, True)
    instructions_array = []
    for i in range(0, len(route.instructions_array)):
        instructions_array.append([])
    for machine in initial_machines:
        y = machine[1]
        x = machine[2]
        machine_id = machine[3]
        state = machine[4]
        machine_type = 'C'
        if machine[0] == 'R':
            machine_type = 'R'
        for i in range(1, len(route.instructions_array)):
            next_state = get_coordinates(route, i, True)
            if machine_type == 'R':
                if state.r_move != RobotMove.NO:
                    if (state.r_move == RobotMove.mvW0 or
                            state.r_move == RobotMove.accW):
                            instructions_array[i].append([machine_id, 'W', 1])
                            x -= 1
                    elif (state.r_move == RobotMove.mvE0 or
                            state.r_move == RobotMove.accE):
                            instructions_array[i].append([machine_id, 'E', 1])
                            x += 1
                    elif state.r_move == RobotMove.mvN1:
                        y += 1
                    elif state.r_move == RobotMove.mvS1:
                        y -= 1
                    elif (state.r_move == RobotMove.w0_mvN3 or
                            state.r_move == RobotMove.w1_mvN3 or
                            state.r_move == RobotMove.w2_mvN3):
                            y += 1
                    elif (state.r_move == RobotMove.w0_mvS3 or
                            state.r_move == RobotMove.w1_mvS3 or
                            state.r_move == RobotMove.w2_mvS3):
                            y -= 1
                    elif (state.r_move == RobotMove.w0_mvE1 or
                            state.r_move == RobotMove.w1_mvE1 or
                            state.r_move == RobotMove.w2_mvE1):
                            x += 1
                    elif (state.r_move == RobotMove.w0_mvW1 or
                            state.r_move == RobotMove.w1_mvW1 or
                            state.r_move == RobotMove.w2_mvW1):
                            x -= 1

                    if (state.r_move == RobotMove.accN or
                            state.r_move == RobotMove.mvN1 or
                            state.r_move == RobotMove.mvN0):
                            instructions_array[i].append([machine_id, 'S', 2])
                    elif (state.r_move == RobotMove.accS or
                            state.r_move == RobotMove.mvS1 or
                            state.r_move == RobotMove.mvS0):
                            instructions_array[i].append([machine_id, 'N', 2])
                    elif (state.r_move == RobotMove.w0_mvN0 or
                            state.r_move == RobotMove.w0_mvN1 or
                            state.r_move == RobotMove.w0_mvN2 or
                            state.r_move == RobotMove.w0_mvN3 or
                            state.r_move == RobotMove.w1_mvN0 or
                            state.r_move == RobotMove.w1_mvN1 or
                            state.r_move == RobotMove.w1_mvN2 or
                            state.r_move == RobotMove.w1_mvN3 or
                            state.r_move == RobotMove.w2_mvN0 or
                            state.r_move == RobotMove.w2_mvN1 or
                            state.r_move == RobotMove.w2_mvN2 or
                            state.r_move == RobotMove.w2_mvN3 or
                            state.r_move == RobotMove.w0_accN or
                            state.r_move == RobotMove.w1_accN or
                            state.r_move == RobotMove.w1_accN):
                            instructions_array[i].append([machine_id, 'S', 4])
                    elif (state.r_move == RobotMove.w0_mvS0 or
                            state.r_move == RobotMove.w0_mvS1 or
                            state.r_move == RobotMove.w0_mvS2 or
                            state.r_move == RobotMove.w0_mvS3 or
                            state.r_move == RobotMove.w1_mvS0 or
                            state.r_move == RobotMove.w1_mvS1 or
                            state.r_move == RobotMove.w1_mvS2 or
                            state.r_move == RobotMove.w1_mvS3 or
                            state.r_move == RobotMove.w2_mvS0 or
                            state.r_move == RobotMove.w2_mvS1 or
                            state.r_move == RobotMove.w2_mvS2 or
                            state.r_move == RobotMove.w2_mvS3 or
                            state.r_move == RobotMove.w0_accS or
                            state.r_move == RobotMove.w1_accS or
                            state.r_move == RobotMove.w1_accS):
                            instructions_array[i].append([machine_id, 'N', 4])
                    elif (state.r_move == RobotMove.w0_accE or
                            state.r_move == RobotMove.w0_mvE0 or
                            state.r_move == RobotMove.w0_mvE1 or
                            state.r_move == RobotMove.w1_accE or
                            state.r_move == RobotMove.w1_mvE0 or
                            state.r_move == RobotMove.w1_mvE1 or
                            state.r_move == RobotMove.w2_accE or
                            state.r_move == RobotMove.w2_mvE0 or
                            state.r_move == RobotMove.w2_mvE1):
                            instructions_array[i].append([machine_id, 'E', 2])
                    elif (state.r_move == RobotMove.w0_accW or
                            state.r_move == RobotMove.w0_mvW0 or
                            state.r_move == RobotMove.w0_mvW1 or
                            state.r_move == RobotMove.w1_accW or
                            state.r_move == RobotMove.w1_mvW0 or
                            state.r_move == RobotMove.w1_mvW1 or
                            state.r_move == RobotMove.w2_accW or
                            state.r_move == RobotMove.w2_mvW0 or
                            state.r_move == RobotMove.w2_mvW1):
                            instructions_array[i].append([machine_id, 'W', 2])

                else:
                    if state.r_vertical != RobotVertical.NO:
                        if state.r_vertical == RobotVertical.lift:
                            instructions_array[i].append([machine_id, 'L', 0])
                        elif state.r_vertical == RobotVertical.l1:
                            instructions_array[i].append([machine_id, 'L', 1])
                        elif state.r_vertical == RobotVertical.l2:
                            instructions_array[i].append([machine_id, 'L', 2])
                        elif state.r_vertical == RobotVertical.l3:
                            instructions_array[i].append([machine_id, 'L', 3])
                        elif state.r_vertical == RobotVertical.l4:
                            instructions_array[i].append([machine_id, 'L', 4])
                        elif state.r_vertical == RobotVertical.drop:
                            instructions_array[i].append([machine_id, 'D', 0])
            else:
                if state.r_move != RobotMove.NO:
                    if (state.r_move == RobotMove.w0_mvN3 or
                            state.r_move == RobotMove.w1_mvN3 or
                            state.r_move == RobotMove.w2_mvN3):
                            y += 1
                    elif (state.r_move == RobotMove.w0_mvS3 or
                            state.r_move == RobotMove.w1_mvS3 or
                            state.r_move == RobotMove.w2_mvS3):
                            y -= 1
                    elif (state.r_move == RobotMove.w0_mvE1 or
                            state.r_move == RobotMove.w1_mvE1 or
                            state.r_move == RobotMove.w2_mvE1):
                            x += 1
                    elif (state.r_move == RobotMove.w0_mvW1 or
                            state.r_move == RobotMove.w1_mvW1 or
                            state.r_move == RobotMove.w2_mvW1):
                            x -= 1

                    if (state.r_move == RobotMove.w0_mvN0 or
                            state.r_move == RobotMove.w0_mvN1 or
                            state.r_move == RobotMove.w0_mvN2 or
                            state.r_move == RobotMove.w0_mvN3 or
                            state.r_move == RobotMove.w1_mvN0 or
                            state.r_move == RobotMove.w1_mvN1 or
                            state.r_move == RobotMove.w1_mvN2 or
                            state.r_move == RobotMove.w1_mvN3 or
                            state.r_move == RobotMove.w2_mvN0 or
                            state.r_move == RobotMove.w2_mvN1 or
                            state.r_move == RobotMove.w2_mvN2 or
                            state.r_move == RobotMove.w2_mvN3 or
                            state.r_move == RobotMove.w0_accN or
                            state.r_move == RobotMove.w1_accN or
                            state.r_move == RobotMove.w1_accN):
                            instructions_array[i].append([machine_id, 'S', 4])
                    elif (state.r_move == RobotMove.w0_mvS0 or
                            state.r_move == RobotMove.w0_mvS1 or
                            state.r_move == RobotMove.w0_mvS2 or
                            state.r_move == RobotMove.w0_mvS3 or
                            state.r_move == RobotMove.w1_mvS0 or
                            state.r_move == RobotMove.w1_mvS1 or
                            state.r_move == RobotMove.w1_mvS2 or
                            state.r_move == RobotMove.w1_mvS3 or
                            state.r_move == RobotMove.w2_mvS0 or
                            state.r_move == RobotMove.w2_mvS1 or
                            state.r_move == RobotMove.w2_mvS2 or
                            state.r_move == RobotMove.w2_mvS3 or
                            state.r_move == RobotMove.w0_accS or
                            state.r_move == RobotMove.w1_accS or
                            state.r_move == RobotMove.w1_accS):
                            instructions_array[i].append([machine_id, 'N', 4])
                    elif (state.r_move == RobotMove.w0_accE or
                            state.r_move == RobotMove.w0_mvE0 or
                            state.r_move == RobotMove.w0_mvE1 or
                            state.r_move == RobotMove.w1_accE or
                            state.r_move == RobotMove.w1_mvE0 or
                            state.r_move == RobotMove.w1_mvE1 or
                            state.r_move == RobotMove.w2_accE or
                            state.r_move == RobotMove.w2_mvE0 or
                            state.r_move == RobotMove.w2_mvE1):
                            instructions_array[i].append([machine_id, 'E', 2])
                    elif (state.r_move == RobotMove.w0_accW or
                            state.r_move == RobotMove.w0_mvW0 or
                            state.r_move == RobotMove.w0_mvW1 or
                            state.r_move == RobotMove.w1_accW or
                            state.r_move == RobotMove.w1_mvW0 or
                            state.r_move == RobotMove.w1_mvW1 or
                            state.r_move == RobotMove.w2_accW or
                            state.r_move == RobotMove.w2_mvW0 or
                            state.r_move == RobotMove.w2_mvW1):
                            instructions_array[i].append([machine_id, 'W', 2])

                else:
                    if state.r_vertical != RobotVertical.NO:
                        if state.r_vertical == RobotVertical.lift:
                            instructions_array[i].append([machine_id, 'L', 0])
                        elif state.r_vertical == RobotVertical.l1:
                            instructions_array[i].append([machine_id, 'L', 1])
                        elif state.r_vertical == RobotVertical.l2:
                            instructions_array[i].append([machine_id, 'L', 2])
                        elif state.r_vertical == RobotVertical.l3:
                            instructions_array[i].append([machine_id, 'L', 3])
                        elif state.r_vertical == RobotVertical.l4:
                            instructions_array[i].append([machine_id, 'L', 4])
                        elif state.r_vertical == RobotVertical.drop:
                            instructions_array[i].append([machine_id, 'D', 0])

            state_holder = get_machine_state(next_state, y, x, machine_type)
            if state_holder is not None:
                state = state_holder
            else:
                print("!!!No state found!!!")

    return json.dumps(instructions_array, separators=(',', ':'))


#  Will return the state of the machine that is on grid coordinates x,y. Helper function for getCoordinates().
def get_machine_state(machines, y, x, machine_type):
    for machine in machines:
        if machine[1] == y and machine[2] == x:
            if machine_type == 'C' and machine[0] != 'R':
                return machine[4]
            elif machine_type == 'R' and machine[0] == 'R':
                return machine[4]


# Helper function for debugging, will return human readable instructions
def print_grid(route, level):
    instructions = route.instructions_array[level]
    parking_lot = ""
    for i in range(0, route.lot_size[0]):
        instr = ""
        for j in range(0, route.lot_size[1]):
            instr += "|"
            instr += instructions[i * 10 + j].on_node.name
            instr += "|"
            instr += instructions[i * 10 + j].nd_stat.name
            instr += "|"
            instr += instructions[i * 10 + j].r_vertical.name
            instr += "|"
            instr += instructions[i * 10 + j].r_move.name
            instr += "\t"
        instr += "\n"
        parking_lot += instr
    parking_lot += "\n\n"
    return parking_lot


# writes abovementioned instructions to a file
def write_instructions(route):
    f = open('instructions.txt', 'w')
    for i in range(0, len(route.instructions_array)):
        f.write(print_grid(route, i))
    f.close()

from enum import Enum, unique

@unique
class State(Enum):
    idle = 0
    start = 1
    running = 2
    finished = 3
    failed = 4
    receiveOrder = 5
    navigate = 6
    approach = 7
    recognize = 8
    grasp = 9
    drop = 10 


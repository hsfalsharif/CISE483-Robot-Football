class Node:
    visited = False

    def __init__(self, position, neighbours):
        self.position = position
        self.neighbours = neighbours

    def to_string(self):
        return "Position :({0} , {1})\nNeighbours: {2}".format(self.position.y, self.position.x, self.neighbours)

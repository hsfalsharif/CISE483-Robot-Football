class Node:
    visited = False

    def __init__(self, position, parent):
        self.position = position
        self.parent = parent

    def to_string(self):
        return "Position :({0} , {1})".format(self.position.x, self.position.y)

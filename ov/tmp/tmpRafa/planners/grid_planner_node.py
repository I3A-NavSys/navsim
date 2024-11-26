class GridPlannerNode:
    def __init__(self, i, j, L, s, cost, parent):
        self.i = i
        self.j = j
        self.L = L
        self.s = s
        self.cost = cost
        self.parent = parent
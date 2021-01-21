#!usr/bin/evn python

import pygame as pg
import os
from queue import PriorityQueue
import math
pg.init()

CLOSED_TYPES = dict(START = "green", 
                GOAL = "red", 
                BARRIER = "#5C75D3",
                VISITED = "#C6394C")
OPEN_TYPES = dict(NEUTRAL = "black",
                PENDING = "#40B6A6",
                PATH = "gold")

SCR_WIDTH, SCR_HEIGHT = 900, 900
TOTAL_ROWS, TOTAL_COLS = 50, 50
LINE_COLOR = "#3654C6"

win = pg.display.set_mode((SCR_WIDTH, SCR_HEIGHT)) #creating windows with sepcified dimensions
pg.display.set_caption("A* Algorithm")

class Rect:
    def __init__(self, row, col, dim):
        self.row = row
        self.col = col
        self.x = (row * dim)
        self.y = (col * dim)
        self.dim = dim
        self.type = OPEN_TYPES["NEUTRAL"]

    def get_cord(self):
        return self.x, self.y
    
    def get_rowcol(self):
        return self.row, self.col
    
    def get_row(self):
        return self.row
    
    def get_col(self):
        return self.col
    
    def get_dim(self):
        return self.dim, self.dim
    
    def set_type(self, color):
        self.type = color
        
    def get_type(self):
        return self.type
   
    def get_neighbors(self, grid):
        row, col = self.get_rowcol()
        neighbors = []
        neighbors.append(grid[row-1][col]) if row-1 >= 0 else neighbors.append(None)
        neighbors.append(grid[row+1][col]) if row+1 < TOTAL_ROWS else neighbors.append(None)
        neighbors.append(grid[row][col-1]) if col-1 >= 0 else neighbors.append(None)
        neighbors.append(grid[row][col+1]) if col+1 < TOTAL_COLS else neighbors.append(None)
        return neighbors 
    
    def __lt__(self, other): #overriding < operator for object
	    return False
        
def create_grid(rows, cols, rect_dim):
    grid = []
    for i in range(rows):
        grid.append([]) #adding empty list in each row (to create 2d list)        
        for j in range(cols):
            r = Rect(i, j, rect_dim) #initializing Rect object                
            grid[i].append(r) #adding rectangle object to grid
    return grid

def draw_grid(win, grid, rect_dim):
    for row in grid:
        for elem in row:
            pg.draw.rect(win, elem.get_type(), (elem.get_cord(), elem.get_dim())) #drawing grid
            
    for r in range(TOTAL_ROWS):
        pg.draw.line(win, LINE_COLOR, (0, r * rect_dim), (SCR_WIDTH, r * rect_dim)) #drawing horizontal lines on grid
        for c in range(TOTAL_COLS):
            pg.draw.line(win, LINE_COLOR, (c * rect_dim, 0), (c * rect_dim, SCR_HEIGHT)) #drawing vertical lines on grid
            
    pg.draw.line(win, LINE_COLOR, (SCR_WIDTH, 0), (SCR_WIDTH, SCR_HEIGHT), 3) #filling last vertical line
    pg.display.update()
        
def get_mouse_pos(x, y, rect_dim):
    row = x // rect_dim #converting mouse position to coordinate pairs in list
    col = y // rect_dim 
    return row, col

def build_path(update_grid, start, goal, grid, predecessor):
    node = ""
    goal.set_type(CLOSED_TYPES["GOAL"])
    while node != start:
        node = predecessor[goal]
        if node != start: node.set_type(OPEN_TYPES["PATH"])
        update_grid()
        goal = node

def dist(p1, p2): #using manhattan distance formula since only can move four directions
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def clear_grid(grid): 
    for row in grid:
        for elem in row:
            elem.set_type(OPEN_TYPES["NEUTRAL"])

def blink(elem, time):
    current_time = pg.time.get_ticks() #getting current time (from start of program)
    if current_time >= time["change_time"]: #checking if time to update change_time
        time["change_time"] = current_time + time["delay"]
        time["switch"] = (time["switch"] + 1) % 2 #switching b/w 0 and 1 each time called
                
    if time["switch"] == 1:
        elem.set_type(CLOSED_TYPES["GOAL"])
    if time["switch"] == 0:
        elem.set_type(OPEN_TYPES["NEUTRAL"])
        
def astar_alg(update_grid, start, goal, grid):   
    blink_timing = {"change_time": 0, #dictionary that holds values to determine blinking freq for goal node
            "delay": 500,
            "switch": 1}

    unvisited = PriorityQueue() #nodes that still need to be evaluated
    pred = {} #keeping track of predecessor nodes
    g_cost = {}
    time_stamp = 0 #a tie breaker for nodes that have same f_score in priority queue
    
    g_cost = {elem: float("inf") for row in grid for elem in row} #float("inf") == infinity
    g_cost[start] = 0
    goal_found = False
    
    unvisited.put((0, time_stamp, start)) #putting starting node in queue
    
    while not unvisited.empty() and not goal_found:      
        for event in pg.event.get(): #giving the user the ability to exit during algorithm
            if event.type == pg.QUIT:
                pg.quit()
                
        _, _, shortest_dist_node = unvisited.get()
        if shortest_dist_node != start:
            shortest_dist_node.set_type(CLOSED_TYPES["VISITED"]) #marking node as visited
            
        path_options = shortest_dist_node.get_neighbors(grid) #getting neighbors from current node
        
        for child_node in path_options:
            if child_node == goal:
                pred[child_node] = shortest_dist_node
                build_path(update_grid, start, goal, grid, pred)
                return
                    
            if child_node is not None and child_node.get_type() != CLOSED_TYPES["BARRIER"]: #checking that neighbor is not a barrrier or out of bounds
                temp_g = g_cost[shortest_dist_node] + 1 #distance between each node will always be 1
                
                if temp_g < g_cost[child_node]:#if sdn g cost + dist from child to sdn node is < g cost of child node then shorter path is from sdn to child node
                    g_cost[child_node] = temp_g #child node g_cost is updated
                    h = dist(child_node.get_rowcol(), goal.get_rowcol()) #estimated distance from child node to goal node
                    f_cost = temp_g + h #calculating f_cost
                    child_node.set_type(OPEN_TYPES["PENDING"])
                    
                    time_stamp += 1
                    
                    unvisited.put((f_cost, time_stamp, child_node)) #putting child_node in queue
                    pred[child_node] = shortest_dist_node #storing predecessor node of child node
                    
                    blink(goal, blink_timing) #making goal node blink
                    update_grid()
                                                    
def main():    
    rect_dim = SCR_WIDTH // TOTAL_ROWS #gives us dimensions of each rectangle
    grid = create_grid(TOTAL_ROWS, TOTAL_COLS, rect_dim) #initializing grid
    start = goal = None
    run = True
    
    while run:
        win.fill("WHITE")
        draw_grid(win, grid, rect_dim)
        
        for event in pg.event.get():
            if event.type == pg.QUIT:
                run = False

            left, _, right = pg.mouse.get_pressed()
            if left or right: #checking if left button on mouse was clicked
                x, y = pg.mouse.get_pos() #returning x and y coord of mouse cursor                
                row, col = get_mouse_pos(x, y, rect_dim) #converting x and y coord to row and col values in grid
                elem = grid[row][col]
                if left:
                    if elem.get_type() not in list(CLOSED_TYPES.values())[:2]: #checking that element is not start, goal, or barrier type
                        if start is None:
                            start = elem
                            start.set_type(CLOSED_TYPES["START"])
                        elif goal is None: 
                            goal = elem
                            goal.set_type(CLOSED_TYPES["GOAL"])
                        else:
                            elem.set_type(CLOSED_TYPES["BARRIER"]) 
                if right:
                    if elem == start:
                        start = None
                    elif elem == goal:
                        goal = None
                    if elem.get_type() != OPEN_TYPES["NEUTRAL"]:
                        elem.set_type(OPEN_TYPES["NEUTRAL"])
        
            if event.type == pg.KEYDOWN and event.key == pg.K_SPACE: #keys[pg.K_SPACE]: #start astar_alg when space bar pressed                
                if start is not None and goal is not None:
                    astar_alg(lambda: draw_grid(win, grid, rect_dim) , start, goal, grid)
                
            if event.type == pg.KEYDOWN and event.key == pg.K_c: #clearing and resetting grid
                clear_grid(grid)
                start = None
                goal = None
    pg.quit()
main()
    
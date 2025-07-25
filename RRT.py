import pygame
import sys
from dataclasses import dataclass
import random

@dataclass
class Node:
    no: int
    parent: int
    x: int
    y: int


class ArenaVisualizer:
    def __init__(self,obstacles_list:list,start:tuple,end:tuple, robot_size:tuple, end_tolerance:tuple, step_size:int):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("RRT Visualizer")

        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.GREY = (200, 200, 200)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 200, 0)
        self.BLUE = (0, 0, 255)
        self.ORANGE = (255,255,0)

        # Arena elements
        self.obstacles = []
        for i in obstacles_list:
            r=pygame.Rect(0,0,i[2]*SCALE,i[3]*SCALE)
            r.center=(i[0]*SCALE,i[1]*SCALE)
            self.obstacles.append(r)

        self.start_pos = start
        self.end_pos = end
        self.robot_size=robot_size
        self.end_tolerance=end_tolerance
        self.step_size=step_size

        self.lines=[]

        self.clock = pygame.time.Clock()
        self.running = True
        self.path_found = False

    def draw_obstacles(self):
        for rect in self.obstacles:
            pygame.draw.rect(self.screen, self.GREY, rect)

    def draw_points(self):
        self.start_rec=pygame.Rect(0,0,self.robot_size[0],self.robot_size[1])
        self.start_rec.center=(self.start_pos[0],self.start_pos[1])

        self.end_rec=pygame.Rect(0,0,self.end_tolerance[0],self.end_tolerance[1])
        self.end_rec.center=(self.end_pos[0],self.end_pos[1])

        self.curr_rec=self.start_rec.copy()

        pygame.draw.rect(self.screen, self.RED, self.start_rec)
        pygame.draw.rect(self.screen, self.GREEN, self.end_rec)
        pygame.draw.rect(self.screen, self.BLUE, self.curr_rec)

        for (start, end) in self.lines:
            pygame.draw.line(self.screen, self.BLACK, start, end, 2)
    
    def check_collisions(self,x0,y0,x1,y1)->bool:
        self.curr_rec.center=(x1,y1)
        for i in self.obstacles:
            if(i.clipline((x0,y0),(x1,y1)) or i.colliderect(self.curr_rec)):
                return True
        
        return False
    
    def check_end(self)->bool:
        if self.curr_rec.colliderect(self.end_rec):
            return True
        return False


    def run(self):
        while self.running:
            self.clock.tick(60)
            self.screen.fill(self.WHITE)

            self.handle_events()
            self.draw_obstacles()
            self.draw_points()

            if not self.path_found:
                (next_x,next_y)=rrt.generate_point()
                nearest=rrt.get_nearest_node(next_x,next_y)
                near_x=nearest.x
                near_y=nearest.y
                near_parent=nearest.parent

                dx = next_x - near_x
                dy = next_y - near_y

                if dx == 0:
                    # Vertical line
                    step_y = self.step_size if dy > 0 else -self.step_size
                    for y in range(near_y + step_y, next_y, step_y):
                        if self.check_collisions(near_x, near_y, near_x, y):
                            continue
                        else:
                            rrt.add_node(near_parent, near_x, y)
                            near_parent = rrt.node_no
                            self.lines.append(((near_x, near_y), (near_x, y)))
                            if self.check_end():
                                print("Solved")
                                self.path_found=True
                else:
                    # Regular slope
                    slope = dy / dx
                    step_x = self.step_size if dx > 0 else -self.step_size
                    for x in range(near_x + step_x, next_x, step_x):
                        y = int(near_y + slope * (x - near_x))
                        if self.check_collisions(near_x, near_y, x, y):
                            continue
                        else:
                            rrt.add_node(near_parent, x, y)
                            near_parent = rrt.node_no
                            self.lines.append(((near_x, near_y), (x, y)))
                            if self.check_end():
                                print("Solved")
                                self.path_found=True
                            
                        
            if self.path_found:
                self.end_things()
                self.running=False

            pygame.display.flip()

        # pygame.quit()
        # sys.exit()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
    
    def end_things(self):
        print("Visaualizing")
        path = []
        node_lookup = {node.no: node for node in rrt.tree}
        current_id = rrt.node_no

        while current_id != 0:
            node = node_lookup[current_id]
            parent = node_lookup[node.parent]
            start=(node.x, node.y)
            end=(parent.x, parent.y)
            pygame.draw.line(self.screen, self.ORANGE, start, end, 2)
            path.append(((start[0]/SCALE,start[1]/SCALE), (end[0]/SCALE,end[1]/SCALE)))
            current_id = node.parent
        
        global PATH_ANS
        PATH_ANS=path
        
        print(f"Path Len: {len(path)}")
        print(f"Total Nodes: {rrt.node_no}")
            

        return path  # from goal to start

class RRT:
    def __init__(self,start:tuple,end:tuple,robot_size:tuple, end_tolerance:tuple, step_size:int):
        self.start=start
        self.end=end
        self.robot_size=robot_size
        self.end_tolerance=end_tolerance
        self.step_size=step_size
        self.node_no=0
        self.tree=[Node(0,0,start[0],start[1])]

    def generate_point(self):
        x=random.randint(0,WIDTH)
        y=random.randint(0,HEIGHT)
        return (x,y)

    def get_nearest_node(self,x,y):
        min_dist=float('inf')
        min_tree=None

        for i in self.tree:
            dist=((i.x-x)**2+(i.y-y)**2)**0.5
            if dist<min_dist:
                min_dist=dist
                min_tree=i
        
        return min_tree
    
    def add_node(self,parent:int,x:int,y:int):
        self.node_no+=1
        new_node=Node(self.node_no,parent,x,y)
        self.tree.append(new_node)
    
    
def RRT_INIT(end_val):
    end=(end_val[0]*SCALE+WIDTH/2,end_val[1]*SCALE)
    global arena
    arena = ArenaVisualizer(obstacles_list,start,end,robot_size,end_tolerance,step_size)

    global rrt
    rrt=RRT(start,end,robot_size,end_tolerance,step_size)

    arena.run()

    return PATH_ANS

SCALE=15
WIDTH=16 * SCALE
HEIGHT=55 * SCALE

PATH_ANS=[]

start=(8*SCALE,1*SCALE)
end=(8*SCALE,50*SCALE)
robot_size=(3*SCALE,3*SCALE)
end_tolerance=(0.5*SCALE,0.5*SCALE)
step_size=2
obstacles_list=[
        (10,10,4,4),
        (5,25,4,4),
        (12,40,4,4)
    ]

arena=None
rrt=None
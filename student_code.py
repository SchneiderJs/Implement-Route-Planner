from queue import PriorityQueue
class A_Star(object):
    def __init__(self, M):
        self.M = M    
        self.came_from = {}
        self.explored  = set()
        self.frontier  = PriorityQueue()
        self.g_scores  = [99999999 for _ in range(len(M.roads))]

    def __reconstruct_path(self, current, start):
        """
        Returns the path from start to current        
        """
        path = []
        while current != start:
            path.insert(0, current)
            current = self.came_from[current]
        path.insert(0, start)
        
        return path
    
    def __expand_node(self, current, goal):
        """
        Explores the neighbors nodes of the current node
        """
        neighbors = self.M.roads[current]
        for neighbor in neighbors:
            if neighbor in self.explored:
                continue
                
            tentative_g_score = self.g_scores[current] + self.euclidean_distance(current, neighbor)
            f_score = self.g_scores[neighbor] + self.euclidean_distance(neighbor, goal)
            if neighbor not in self.frontier.queue:
                self.frontier.put((f_score, neighbor))
              
            if tentative_g_score >= self.g_scores[neighbor]:
                continue
                
            self.came_from[neighbor] = current
            self.g_scores[neighbor] = tentative_g_score
            self.frontier.put((f_score, neighbor))
            
    def euclidean_distance(self, nodeA, nodeB):
        """
        Calculates the Euclidean distance between two nodes
        """
        pointA = self.M.intersections[nodeA]
        pointB = self.M.intersections[nodeB]
    
        a = pointB[0] - pointA[0]
        b = pointB[1] - pointA[1]
        h = (a**2 + b**2)**0.5
    
        return h

    def search(self, start, goal):
        """
        Search the shortest path from start to goal
        """
        if self.came_from:
            self.__init__(self.M)
            
        self.g_scores[start] = 0.0
        f_score = self.euclidean_distance(start, goal)
        self.frontier.put((f_score, start)) 
        
        while self.frontier.queue:
            current = self.frontier.get()[1]  # best node to go ahead
            if current == goal:
                return self.__reconstruct_path(current, start)
            
            self.explored.add(current)
            self.__expand_node(current, goal)
        
        return None
        
def shortest_path(M, start, goal):
    print("shortest path called")
    """
    Returns the shortest path from start to goal
    """
    
    Astar = A_Star(M)
    path = Astar.search(start, goal)
    
    return path
    
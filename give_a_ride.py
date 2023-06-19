from math import inf


class Vertex:
    def __init__(self, id):
        """
        Code inspired by Lecture Materials from Monash FIT2004

        Function description: This function is a constructor for the Vertex class 

        Input:
            id: the id representing the Vertex

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        # stores the list of Edges of a Vertex
        self.edges = []

        # id that identifies a Vertex
        self.id = id

        # to determine if a Vertex has been discovered during traversal of graphs
        self.discovered = False

        # to determine if a Vertex has been visited during traversal of graphs
        self.visited = False

        # stores the distance of a Vertex (depending on scenario)
        self.distance = 0

        # stores the index of the Vertex in the heap array
        self.position = None

        # to determine if a passenger exists in a Vertex (location)
        self.has_passenger = False

        # to determine which is the previous vertex of the Vertex
        self.previous = None

    def add_edge(self, edge) -> None:
        """
        Function description: This function adds an Edge into the edges list of a Vertex 

        Input:
            edge: Edge, representing an edge in a graph 

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        self.edges.append(edge)

    def reset(self):
        """
        Function description: This function resets some of the attributes's value of the Vertex

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        self.discovered = False
        self.visited = False
        self.distance = 0
        self.position = None
        self.previous = None


class Edge:
    def __init__(self, a, b, c, d):
        """
        Code inspired by Lecture Materials from Monash FIT2004

        Function description: This function is a constructor for the Edge class 

        Input:
            a: starting location of a road
            b: ending location of a road
            c: minutes needed to travel from a to b, if there is only 1 person in the car
            d: minutes needed to travel from a to b, if there are 2 or more person in the car

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        self.a = a
        self.b = b
        self.c = c
        self.d = d


class Graph:
    def __init__(self, roads, carries_passenger=False):
        """
        Codes are inspired by Lecture Materials from Monash FIT2004

        Constructor for the Graph Class

        V = number of vertices in the graph 
        E = number of edges in the graph 

        Time Complexity: O(V + E)
        Aux Space Complexity: O(V + E)
        """

        self.roads = roads
        vertices_count = 0

        # O(E)
        for tup in self.roads:
            cur_max = max(tup[0], tup[1])
            vertices_count = max(cur_max, vertices_count)

        vertices_count += 1

        # O(V)
        self.vertices = [None] * vertices_count
        for i in range(vertices_count):
            self.vertices[i] = Vertex(i)
            if carries_passenger:
                self.vertices[i].has_passenger = True

        # O(E)
        for tup in self.roads:
            a = self.vertices[tup[0]]
            b = self.vertices[tup[1]]
            c = tup[2]
            d = tup[3]

            current_edge = Edge(a, b, c, d)
            a.add_edge(current_edge)

    def reset(self):
        """
        Function description: This function resets the graph

        Approach description: Iterates through every vertices in the graph and calling its reset function

        Time complexity: O(V), where V is the number of vertices in the graph
        Aux space complexity: O(1)
        """
        for vertex in self.vertices:
            vertex.reset()
        return


class Heap:
    """
    Source Code: Monash FIT 1008
    Taken and Modified Implementation of a Min Heap from Monash FIT 1008
    """
    MIN_CAPACITY = 1

    def __init__(self, argv_vertex_count):
        """
        Function description: This function is the constructor for the Heap class

        Approach description: Initialises an array to represent a Min Heap

        Input:
            argv_vertex_count: the number of vertices in the graph 

        Time complexity: O(V), where V is the number of elements in the Heap
        Aux space complexity: O(V), where V is the number of elements in the Heap
        """
        size = max((argv_vertex_count + 1), self.MIN_CAPACITY)
        self.heap = [None] * size
        self.length = 0

    def __len__(self):
        """
        Function description: This function returns the number of elements in the Heap   

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        return self.length

    def is_full(self):
        """
        Function description: This function returns True if the Heap is full, otherwise False   

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        return self.length + 1 == len(self.heap)

    def is_empty(self):
        """
        Function description: This function returns True if the Heap is empty, otherwise False   

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        return self.length == 0

    def insert(self, element):
        """
        Function description: This function inserts an element into the Heap and returns the index of the element inserted in the array

        Approach description: Insert the element in the end of the array, then perform a rise operation to its proper position in the Heap

        Input:
            element: the Vertex to be inserted into the Heap

        Return:
            idx: the inserted element's index in the array

        Time complexity: O(Log V), where V is the number of elements in the Heap
        Aux space complexity: O(1)
        """
        if not self.is_full():
            self.length += 1
            self.heap[self.length] = element
            # initialise the position attribute of the Vertex
            element.position = self.length
            idx = self.rise(self.length)
            # set the index of the element in the array to the position attribute
            element.position = idx
            return idx

    def rise(self, k):
        """
        Function description: This function rise an element in the Heap and returns the index of the element after rise in the array

        Approach description: Perform a swap with the parent element while the element's value is smaller than the parent's value

        Input:
            k: the index of the element to be called rise

        Return:
            k: the index of the element in the array after rise 

        Time complexity: O(Log V), where V is the number of elements in the Heap
        Aux space complexity: O(1)
        """
        while k > 1 and (self.heap[k//2].distance > self.heap[k].distance):
            self.swap(k, k//2)
            k = k // 2
        return k

    def sink(self, k):
        """
        Function description: This function sinks an element in the Heap and returns the index of the element after sink in the array

        Approach description: Perform a swap with the child element while the element's value is larger than the child's value

        Input:
            k: the index of the element to be called rise

        Return: 
            k: the index in the heap of the element sinked

        Time complexity: O(Log V), where V is the number of elements in the Heap
        Aux space complexity: O(1)
        """
        while 2 * k <= self.length:
            child = self.smallest_child(k)

            # current <= child need no swapping
            if self.heap[k].distance <= self.heap[child].distance:
                break

            self.swap(k, child)
            k = child
        return k

    def smallest_child(self, k):
        """
        Function description: This function finds the smaller child element of the element at index k in the array and returns the index of the smaller child element 

        Approach description: Compares the value of the left and right child's element value

        Input:
            k: the index of the element to be called rise 

        Return:
            the index of the smaller child element 

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        l_idx = 2 * k
        r_idx = 2 * k + 1

        # right child does not exist or left child < right child
        if 2 * k == self.length or (self.heap[l_idx].distance < self.heap[r_idx].distance):
            return l_idx

        # right child <= left child
        else:
            return r_idx

    def serve(self) -> Vertex:
        """
        Function description: This function returns the element at the root of the Heap which has the minimum value in the Heap

        Approach description: Perform a swap between the root element and the last added element in the Heap, then sink the element at the root position 

        Time complexity: O(Log V), where V is the number of elements in the Heap
        Aux space complexity: O(1)
        """
        self.swap(1, self.length)
        item = self.heap[self.length]
        self.heap[self.length] = None
        self.length -= 1
        self.sink(1)
        return item

    def swap(self, index_1, index_2) -> None:
        """
        Function description: This function swaps the element in the Heap based on the two given index 

        Approach description: Perform a swap between element at index_1 and index_2

        Input:
            index_1: the index of the element to be swapped 
            index_2: the index of the element to be swapped 

        Time complexity: O(1)
        Aux space complexity: O(1)
        """
        self.heap[index_1], self.heap[index_2] = self.heap[index_2], self.heap[index_1]

        # swaps also the position attribute of the Vertex
        vertex_1 = self.heap[index_1]
        vertex_2 = self.heap[index_2]
        vertex_1.position, vertex_2.position = vertex_2.position, vertex_1.position


def optimalRoute(start, end, passengers, roads):
    """
    Written By: Chan Yanhan
    Inspired by Lecture Materials from Monash FIT2004 

    Function description:
    This function finds the shortest amount of driving time needed to travel from one location to the another while the carpool lane which can be used if there are more than 1 passenger in the car can reduce the driving time, and hence are also taken into consideration by this function. It returns the optimal route as a list of integers where each integers represent the location. 

    Approach description:
    We can model the scenario using a directed weighted graph data structure where each location represents a vertex and each road that connects the locations represents an edge. Then there are two weights for the edge, where in this case would be the driving time using the carpool and non carpool lane. Then we perform a Dijkstra algorithm on the graph to find the shortest driving time.

    Before we run Dijkstra, we first need to initialise two Graph where only the non carpool lane could be used in the first graph, and a carpool or non carpool lane can be used in the second graph. The two graphs have the same number of locations and edges connecting the vertices, the only difference is that the option to use the carpool lane is available if we are at the second graph. This approach works because the only way/edge that connects the two graph is the vertices that contains a passenger, hence doing so ensures that we have a passenger picked up in the car, hence we can use the carpool lane. This can be simply achieved by adding an edge from the vertices that contains passenger in graph 1, to the same vertex in graph 2. Then, we perform a Dijkstra on the graph, and then we could simply perform a backtrack to trace the route taken when we reach the end vertex.

    Input:
        start:      the int id representing the starting location
        end:        the int id representing the ending location
        passengers: a list containing the id of locations that has a passenger
        roads:      a list of tuples containing the details of the road (a, b, c, d)
                        a: starting location of the road
                        b: ending location of the road 
                        c: the driving time from a to b when only 1 person is present in the car 
                        d: the driving time from a to b when only 2 or more person are present in the car

    Return:
        a list of integers where each integer represents the location.

    L = number of locations (vertices)
    R = number of roads (edges)
    P = number of passengers

    Time complexity:
        Worst Case: O(R*Log L)

            Derivation of the complexity
            - Initialisation of two Graphs, O(2R + 2L)
            - Connecting the two Graph, O(P)
            - Create a Heap, O(L)
            - Main while loop , O(L * (Log L + L * Log L)) = O(R*LogL)

            Total Time Complexity:
                2R + 2L + P + L + R*LogL, since R >= L and P < L, it can be simplified to O(R*LogL).

    Space Complexity:
        Input:  O(R)
        Aux:    O(R + L)

        Derivation of the complexity:
            O(L) for the heap and O(R) for the return list
    """
    # represents the graph when only 1 person is in the car
    graph_1 = Graph(roads)
    # represents the graph when 2 or more person is in the car
    graph_2 = Graph(roads, True)

    # iterate through the list of locations that contains passengers, O(P)
    for p in passengers:
        v1 = graph_1.vertices[p]
        v2 = graph_2.vertices[p]
        v2.has_passenger = True
        # initialise an edge with driving time of -inf so that it would be first served in the heap
        e = Edge(v1, v2, -inf, -inf)
        # add an edge from the vertex in graph 1 to the vertex in graph 2
        v1.add_edge(e)

    # insert the source vertex into the min heap
    source = graph_1.vertices[start]
    source.visited = True
    source.discovered = True
    discovered = Heap(2*len(graph_1.vertices))
    discovered.insert(source)
    vertex_u = None

    while not discovered.is_empty():
        vertex_u = discovered.serve()
        vertex_u.visited = True

        if vertex_u.id == end:
            break

        # to ensure the correct distance is carried over to the second graph
        if vertex_u.id != start and vertex_u.previous.id == vertex_u.id:
            vertex_u.distance = vertex_u.previous.distance

        # edge relaxation by iterating through every road (edges)
        for edge in vertex_u.edges:
            vertex_v = edge.b

            # location (vertex) has not been discovered
            if not vertex_v.discovered:
                vertex_v.discovered = True

                # update the driving time of the vertex depending if the car has more than 1 passengers or not, O(1)
                if not vertex_u.has_passenger:
                    vertex_v.distance = vertex_u.distance + edge.c
                else:
                    vertex_v.distance = vertex_u.distance + edge.d

                vertex_v.previous = vertex_u
                discovered.insert(vertex_v)

            # location(vertex) has been discovered, but not visited
            elif not vertex_v.visited:

                # only 1 passenger in the car
                if not vertex_u.has_passenger:
                    # update the driving time for only non car pool lanes
                    if vertex_v.distance > vertex_u.distance + edge.c:
                        vertex_v.distance = vertex_u.distance + edge.c
                        vertex_v.previous = vertex_u
                        discovered.rise(vertex_v.position)

                # more than 1 passenger in the car
                # update the driving time when carpool lane can be used
                elif vertex_v.distance > vertex_u.distance + edge.d:
                    vertex_v.distance = vertex_u.distance + edge.d
                    vertex_v.previous = vertex_u
                    discovered.rise(vertex_v.position)

    # backtracking and add to return list
    path = []
    if vertex_u.visited:
        while vertex_u != source:

            # checks that return list does not contain the duplicated location
            if vertex_u.id != vertex_u.previous.id:
                path.append(vertex_u.id)
            vertex_u = vertex_u.previous
        path.append(source.id)

    # reverse the list
    n = len(path)
    for i in range(n // 2):
        path[i], path[n - i - 1] = path[n - i - 1], path[i]
    return path


if __name__ == "__main__":
    # Example
    start = 0
    end = 4
    # The locations where there are potential passengers
    passengers = [2, 1]
    # The roads represented as a list of tuple
    roads = [(0, 3, 5, 3), (3, 4, 35, 15), (3, 2, 2, 2), (4, 0, 15, 10),
             (2, 4, 30, 25), (2, 0, 2, 2), (0, 1, 10, 10), (1, 4, 30, 20)]

    # Returns the id of the vertex representing the location
    print(optimalRoute(start, end, passengers, roads))

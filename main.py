from math import sqrt


class City:
    """A node of a weighted graph of cities."""
    def __init__(self, name: str, latitude: float, longitude: float) -> None:
        self.name = name
        self.latitude = latitude
        self.longitude = longitude
        self.neighbours: list[City] = []

    def add_neighbours(self, cities: list["City"]) -> None:
        """Adds a list if roads (edges) to the city node."""
        self.neighbours = self.neighbours + cities

    def distance_to(self, other: "City") -> float:
        """Calculates the euclidian distance between this city and another."""
        return sqrt(((self.latitude - other.latitude) ** 2) + ((self.longitude - other.longitude) ** 2))


class Node:
    """A node of a stack used internally in the A* algorithm."""
    def __init__(self, parent: "Node | None", city: City) -> None:
        self.city = city
        self.parent = parent
        # Distance between this node and the start node
        self.g: float = 0
        # Estimated distance from this node to the goal node (heuristic)
        self.h: float = 0
        # Total cost of this node: f = g + h
        self.f: float = 0


def calc_heuristic(node: Node, goal_node: Node) -> float:
    """The heuristic function.\n
    Estimates the distance between a node and the goal."""
    return node.city.distance_to(goal_node.city)


def a_star(start: City, goal: City) -> list[City] | None:
    """Performs the A* pathfinding algorithm.\n"""
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    # A list of nodes that are candidates for further exploration
    open_list: list[Node] = [start_node]
    # A list of nodes that have already been explored
    closed_list: list[Node] = []
    while len(open_list) > 0:
        current_node = open_list[0]
        current_index = 0
        # Find the node with the lowest f (best node)
        for i, node in enumerate(open_list):
            if node.f < current_node.f:
                current_node = node
                current_index = i
        # Set the current node as explored
        open_list.pop(current_index)
        closed_list.append(current_node)
        # Check if the goal was reached
        if current_node.city == goal_node.city:
            # Traverse the nodes backwards until the start node is reached
            # while storing each city in a list
            path = []
            while current_node is not None:
                path.append(current_node.city)
                current_node = current_node.parent
            # Return the reversed path
            return path[::-1]
        # Loop through the current node's adiacent roads
        for neighbour_city in current_node.city.neighbours:
            # Skip if the neighbour city has already been explored
            in_closed = False
            for node in closed_list:
                if neighbour_city == node.city:
                    in_closed = True
                    break
            if in_closed:
                continue
            # Create a node from the neighbour and calculate its g, h and f
            neighbour_node = Node(current_node, neighbour_city)
            neighbour_node.g = current_node.g + current_node.city.distance_to(neighbour_city)
            neighbour_node.h = calc_heuristic(neighbour_node, goal_node)
            neighbour_node.f = neighbour_node.g + neighbour_node.h
            # If the neighbour isn't in the open list already, add it
            in_open = False
            for node in open_list:
                if neighbour_city == node.city:
                    in_open = True
                    # If the neighbour node is already in the open list, check if its current g
                    # is better than the previous one and, in that case, update its g with the
                    # new value and set its parent to the current node
                    if neighbour_node.g < node.g:
                        node.g = neighbour_node.g
                        node.parent = current_node
            if in_open:
                continue
            open_list.append(neighbour_node)
    return None


def main() -> None:
    # Create the cities
    aosta = City("Aosta", 45.735062, 7.313080)
    torino = City("Torino", 45.070339, 7.686864)
    bergamo = City("Bergamo", 45.698265, 9.677270)
    milano = City("Milano", 45.4641943, 9.1896346)
    alessandria = City("Alessandria", 44.8349533, 8.7450304)
    cuneo = City("Cuneo", 44.4580703, 7.5581367)
    genova = City("Genova", 44.40726, 8.9338624)
    parma = City("Parma", 44.8013678, 10.3280833)
    modena = City("Modena", 44.5384728, 10.9359609)
    brescia = City("Brescia", 45.7795804, 10.425873)
    trento = City("Trento", 46.1029536, 11.1297425)
    verona = City("Verona", 45.4384958, 10.9924122)
    padova = City("Padova", 45.4077172, 11.8734455)
    bolzano = City("Bolzano", 46.4981125, 11.3547801)
    ferrara = City("Ferrara", 44.7667642, 11.827939)
    venezia = City("Venezia", 45.4371908, 12.3345898)
    belluno = City("Belluno", 46.2805407, 12.0789137)
    trieste = City("Trieste", 45.6496485, 13.7772781)
    # Connect the cities
    aosta.add_neighbours([torino, alessandria, milano, bergamo])
    torino.add_neighbours([alessandria, genova, cuneo, aosta])
    bergamo.add_neighbours([milano, brescia, aosta, trento, bolzano])
    milano.add_neighbours([aosta, brescia, bergamo, alessandria, parma])
    alessandria.add_neighbours([aosta, torino, cuneo, genova, milano, brescia])
    cuneo.add_neighbours([torino, alessandria, genova])
    genova.add_neighbours([cuneo, torino, alessandria, parma, modena])
    parma.add_neighbours([modena, genova, alessandria, milano, brescia, verona])
    modena.add_neighbours([genova, parma, brescia, verona, ferrara])
    brescia.add_neighbours([milano, bergamo, verona, alessandria, parma, modena, trento])
    trento.add_neighbours([bolzano, belluno, padova, verona, brescia, bergamo])
    verona.add_neighbours([brescia, trento, belluno, padova, ferrara, modena, parma])
    padova.add_neighbours([venezia, ferrara, verona, trento, belluno])
    bolzano.add_neighbours([belluno, trento, bergamo])
    ferrara.add_neighbours([modena, verona, padova, venezia])
    venezia.add_neighbours([ferrara, padova, belluno])
    belluno.add_neighbours([trieste, venezia, padova, verona, trento, bolzano])
    trieste.add_neighbours([belluno])
    # Run the A* algorithm
    path = a_star(torino, belluno)
    # Exit if the algorithm failed
    if path is None:
        print("No roads found.")
        return
    # Map the list of city objects to a list of city names
    path = map(lambda city: city.name, path)
    # Nicely print the path
    print(" -> ".join(path))


if __name__ == "__main__":
    main()

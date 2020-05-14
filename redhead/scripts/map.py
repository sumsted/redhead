import pprint
from math import radians, atan, atan2, cos, sin, sqrt, acos

class Map:

    def __init__(self):
        self._links = {}
        self._links_map = {}
        self.load_map()

    def load_map(self):
        # self.add_link("a", "b", 5, 90)
        # self.add_link("b", "c", 5, 90)
        # self.add_link("a", "d", 4, 180)
        # self.add_link("b", "e", 4, 180)
        # self.add_link("c", "h", 10, 180)
        # self.add_link("d", "e", 5, 90)
        # self.add_link("e", "g", 6, 180)
        # self.add_link("d", "f", 6, 180)
        # self.add_link("f", "g", 5, 90)
        # self.add_link("f", "i", 8, 180)
        # self.add_link("g", "j", 8, 180)
        # self.add_link("g", "h", 5, 90)
        # self.add_link("h", "k", 6, 180)
        # self.add_link("i", "j", 5, 90)
        # self.add_link("j", "k", 6, 80)
        self.add_link("A", "B", 2,90)
        self.add_link("B","C",4, 90)		
        self.add_link("C", "D", 5,90)		
        self.add_link("D","E",3, 90)
        self.add_link("A", "F", 4,180)		
        self.add_link("B","G",3, 180)		
        self.add_link("C", "H", 3,180)		
        self.add_link("D","I",3, 180)		
        self.add_link("E", "J", 3, 180)
        self.add_link("F", "G", 2,90)		
        self.add_link("G","H",6, 90)		
        self.add_link("H", "I", 5,90)		
        self.add_link("I","J",1, 90)
        self.add_link("F", "K", 3,180)		
        self.add_link("G","L",5, 180)		
        self.add_link("H", "M", 2,180)		
        self.add_link("I","N",5, 180)		
        self.add_link("J", "O", 4, 180)
        self.add_link("K", "L", 4,90)		
        self.add_link("L","M",2, 90)		
        self.add_link("M", "N", 4,90)		
        self.add_link("N","O",2, 90)
        self.add_link("K", "P", 2,180)		
        self.add_link("L","Q",2, 180)		
        self.add_link("M", "R", 2,180)		
        self.add_link("N","S",4, 180)		
        self.add_link("O", "T", 5, 180)
        self.add_link("P", "Q", 5,90)		
        self.add_link("Q","R",4, 90)		
        self.add_link("R", "S", 2,90)		
        self.add_link("S","T",1, 90)
        self.add_link("P", "U", 3,180)		
        self.add_link("Q","V",4, 180)		
        self.add_link("R", "W", 1,180)		
        self.add_link("S","X",2, 180)		
        self.add_link("T", "Y", 4, 180)
        self.add_link("U", "V", 4,90)		
        self.add_link("V","W",2, 90)		
        self.add_link("W", "X", 3,90)		
        self.add_link("X","Y",3, 90)
        self.add_link("Y", "Z", 4, 180)



    def add_link(self, start_node, end_node, effort, direction=None):
        key = self.get_key(start_node, end_node)

        link = {
            "key": key,
            "start_node": start_node,
            "end_node": end_node,
            "pair": (start_node, end_node),
            "effort": effort,
            "direction": direction,
            "reverse": abs(180-direction) if direction >= 180 else 180 + direction
        }

        try:
            if key not in self._links_map[start_node]:
                self._links_map[start_node].append(key)
        except KeyError:
            self._links_map[start_node] = [key]

        try:
            if key not in self._links_map[end_node]:
                self._links_map[end_node].append(key)
        except KeyError:
            self._links_map[end_node] = [key]

        self._links[key] = link

        return key

    def get_key(self, start_node, end_node):
        return "%s_%s" % (start_node, end_node) if start_node < end_node else "%s_%s" % (
            end_node, start_node)
        
    def get_links(self, node):
        result = []
        try:
            result = [self._links[x] for x in self._links_map[node]]
        except KeyError:
            pass
        return result

    def get_neighbors(self, node):
        links = self.get_links(node)
        neighbors = []
        for link in links:
            neighbors.append( link["start_node"] if node == link["end_node"] else link["end_node"])
        return neighbors

    def get_graph(self, node):
        links = self.get_links(node)
        graph = {}
        for link in links:
            neighbor = link["start_node"] if node == link["end_node"] else link["end_node"]
            graph[neighbor] = link
        return graph

    def get_link(self, key):
        return self._links[key]

    def get_set(self):
        return self._links_map.keys()

    @staticmethod
    def calculate_distance(angle_coords):
        earth_radius_mi = 3958.90
        earth_radius_kms = 6378.14
        ft_mi = 5280
        m_kms = 1000
        rad_coords = [radians(x) for x in angle_coords]
        distance =  acos(sin(rad_coords[0])*sin(rad_coords[2])+cos(rad_coords[0])*cos(rad_coords[2])*cos(rad_coords[1]-rad_coords[3]))
        distance_m = distance * earth_radius_kms * m_kms
        distance_ft = distance * earth_radius_mi * ft_mi
        return {'meters':distance_m, 'feet': distance_ft}
        
    def __str__(self):
        return str(pprint.pprint((self._links,self._links_map)))


if __name__=='__main__':
    coords = [35.121848, -89.9394625, 35.1216198, -89.9394706]
    print(Map.calculate_distance(coords))

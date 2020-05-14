from map import Map


class Cost:

    def __init__(self, map):
        # {'a':{'total_effort':12, 'link_key': 'a_b', 'checked': true}}
        self._costs = {}
        self._map = map

    def start(self, start_node, end_node):
        self._costs = {}
        self.start_node = start_node
        self.end_node = end_node
        # seed with first node
        self._costs[start_node] = {
            "total_effort": 0,
            "link_key": None,
            "check_neighbors": True
        }
        node = self.get_unchecked_node()
        while node is not None:
            self.check_paths(node)
            node = self.get_unchecked_node()
        path = self.get_path()
        return path

    def check_paths(self, current_node):
        neighbors = self._map.get_neighbors(current_node)
        for neighbor in neighbors:
            key = self._map.get_key(current_node, neighbor)
            current_node_cost = self._costs[current_node]["total_effort"]
            total_effort = current_node_cost + self._map.get_link(key)["effort"]
            if (neighbor in self._costs and total_effort < self._costs[neighbor]["total_effort"]) or neighbor not in self._costs:
                self._costs[neighbor] = {
                    "total_effort": total_effort,
                    "link_key": key,
                    "check_neighbors": True
                }
        self._costs[current_node]["check_neighbors"] = False

    def get_unchecked_node(self):
        for node in self._costs:
            if self._costs[node]["check_neighbors"] == True:
                return node
        return None

    def get_path(self):
        path = []
        current_node = self.end_node
        cost = self._costs[current_node]
        while current_node != self.start_node:
            link = self._map.get_link(cost["link_key"])
            previous_node = link["start_node"] if current_node != link["start_node"] else link["end_node"]
            p = {
                "current_node": previous_node,
                "next_node": current_node,
                "link_key": cost["link_key"],
                "effort": cost["total_effort"]
            }
            path.append(p)
            current_node = previous_node
            cost = self._costs[current_node]
        path.reverse()
        return path
        
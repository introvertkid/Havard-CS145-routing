####################################################
# LSrouter.py
# Name:
# HUID:
#####################################################

import json
import heapq
from router import Router
from packet import Packet


class LSrouter(Router):
    """Link state routing protocol implementation.

    Add your own class fields and initialization code (e.g. to create forwarding table
    data structures). See the `Router` base class for docstrings of the methods to
    override.
    """

    def __init__(self, addr, heartbeat_time):
        super().__init__(addr)
        self.heartbeat_time = heartbeat_time
        self.last_time = 0
        self.lsdb = {}  # {router: {'links': {neighbor: cost}, 'sequence': int}}
        self.forwarding_table = {}  # {dst: (port, next_hop)}
        self.sequence = 0
        self.local_links = {}  # {endpoint: (port, cost)}

    def dijkstra(self):
        """Tính toán đường đi ngắn nhất bằng thuật toán Dijkstra."""
        # Xây dựng đồ thị từ LSDB và local_links
        graph = {}
        all_nodes = set()

        # Thêm thông tin từ LSDB
        for router, data in self.lsdb.items():
            graph[router] = data['links']
            all_nodes.add(router)
            all_nodes.update(data['links'].keys())

        # Thêm liên kết cục bộ (bao gồm client)
        if self.addr not in graph:
            graph[self.addr] = {}
        for endpoint, (port, cost) in self.local_links.items():
            graph[self.addr][endpoint] = cost
            if endpoint not in graph:
                graph[endpoint] = {}
            graph[endpoint][self.addr] = cost
            all_nodes.add(endpoint)

        # Khởi tạo khoảng cách và tiền thân
        distances = {node: float('inf') for node in all_nodes}
        distances[self.addr] = 0
        predecessors = {node: None for node in all_nodes}
        ports = {node: None for node in all_nodes}
        pq = [(0, self.addr)]
        visited = set()

        # Thuật toán Dijkstra
        while pq:
            dist, current = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)

            for neighbor, cost in graph.get(current, {}).items():
                if neighbor in visited:
                    continue
                new_dist = dist + cost
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    predecessors[neighbor] = current
                    # Gán cổng nếu neighbor là liên kết trực tiếp từ router hiện tại
                    if current == self.addr and neighbor in self.local_links:
                        ports[neighbor] = self.local_links[neighbor][0]
                    heapq.heappush(pq, (new_dist, neighbor))

        # Cập nhật bảng định tuyến
        self.forwarding_table = {}
        for dst in all_nodes:
            if dst == self.addr:
                continue
            current = dst
            path = []
            while current != self.addr and predecessors[current] is not None:
                path.append(current)
                current = predecessors[current]
            if current == self.addr:
                path.append(self.addr)
                path.reverse()
                if len(path) > 1:
                    next_hop = path[1]
                    if next_hop in self.local_links:
                        self.forwarding_table[dst] = (self.local_links[next_hop][0], next_hop)

    def broadcast_lsa(self):
        """Phát sóng LSA đến tất cả các láng giềng."""
        self.sequence += 1
        lsa = {
            'router': self.addr,
            'sequence': self.sequence,
            'links': {k: v[1] for k, v in self.local_links.items()}
        }
        packet = Packet(Packet.ROUTING, self.addr, None, content=json.dumps(lsa))
        for port in self.links:
            self.send(port, packet.copy())

    def handle_packet(self, port, packet):
        """Xử lý gói tin đến."""
        if packet.is_traceroute:
            if packet.dst_addr in self.forwarding_table:
                out_port, _ = self.forwarding_table[packet.dst_addr]
                self.send(out_port, packet)
        else:
            try:
                lsa = json.loads(packet.content)
                router = lsa['router']
                sequence = lsa['sequence']
                links = lsa['links']
                if router not in self.lsdb or sequence > self.lsdb[router]['sequence']:
                    self.lsdb[router] = {'links': links, 'sequence': sequence}
                    self.dijkstra()
                    for p in self.links:
                        if p != port:
                            self.send(p, packet.copy())
            except (json.JSONDecodeError, KeyError):
                pass

    def handle_new_link(self, port, endpoint, cost):
        """Xử lý liên kết mới."""
        self.local_links[endpoint] = (port, cost)
        self.lsdb[self.addr] = {'links': {k: v[1] for k, v in self.local_links.items()}, 'sequence': self.sequence}
        self.dijkstra()
        self.broadcast_lsa()

    def handle_remove_link(self, port):
        """Xử lý xóa liên kết."""
        endpoint = next((ep for ep, (p, _) in self.local_links.items() if p == port), None)
        if endpoint:
            del self.local_links[endpoint]
            self.lsdb[self.addr] = {'links': {k: v[1] for k, v in self.local_links.items()}, 'sequence': self.sequence}
            self.dijkstra()
            self.broadcast_lsa()

    def handle_time(self, time_ms):
        """Xử lý cập nhật định kỳ."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast_lsa()

    def __repr__(self):
        """String representation for debugging."""
        return f"LSrouter(addr={self.addr}, lsdb={self.lsdb}, forwarding_table={self.forwarding_table})"

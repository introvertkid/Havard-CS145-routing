####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

import json
from router import Router
from packet import Packet

INFINITY = 16

class DVrouter(Router):
    def __init__(self, addr, heartbeat_time):
        super().__init__(addr, heartbeat_time=heartbeat_time)
        # lưu heartbeat_time để handle_time
        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        # state chính
        self.routing_table    = {addr: (0, addr)}  # dest -> (cost, next_hop)
        self.neighbor_vectors = {}                 # neighbor_addr -> their last sent vector
        self.link_costs       = {}                 # neighbor_addr -> direct link cost

        # maps port <-> neighbor để forwarding
        self.port_to_neighbor = {}
        self.neighbor_to_port = {}

    def handle_packet(self, port, packet):
        if packet.is_traceroute:
            dst = packet.dst_addr
            if dst in self.routing_table:
                next_hop = self.routing_table[dst][1]
                out_port = self.neighbor_to_port.get(next_hop)
                if out_port is not None:
                    self.send(out_port, packet)

        elif packet.is_routing:
            sender = packet.src_addr
            # ignore nếu sender đã mất link
            if sender not in self.link_costs:
                return

            vec = json.loads(packet.content)
            # chỉ cập nhật khi vector thay đổi
            if self.neighbor_vectors.get(sender) != vec:
                self.neighbor_vectors[sender] = vec
                self.recompute_table()
                self.broadcast_vector()

    def handle_new_link(self, port, endpoint, cost):
        # lưu map để traceroute
        self.port_to_neighbor[port] = endpoint
        self.neighbor_to_port[endpoint] = port

        # cập nhật direct link cost và khởi vector neighbor
        self.link_costs[endpoint] = cost
        self.neighbor_vectors[endpoint] = {endpoint: 0}

        self.recompute_table()
        self.broadcast_vector()

    def handle_remove_link(self, port):
        if port not in self.port_to_neighbor:
            return
        nbr = self.port_to_neighbor.pop(port)
        self.neighbor_to_port.pop(nbr, None)
        self.link_costs.pop(nbr, None)
        self.neighbor_vectors.pop(nbr, None)

        self.recompute_table()
        self.broadcast_vector()

    def handle_time(self, time_ms):
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast_vector()

    def recompute_table(self):
        """Rebuild routing_table từ neighbor_vectors + link_costs."""
        me = self.addr
        new_table = {me: (0, me)}

        for nbr, vec in self.neighbor_vectors.items():
            if nbr not in self.link_costs:
                continue
            link_cost = self.link_costs[nbr]
            for dest, nbr_cost in vec.items():
                total = min(link_cost + nbr_cost, INFINITY)
                if total >= INFINITY:
                    continue
                if dest not in new_table or total < new_table[dest][0]:
                    new_table[dest] = (total, nbr)

        self.routing_table = new_table

    def broadcast_vector(self):
        """Broadcast poisoned distance vector to neighbors."""
        for nbr, port in self.neighbor_to_port.items():
            poisoned = {}
            for dest, (cost, nh) in self.routing_table.items():
                poisoned[dest] = INFINITY if nh == nbr else cost
            pkt = Packet(Packet.ROUTING, self.addr, nbr, json.dumps(poisoned))
            self.send(port, pkt)

    def __repr__(self):
        lines = [f"Router {self.addr} Routing Table:"]
        for dest, (cost, nh) in sorted(self.routing_table.items()):
            lines.append(f"  {dest} via {nh} cost {cost}")
        return "\n".join(lines)

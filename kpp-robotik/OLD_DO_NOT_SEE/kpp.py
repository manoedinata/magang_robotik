# KPP Programming
# UKM Robotika ITS 2025
#
# Hendra Manudinata / 5027251051
# Teknologi Informasi / FTEIC

import heapq

# Energi total
# untuk setiap awal jalan dan recharge
MAX_ENERGY = 1000

class Graph:
    def __init__(self, n):
        self.n = n
        self.adj = { }  # adjacency list: node -> [(neighbor, w, o)]

    def add_edge(self, u, v, w, o):
        if u not in self.adj: self.adj[u] = []
        self.adj[u].append((v, w, o))
        if v not in self.adj: self.adj[v] = []
        self.adj[v].append((u, w, o))

def solve_it_dawg(
    graph,
    start,
    target,
    rest_points,
    charging_stations,
    start_hour
):
    time = start_hour * 60

    pq = []
    # Priority queue (pq)
    # (total_energy_used, time, node, energy_left, path)
    heapq.heappush(pq, (
        0, # total_energy_used
        time, # time
        start, # node
        MAX_ENERGY, # energy_left
        [start] # [path]
    ))

    # Simpan visited node
    visited = dict()

    # Loop terus selama PQ tidak kosong
    while pq:
        # Ambil antrian dari PQ
        energy_used, time, node, energy_left, path = heapq.heappop(pq)

        # Robot sudah sampai tujuan
        if node == target:
            print(f"Total energi minimum: {energy_used}")
            print("Jalur: " + " -> ".join(path))
            print("Waktu tiba:")
            for i, n in enumerate(path):
                print(f"{n} (menit {i*2})")  # asumsi tiap edge waktu = 2 menit di contoh
            return

        # Posisi saat ini
        state = (node, energy_left, time)

        # Jika node sudah dikunjungi
        # dan energi lebih hemat: Skip
        # Jika sebaliknya, simpan visited node saat ini
        if state in visited and visited[state] <= energy_used:
            continue
        else:
            visited[state] = energy_used

        # Charging = Isi ke maksimum
        if node in charging_stations:
            heapq.heappush(pq, (energy_used, time, node, MAX_ENERGY, path))

        # Rest point = Tunggu hingga jam genap
        if node in rest_points:
            current_hour = time // 60

            if current_hour % 2 == 1:  # kalau masih jam ganjil
                minutes_to_next_hour = 60 - (time % 60)
                new_time = time + minutes_to_next_hour
            else:
                # Jika sudah jam genap, tidak perlu rest
                # new_time = time + 1
                new_time = time

            heapq.heappush(pq, (energy_used, new_time, node, energy_left, path))

        # Explore neighbors
        # (Path finding)
        for v, w, o in graph.adj.get(node, []):
            base_cost = w + o

            # Efek waktu
            hour = time // 60
            if hour % 2 == 1:  # ganjil
                cost = int(base_cost * 1.3)
            else:  # genap
                cost = int(base_cost * 0.8)

            if cost <= energy_left:
                # Add path to PQ
                heapq.heappush(pq, (
                    energy_used + cost,
                    time + 2,
                    v,
                    energy_left - cost,
                    path + [v]
                ))

    # Jika PQ sudah kosong dan tidak sampai ke tujuan,
    # Robot gagal
    print("Robot gagal dalam mencapai tujuan :(")

def main():
    # Input: Graph Node & Edge
    n, m = map(int, input().split())
    graph = Graph(n)
    for _ in range(m):
        u, v, w, o = input().split()
        w, o = int(w), int(o)
        graph.add_edge(u, v, w, o)

    # S T endpoint
    S, T = input().split()

    # Rest points
    rest_points = set(input().split())
    if "-" in rest_points: rest_points = set()

    # Charging stations
    charging_stations = set(input().split())
    if "-" in charging_stations: charging_stations = set()

    # Mechanic & Electrical (UNUSED)
    mechanic = input()
    electrical = input()

    # Start minutes
    start_hour = int(input())

    solve_it_dawg(graph, S, T, rest_points, charging_stations, start_hour)

if __name__ == "__main__":
    main()

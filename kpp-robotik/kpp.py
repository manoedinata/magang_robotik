# KPP Programming
# UKM Robotika ITS 2025
#
# Hendra Manudinata / 5027251051
# Teknologi Informasi / FTEIC

# KPP ini menggunakan algoritma Dijkstra
# yang dimodifikasi untuk sekalian menghitung
# waktu (ganjil/genap) dan charging point

MAX_ENERGY = 1000
SPEED = 100 # 100 m / min

class Graph:
    def __init__(self, n):
        self.n = n
        self.adj = {}
    def add_edge(self, u, v, w, o):
        self.adj.setdefault(u, []).append((v, w, o))
        self.adj.setdefault(v, []).append((u, w, o))

class KPPRobotik:
    def __init__(self):
        self.Graph = None
        self.states = [()]

    def init_graph(self, n):
        self.Graph = Graph(n)

    def get_best_routes(self, n, start, target, rest_points, charging_stations, start_hour):
        # states: list of (energy_used, time, node, energy_left, path, timeline)
        # Simpan state pertama
        self.states = [(0, start_hour*60, start, MAX_ENERGY, [start], [0])]

        # Simpan visited node
        visited = {}

        # Loop terus sampai states terambil semua
        while self.states:

            # Linear search: Ambil state dengan energi terendah
            min_energy = float('inf')
            idx = -1
            for i in range(len(self.states)):
                if self.states[i][0] >= min_energy: continue
                min_energy = self.states[i][0]
                idx = i

            # Pop (ambil) dari idx
            energy_used, time, node, energy_left, path, timeline = self.states.pop(idx)

            # Node sudah sampai target (T)
            if node == target:
                print("Total energi minimum:", energy_used)
                print("Jalur:", " -> ".join(path))
                print("Waktu tiba:")
                for n, t in zip(path, timeline):
                    t = t if t < 60 else t - (60 * start_hour)
                    print(f"{n} (menit {t})")
                return

            # key: (node, time parity, energy_left bucketed)
            state_key = (node, time // 60 % 2, energy_left)
            if state_key in visited and visited[state_key] <= energy_used:
                continue
            visited[state_key] = energy_used

            # Charging station: Refill energy ke maksimum
            if node in charging_stations and energy_left < MAX_ENERGY:
                self.states.append((energy_used, time, node, MAX_ENERGY, path, timeline))

            # Rest point: Cek waktu & istirahat
            # Jika jam masih ganjil, istirahat sampai genap
            if node in rest_points:
                hour = time // 60
                if hour % 2 == 1:
                    minutes_to_next_hour = 60 - (time % 60)
                    new_time = time + minutes_to_next_hour
                    self.states.append((energy_used, new_time, node, energy_left, path, timeline+[new_time]))

            # Jelajah node lain
            for v, w, o in self.Graph.adj.get(node, []):
                base_cost = w + o

                # Aturan waktu:
                # Ganjil = * 1.3 || Genap = * 0.8
                hour = time // 60
                if hour % 2 == 1:
                    cost = int(base_cost * 1.3)
                else:
                    cost = int(base_cost * 0.8)

                # Jika biaya energi lebih rendah,
                # tambah ke states
                if cost <= energy_left:
                    # Waktu = Jarak / Kecepatan
                    # SPEED disini adalah statis, yaitu 100 m/h
                    # Ini hanyalah contoh yang mempresentasikan waktu tempuh
                    travel_time = round(w / SPEED)

                    new_time = time + travel_time

                    self.states.append((
                        energy_used + cost,
                        new_time,
                        v,
                        energy_left - cost,
                        path + [v],
                        timeline + [new_time]
                    ))

        # Kondisi ini akan berjalan jika tidak ada states yang
        # cukup untuk sampai ke target
        print("Robot gagal dalam mencapai tujuan :(")

if __name__ == "__main__":
    # Initialize program
    kppObj = KPPRobotik()

    # N, M (Jumlah node, jumlah edge)
    n, m = map(int, input().split())
    kppObj.init_graph(n)

    # Add edges to Graph
    for _ in range(m):
        u, v, w, o = input().split()
        w, o = int(w), int(o)
        kppObj.Graph.add_edge(u, v, w, o)

    # Input: Start, Target
    S, T = input().split()

    # Rest points
    rest_points = set(input().split())
    if "-" in rest_points: rest_points = set()

    # Charging stations
    charging_stations = set(input().split())
    if "-" in charging_stations: charging_stations = set()

    # Mechanic & Electrial (UNUSED)
    input()
    input()

    # Start hour
    start_hour = int(input())

    # Execute dawg
    kppObj.get_best_routes(n, S, T, rest_points, charging_stations, start_hour)

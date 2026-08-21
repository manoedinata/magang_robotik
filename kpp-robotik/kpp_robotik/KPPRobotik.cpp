#include <iostream>
#include <queue>
#include <vector>
#include <map>
#include <cmath>
#include <functional>
#include <tuple>

#include "KPPRobotik.h"

// Tambah edge ke graph (u -> v, w, o)
void KPPRobotik::add_edge(
    const std::string& u,
    const std::string& v, int w, int o
){
    graph.add_edge(u, v, w, o);
}

// Core function: Ambil rute terbaik
void KPPRobotik::get_best_routes(
    const std::string& start,
    const std::string& target,
    const std::set<std::string>& rest_points,
    const std::set<std::string>& charging_stations,
    int start_hour
) {
    // Min-heap Priority Queue
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

    // Simpan visited node
    std::map<
        // State
        std::pair<
            std::string,
            int
        >,
        // Energy used
        int
    > visited;

    // Initial state
    State initial_state = {
        0, start_hour * 60, start, MAX_ENERGY, {start}, {0}
    };
    pq.push(initial_state);

    // Loop hingga Priority Queue kosong
    while (!pq.empty()) {
        State current = pq.top();
        pq.pop();

        // Jika node == T (Target), done. Tugas kita selesai.
        if (current.node == target) {
            std::cout << "Total energi minimum: " << current.energy_used << std::endl;
            std::cout << "Jalur: ";
            for (size_t i = 0; i < current.path.size(); ++i) {
                std::cout << current.path[i]
                << (i == current.path.size() - 1 ? "" : " -> ");
            }
            std::cout << "\n\nWaktu tiba:" << std::endl;
            for (size_t i = 0; i < current.path.size(); ++i) {
                std::cout << current.path[i] << " (menit "
                << (current.timeline[i] > (60)
                    ? current.timeline[i] - (60 * start_hour)
                    : current.timeline[i])
                << ")" << std::endl;
            }

            return;
        }

        // Jika node ini sudah visited dan punya energi yang cukup, skip.
        int time_parity = (current.time / 60) % 2;
        auto state_key = std::make_pair(current.node, time_parity);
        if (visited.count(state_key) && visited[state_key] <= current.energy_used) {
            continue;
        }
        visited[state_key] = current.energy_used;

        // Charging station
        // Refill energi ke energi maksimum
        if (charging_stations.count(current.node) && current.energy_left < MAX_ENERGY) {
            State next_state = current;
            next_state.energy_left = MAX_ENERGY;
            pq.push(next_state);
        }

        // Rest point
        // Disini, kita dapat menunggu hingga jam genap
        // untuk efisiensi energi. Secara waktu, ini sangatlah
        // boros waktu dan memperlama waktu tempuh node. Namun,
        // sangat membantu mengurangi penggunaan energi karena
        // jika genap, maka cost energi *= 0.8
        if (rest_points.count(current.node)) {
            int hour = current.time / 60;
            if (hour % 2 == 1) {
                int minutes_to_next_hour = 60 - (current.time % 60);
                int new_time = current.time + minutes_to_next_hour;
                
                State next_state = current;
                next_state.time = new_time;
                next_state.timeline.push_back(new_time);

                pq.push(next_state);
            }
        }

        // Explore neighbors menggunakan Dijkstra
        // Bandingkan cost saat ini dengan yang sudah ada.
        // Fungsi Priority Queue disini adalah menyimpan states per node
        // dan otomatis memberikan prioritas berdasarkan cost terendah
        // (min-heap)
        for (const auto& edge : graph.get_neighbors(current.node)) {
            std::string neighbor = std::get<0>(edge);
            int w = std::get<1>(edge);
            int o = std::get<2>(edge);

            int base_cost = w + o;

            int hour = current.time / 60;

            // Perhitungan biaya (cost) energi sesuai jam
            // Ganjil: cost *= 1.3
            // Genap: cost *= 0.8
            // Yang mana? Yap, cari yang genap.
            int cost = (hour % 2 == 1)
                ? static_cast<double>(base_cost * 1.3)
                : static_cast<double>(base_cost * 0.8);

            // Jika cost masih cukup dengan sisa energi,
            // tambahkan ke PQ
            if (cost <= current.energy_left) {
                // Hitung waktu tempuh
                // Waktu = Jarak / Kecepatan
                // SPEED disini adalah statis, yaitu 100 m/h
                // Ini hanyalah contoh yang mempresentasikan waktu tempuh
                int travel_time = std::round(static_cast<double>(w) / SPEED);

                int new_time = current.time + travel_time;

                State next_state;
                next_state.energy_used = current.energy_used + cost;
                next_state.time = new_time;
                next_state.node = neighbor;
                next_state.energy_left = current.energy_left - cost;
                next_state.path = current.path;
                next_state.path.push_back(neighbor);
                next_state.timeline = current.timeline;
                next_state.timeline.push_back(new_time);

                pq.push(next_state);
            }
        }
    }

    // Tereksekusi jika robot gagal :(
    std::cout << "Robot gagal dalam mencapai tujuan :(" << std::endl;
}

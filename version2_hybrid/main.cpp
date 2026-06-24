// =============================================================================
//  main.cpp  -  3D RRT Core Computation Engine (Version 2)
// =============================================================================
//  Heavy-math core of the hybrid desktop-simulation pipeline. Mirrors the pure
//  Python planner exactly, then ships the solved trajectory to the Python GUI
//  over a UDP socket.
//
//  DATA FLOW (important):
//      The Python GUI binds and LISTENS on 127.0.0.1:5005.
//      This engine COMPUTES the path, then SENDS it to that address.
//      (Two processes cannot bind the same port, so the "server" that owns the
//       socket is the Python listener; this engine is the UDP sender/client.)
//
//  The goal coordinates are passed in as command-line arguments by the GUI:
//      ./rrt_engine <goal_x> <goal_y> <goal_z>
//  If omitted, sensible defaults are used.
//
//  Wire format (raw ASCII string, one datagram):
//      "x1,y1,z1;x2,y2,z2;...;xn,yn,zn"
//  A leading "FAIL" datagram is sent if no path is found.
//
//  Build (Linux / macOS):
//      g++ -std=c++17 -O2 main.cpp -o rrt_engine
//  Build (Windows / MinGW):
//      g++ -std=c++17 -O2 main.cpp -o rrt_engine.exe -lws2_32
//  Run:
//      ./rrt_engine 90 90 20
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// ----------------------------- Socket headers ------------------------------ #
#if defined(_WIN32)
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  using socklen_t = int;
#else
  #define INVALID_SOCKET -1
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <unistd.h>
#endif

// --------------------------- Airspace configuration ------------------------ #
static const double X_MAX            = 100.0;  // Airspace width   (m)
static const double Y_MAX            = 100.0;  // Airspace depth   (m)
static const double Z_MAX            = 30.0;   // Airspace ceiling (m)

static const double STEP_SIZE        = 5.0;    // Max expansion per iteration (m)
static const double GOAL_SAMPLE_RATE = 0.10;   // Goal-bias probability (10%)
static const double GOAL_THRESHOLD   = 5.0;    // Goal-reached distance (m)
static const int    MAX_ITER         = 3000;   // Iteration budget (increased for better success rate)
static const double INFLATION        = 1.5;    // Obstacle inflation buffer (m)

static const char* UDP_HOST = "127.0.0.1";
static const int   UDP_PORT = 5005;

// --------------------------------- Obstacles ------------------------------- #
struct Box {
    double xmin, ymin, zmin, xmax, ymax, zmax;
};

static const std::vector<Box> OBSTACLES = {
    {20.0, 20.0,  0.0, 40.0, 40.0, 25.0},
    {60.0, 50.0,  0.0, 75.0, 70.0, 30.0},
    {45.0, 10.0,  0.0, 55.0, 60.0, 15.0},
};

// ------------------------------- Node structure ---------------------------- #
struct Node {
    double x, y, z;
    int    parent;   // index of the parent node in the tree (-1 for the root)
    double cost;     // cumulative distance from the start node
};

// ---------------------- Random number generation --------------------------- #
static std::mt19937 g_rng(static_cast<unsigned>(std::time(nullptr)));
static std::uniform_real_distribution<double> g_unit(0.0, 1.0);

static double rand_uniform(double lo, double hi) {
    return lo + g_unit(g_rng) * (hi - lo);
}

// ------------------------------- Core functions ---------------------------- #
// Euclidean distance (hitung jarak = "compute distance").
static double hitung_jarak(const Node& a, const Node& b) {
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Sample a node, with GOAL_SAMPLE_RATE probability of returning the goal.
static Node get_random_node(const Node& goal) {
    if (g_unit(g_rng) < GOAL_SAMPLE_RATE) {
        return Node{goal.x, goal.y, goal.z, -1, 0.0};
    }
    return Node{rand_uniform(0.0, X_MAX),
                rand_uniform(0.0, Y_MAX),
                rand_uniform(0.0, Z_MAX), -1, 0.0};
}

// Index of the node nearest to rnd.
static int get_nearest_node_id(const std::vector<Node>& nodes, const Node& rnd) {
    int best = 0;
    double best_d = hitung_jarak(nodes[0], rnd);
    for (size_t i = 1; i < nodes.size(); ++i) {
        double d = hitung_jarak(nodes[i], rnd);
        if (d < best_d) { best_d = d; best = static_cast<int>(i); }
    }
    return best;
}

// Local planner: step from `from` toward `to` by at most STEP_SIZE metres.
// Isolated so the linear block can later be swapped for a motion-primitives LUT.
static Node steer(const std::vector<Node>& nodes, int from_id, const Node& to) {
    const Node& from = nodes[from_id];
    double dist = hitung_jarak(from, to);
    Node nn;

    if (dist <= STEP_SIZE) {
        nn = Node{to.x, to.y, to.z, from_id, 0.0};
    } else {
        // ---- swappable linear-primitive block ----------------------------- //
        double ratio = STEP_SIZE / dist;
        nn = Node{from.x + (to.x - from.x) * ratio,
                  from.y + (to.y - from.y) * ratio,
                  from.z + (to.z - from.z) * ratio,
                  from_id, 0.0};
        // ------------------------------------------------------------------- //
    }
    nn.cost = from.cost + hitung_jarak(from, nn);
    return nn;
}

// Point inside an inflated box?
static bool point_in_inflated_box(double x, double y, double z, const Box& b) {
    return (b.xmin - INFLATION <= x && x <= b.xmax + INFLATION &&
            b.ymin - INFLATION <= y && y <= b.ymax + INFLATION &&
            b.zmin - INFLATION <= z && z <= b.zmax + INFLATION);
}

// Densely sample the segment; true if entirely collision-free.
static bool check_collision(const Node& from, const Node& to) {
    double dist = hitung_jarak(from, to);
    int steps = std::max(2, static_cast<int>(dist / 0.5));
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = from.x + (to.x - from.x) * t;
        double y = from.y + (to.y - from.y) * t;
        double z = from.z + (to.z - from.z) * t;
        for (const Box& b : OBSTACLES) {
            if (point_in_inflated_box(x, y, z, b)) return false;
        }
    }
    return true;
}

// Trace parents from the goal back to the start; returns ordered waypoints.
static std::vector<Node> extract_path(const std::vector<Node>& nodes, int goal_id) {
    std::vector<Node> path;
    int idx = goal_id;
    while (idx != -1) {
        path.push_back(nodes[idx]);
        idx = nodes[idx].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// --------------------------------- Planner --------------------------------- #
// Returns the solved path, or an empty vector on failure.
static std::vector<Node> plan(const Node& start, const Node& goal) {
    std::vector<Node> nodes;
    nodes.push_back(start);

    for (int i = 0; i < MAX_ITER; ++i) {
        Node rnd     = get_random_node(goal);
        int  near_id = get_nearest_node_id(nodes, rnd);
        Node new_n   = steer(nodes, near_id, rnd);

        if (!check_collision(nodes[near_id], new_n)) continue;
        nodes.push_back(new_n);
        int new_id = static_cast<int>(nodes.size()) - 1;

        if (hitung_jarak(new_n, goal) <= GOAL_THRESHOLD &&
            check_collision(new_n, goal)) {
            Node g = goal;
            g.parent = new_id;
            g.cost   = new_n.cost + hitung_jarak(new_n, goal);
            nodes.push_back(g);
            int goal_id = static_cast<int>(nodes.size()) - 1;

            std::cout << "[OK]  Goal reached at iteration " << (i + 1)
                      << "  |  path cost = " << g.cost << " m\n";
            return extract_path(nodes, goal_id);
        }
    }
    std::cout << "[FAIL] No path found within " << MAX_ITER << " iterations.\n";
    return {};
}

// ----------------------------- Serialisation ------------------------------- #
// "x1,y1,z1;x2,y2,z2;..." with 3-decimal precision.
static std::string serialise(const std::vector<Node>& path) {
    if (path.empty()) return "FAIL";
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(3);
    for (size_t i = 0; i < path.size(); ++i) {
        if (i) os << ';';
        os << path[i].x << ',' << path[i].y << ',' << path[i].z;
    }
    return os.str();
}

// ----------------------------- UDP transmission ---------------------------- #
static bool send_udp(const std::string& payload) {
#if defined(_WIN32)
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "WSAStartup failed\n";
        return false;
    }
#endif

#if defined(_WIN32)
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
#else
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
#endif

    if (sock == INVALID_SOCKET) {
        std::cerr << "socket() failed\n";
        return false;
    }

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(UDP_PORT);
    inet_pton(AF_INET, UDP_HOST, &dest.sin_addr);

    int sent = sendto(sock, payload.c_str(),
                       static_cast<int>(payload.size()), 0,
                       reinterpret_cast<sockaddr*>(&dest), sizeof(dest));

#if defined(_WIN32)
    closesocket(sock);
    WSACleanup();
#else
    close(sock);
#endif

    if (sent < 0) {
        std::cerr << "sendto() failed\n";
        return false;
    }
    std::cout << "[OK]  Sent " << sent << " bytes to "
              << UDP_HOST << ":" << UDP_PORT << "\n";
    return true;
}

// ----------------------------------- main ---------------------------------- #
int main(int argc, char** argv) {
    // Fixed start; goal taken from argv (supplied by the GUI), with defaults.
    Node start{5.0, 5.0, 5.0, -1, 0.0};
    Node goal {90.0, 90.0, 20.0, -1, 0.0};

    if (argc >= 4) {
        goal.x = std::atof(argv[1]);
        goal.y = std::atof(argv[2]);
        goal.z = std::atof(argv[3]);
    }
    std::cout << "Goal = (" << goal.x << ", " << goal.y << ", " << goal.z << ")\n";

    std::vector<Node> path = plan(start, goal);
    std::string payload = serialise(path);
    send_udp(payload);
    return 0;
}

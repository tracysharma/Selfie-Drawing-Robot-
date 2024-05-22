#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>

struct Point {
    double x, y;
    int index;
};

double distance(const Point& p1, const Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

std::vector<Point> readPointsFromFile(const std::string& filename) {
    std::ifstream inputFile(filename);
    std::vector<Point> points;
    double x, y;
    char comma;
    
    int index = 0;
    while (inputFile >> x >> comma >> y) {
        points.push_back({x, y, index++});
    }
    inputFile.close();
    
    return points;
}

std::vector<int> nearestNeighbor(const std::vector<Point>& points) {
    int n = points.size();
    std::vector<bool> visited(n, false);
    std::vector<int> path;
    
    int current = 0;
    visited[current] = true;
    path.push_back(current);
    
    for (int i = 1; i < n; ++i) {
        double minDistance = std::numeric_limits<double>::max();
        int nearestNeighbor = -1;
        
        for (int j = 0; j < n; ++j) {
            if (!visited[j]) {
                double dist = distance(points[current], points[j]);
                if (dist < minDistance) {
                    minDistance = dist;
                    nearestNeighbor = j;
                }
            }
        }
        
        visited[nearestNeighbor] = true;
        path.push_back(nearestNeighbor);
        current = nearestNeighbor;
    }
    
    path.push_back(0);
    
    return path;
}

void exportToCSV(const std::vector<Point>& points, const std::vector<int>& path, const std::string& filename) {
    std::ofstream outputFile(filename);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to create the output CSV file.\n";
        return;
    }
    
    for (int idx : path) {
        outputFile << points[idx].x << "," << points[idx].y << "\n";
    }
    
    outputFile.close();
    std::cout << "Optimized path exported to " << filename << std::endl;
}

int main() {
    std::string inputFile = "test.csv";
    std::string outputFile = "optimized_path.csv";
    
    std::vector<Point> points = readPointsFromFile(inputFile);
    std::vector<int> optimizedPath = nearestNeighbor(points);
    
    exportToCSV(points, optimizedPath, outputFile);
    
    return 0;
}

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm> 

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

std::vector<Point> downsamplePoints(const std::vector<Point>& originalPoints, int targetSize) {
    int originalSize = originalPoints.size();
    std::vector<Point> downsampledPoints;
    
    if (originalSize <= targetSize) {
        return originalPoints;
    }
    
    double step = static_cast<double>(originalSize) / targetSize;
    double idx = 0;
    while (downsampledPoints.size() < targetSize) {
        downsampledPoints.push_back(originalPoints[static_cast<int>(std::round(idx))]);
        idx += step;
    }
    
    return downsampledPoints;
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

struct Bounds {
    double minX, maxX, minY, maxY;
};

Bounds findBounds(const std::vector<Point>& points) {
    Bounds bounds;
    bounds.minX = bounds.maxX = points[0].x;
    bounds.minY = bounds.maxY = points[0].y;
    
    for (const auto& point : points) {
        if (point.x < bounds.minX) bounds.minX = point.x;
        if (point.x > bounds.maxX) bounds.maxX = point.x;
        if (point.y < bounds.minY) bounds.minY = point.y;
        if (point.y > bounds.maxY) bounds.maxY = point.y;
    }
    return bounds;
}

void scalePointsToA5(std::vector<Point>& points) {
    const double a5Width = 148.0; // A5 width in mm
    const double a5Height = 210.0; // A5 height in mm
    
    Bounds bounds = findBounds(points);
    
    double rangeX = bounds.maxX - bounds.minX;
    double rangeY = bounds.maxY - bounds.minY;
    double scaleX = a5Width / rangeX;
    double scaleY = a5Height / rangeY;

    double scale = std::min(scaleX, scaleY);

    for (auto& point : points) {
        point.x = (point.x - bounds.minX) * scale;
        point.y = (point.y - bounds.minY) * scale;
    }
}

int main() {
    std::string inputFile = "test.csv";
    std::string outputFile = "Scaled_Downsample_optimized_path.csv";
    int downsampledSize = 500;
    
    std::vector<Point> originalPoints = readPointsFromFile(inputFile);
    std::vector<Point> downsampledPoints = downsamplePoints(originalPoints, downsampledSize);
    scalePointsToA5(downsampledPoints); // Scale the points before finding the path
    std::vector<int> optimizedPath = nearestNeighbor(downsampledPoints);
    
    exportToCSV(downsampledPoints, optimizedPath, outputFile);
    
    return 0;
}

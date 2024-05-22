#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gd.h>

#define MAX_POINTS 100
#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 600
#define POINT_SIZE 4

typedef struct {
    double x;
    double y;
} Point;

int main() {
    FILE *file;
    char filename[] = "Downsample_optimized_path.csv"; // Fixed filename
    char line[1024];
    Point points[MAX_POINTS];
    int num_points = 0;

    // Open the CSV file
    file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error opening file!\n");
        return 1;
    }

    // Read the CSV file line by line
    while (fgets(line, sizeof(line), file) && num_points < MAX_POINTS) {
        // Tokenize the line
        char *token = strtok(line, ",");
        if (token == NULL) {
            continue; // Skip empty lines
        }

        // Read x coordinate
        points[num_points].x = atof(token);

        // Read y coordinate
        token = strtok(NULL, ",");
        if (token == NULL) {
            printf("Invalid format!\n");
            return 1;
        }
        points[num_points].y = atof(token);

        num_points++;
    }

    // Close the file
    fclose(file);

    // Calculate the minimum and maximum x and y coordinates
    double x_min = points[0].x;
    double x_max = points[0].x;
    double y_min = points[0].y;
    double y_max = points[0].y;

    for (int i = 1; i < num_points; i++) {
        if (points[i].x < x_min) {
            x_min = points[i].x;
        }
        if (points[i].x > x_max) {
            x_max = points[i].x;
        }
        if (points[i].y < y_min) {
            y_min = points[i].y;
        }
        if (points[i].y > y_max) {
            y_max = points[i].y;
        }
    }

    // Create a new GD image
    gdImagePtr image = gdImageCreateTrueColor(IMAGE_WIDTH, IMAGE_HEIGHT);
    int white = gdImageColorAllocate(image, 255, 255, 255);
    int black = gdImageColorAllocate(image, 0, 0, 0);

    // Draw the points on the image
    for (int i = 0; i < num_points; i++) {
        int x = (int)((points[i].x - x_min) / (x_max - x_min) * (IMAGE_WIDTH - 1));
        int y = (int)((points[i].y - y_min) / (y_max - y_min) * (IMAGE_HEIGHT - 1));
        gdImageFilledRectangle(image, x - POINT_SIZE / 2, y - POINT_SIZE / 2, x + POINT_SIZE / 2, y + POINT_SIZE / 2, black);
    }

    // Output the image to a file
    FILE *output_file = fopen("points.png", "wb");
    if (output_file == NULL) {
        printf("Error creating output file!\n");
        return 1;
    }
    gdImagePng(image, output_file);
    fclose(output_file);

    // Free the image
    gdImageDestroy(image);

    printf("Image saved as points.png\n");

    return 0;
}

#include <iostream>
#include <cmath>
#include <vector>

std::pair<std::vector<int>, std::vector<double>> calculatePixelDistances(double tilt, double height, double FOV, int resolution) {
    double dTheta = FOV / resolution;
    double leftAngle = 0.5 * FOV - tilt;
    double rightAngle = 0.5 * FOV + tilt;

    std::vector<double> leftThetas;
    for (double theta = 0; theta < leftAngle; theta += dTheta) {
        leftThetas.push_back(theta);
    }
    std::vector<double> rightThetas;
    for (double theta = 0; theta <= rightAngle; theta += (rightAngle - leftAngle) / (resolution - leftThetas.size())) {
        rightThetas.push_back(theta);
    }

    std::vector<double> leftDisplacements;
    for (double theta : leftThetas) {
        leftDisplacements.push_back(dTheta * height / (std::cos(theta) * std::cos(theta)));
    }
    std::vector<double> rightDisplacements;
    for (double theta : rightThetas) {
        rightDisplacements.push_back(dTheta * height / (std::cos(theta) * std::cos(theta)));
    }

    std::vector<double> leftDistances(leftDisplacements.size());
    for (int i = 0; i < leftDisplacements.size(); ++i) {
        double sum_of_disps = 0;
        for (int j = 0; j <= i; ++j) {
            sum_of_disps += leftDisplacements[j];
        }
        leftDistances[i] = sum_of_disps;
    }

    std::vector<double> rightDistances(rightDisplacements.size());
    for (int i = 0; i < rightDisplacements.size(); ++i) {
        double sum_of_disps = 0;
        for (int j = 0; j <= i; ++j) {
            sum_of_disps += rightDisplacements[j];
        }
        rightDistances[i] = sum_of_disps;
    }

    std::vector<int> pixel_indices;
    for (int i = 0; i < resolution; ++i) {
        pixel_indices.push_back(i);
    }
    std::vector<double> distances(resolution);
    for (int i = 0; i < resolution; ++i) {
        if (i < leftDistances.size()) {
            distances[i] = leftDistances[leftDistances.size() - 1 - i];
        } else {
            distances[i] = rightDistances[i - leftDistances.size()];
        }
    }

    return std::make_pair(pixel_indices, distances);
}

std::pair<double, double> addMetersToCoords(double latitude, double longitude, double dlat, double dlon) {
    const double R_EARTH = 6378000;
    double new_lat = latitude + (dlat / R_EARTH) * (180 / M_PI);
    double new_lon = longitude + (dlon / R_EARTH) * (180 / M_PI) / std::cos(latitude * M_PI / 180);

    return std::make_pair(new_lat, new_lon);
}

std::pair<double, double> getTargetLatLon(double plane_lat, double plane_lon, double plane_pitch, double plane_roll, double plane_yaw, double plane_altitude, int target_x, int target_y, int image_x, int image_y) {
    const double NORTHYAW = 0;
    const double DiagFOV = 120

    double XFOV = DiagFOV * image_x / std::sqrt(image_y * image_y + image_x * image_x) * M_PI / 180;
    double YFOV = DiagFOV * image_y / std::sqrt(image_y * image_y + image_x * image_x) * M_PI / 180;

    std::vector<std::vector<double>> x_distances = calculatePixelDistances(plane_roll, plane_altitude, XFOV, image_x);
    std::vector<std::vector<double>> y_distances = calculatePixelDistances(plane_pitch, plane_altitude, YFOV, image_y);

    int x_pixel_index;
    for (int i = 0; i < x_distances[0].size(); i++) {
        if (x_distances[0][i] == target_x) {
            x_pixel_index = i;
            break;
        }
    }
    double x_dist = x_distances[1][x_pixel_index];

    int y_pixel_index;
    for (int i = 0; i < y_distances[0].size(); i++) {
        if (y_distances[0][i] == target_y) {
            y_pixel_index = i;
            break;
        }
    }
    double y_dist = y_distances[1][y_pixel_index];

    double yaw = plane_yaw - NORTHYAW;

    double x_prime_meters = x_dist * std::cos(yaw) + y_dist * std::sin(yaw);
    double y_prime_meters = y_dist * std::cos(yaw) - x_dist * std::sin(yaw);

    std::pair<double, double> finalCoords = addMetersToCoords(plane_lat, plane_lon, y_prime_meters, x_prime_meters);

    double target_lat = finalCoords.first;
    double target_lon = finalCoords.second;

    return std::make_pair(target_lat, target_lon);
}

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <random>
#include <algorithm>
#include <math.h>
#include <map>




struct Point {
    double x, y, z;
    int label, r, g, b;

    bool operator==(const Point& other) const {
        return (x == other.x) && (y == other.y) && (z == other.z);
    }

    Point& operator=(const Point& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
            r = other.r;
            g = other.g;
            b = other.b;
            label = other.label;
        }
        return *this;
    }
};

// Calculate rgbkey
std::string calculate_rgbKey(Point point) {
    std::string x;
    x = std::to_string(point.r) + std::to_string(point.g) + std::to_string(point.b);
    int rgb;
    rgb = point.r + point.g + point.b;
    x += std::to_string(rgb);
    return x;
}

double calculate_distance(Point pointA, Point pointB) {
    double distance;
    distance = sqrt(pow((pointA.x - pointB.x), 2) + pow((pointA.y - pointB.y), 2) + pow((pointA.z - pointB.z), 2));
    return distance;
}

double calculate_mean(std::map<std::string, double> map) {
    double sum = 0;
    int size = map.size();
    double mean = 0;
    for (const auto& pair : map) {
        sum += pair.second;
    }
    mean = sum / (double)size;
    return mean;
}

Point calculate_centralpoint(std::vector<Point> points) {
    Point centralpoint;
    double x_sum = 0;
    double y_sum = 0;
    double z_sum = 0;
    int count = 0;


    for (const auto& point : points) {
        x_sum += point.x;
        y_sum += point.y;
        z_sum += point.z;
        count += 1;
    }

    double x_mean = x_sum / double(count);
    double y_mean = y_sum / double(count);
    double z_mean = z_sum / double(count);

    centralpoint.x = x_mean;
    centralpoint.y = y_mean;
    centralpoint.z = z_mean;
    centralpoint.r = points[0].r;
    centralpoint.g = points[0].g;
    centralpoint.b = points[0].b;
    centralpoint.label = points[0].label;


    return centralpoint;
}

//------------------------------MAIN------------------------------//

int main(int argc, char *argv[]) {
    std::string source_dir = "/home/airlab/catkin_dores/";
    std::string data_dir = source_dir + "src/DORES/map_data/" +argv[1] +"/";
    std::string LidarPoseFile = data_dir + argv[1] + "_Lidar_pose_section.txt";
    std::string OriginalMapFile = data_dir + argv[1] + "_clustered_input.txt";
    std::string ground_removed_map = data_dir + argv[1] + "_ground_removed.txt";
    std::string OriginalMap_withline = data_dir + argv[1] + "_with_line.txt";
    std::string dynamic_removed_map = data_dir + argv[1] + "_dynamic_removed.txt";


    double range = 0.5;
    int random_value_count = 10;

    std::cout << "---------------Getting the LIDAR Pose--------------" << std::endl;
    std::ifstream inputFile1(LidarPoseFile);
    if (!inputFile1) {
        std::cerr << "Error in opening LidarPoseFile" << std::endl;
    }
    
    std::vector<Point> Lidar_trajectory;
    double x, y, z, r11,r12,r13,r21,r22,r23,r31,r32,r33;
    int label, r, g, b;
    while (inputFile1 >> r11 >> r12 >> r13 >> x >> r21 >> r22 >> r23 >> y >> r31 >> r32 >> r33 >> z) {
        Point LidarPoint = { z,-x,-y,255,255,255 };
        Lidar_trajectory.push_back(LidarPoint);
    }

    inputFile1.close();

    std::cout << "---------------Getting the Original Map---------------" << std::endl;
    std::ifstream inputFile2(OriginalMapFile);
    if (!inputFile2) {
        std::cerr << "Error in opening original map file" << std::endl;
    }

    std::vector<Point> OriginalMap;
    while (inputFile2 >> x >> y >> z >> label >> r >> g >> b) {
        Point OriginalMap_point = { x,y,z,label,r,g,b };
        OriginalMap.push_back(OriginalMap_point);
    }

    inputFile2.close();

    std::cout << "---------------Clustering same color points---------------" << std::endl;
    std::map<std::string, std::vector<Point>> clustered_group;
    for (const auto& point : OriginalMap) {
        std::string string_rgb = calculate_rgbKey(point);
        if (clustered_group.find(string_rgb) == clustered_group.end()) {
            clustered_group[string_rgb] = std::vector<Point>();
        }
        clustered_group[string_rgb].push_back(point);
    }

    std::cout << "---------------Calculating point count of each cluster---------------" << std::endl;
    std::map<std::string, int> point_count;
    for (const auto& pair : clustered_group) {
        const std::string color = pair.first;
        const std::vector<Point> points = pair.second;
        int count = points.size();
        point_count[color] = count;
      //  std::cout << count << std::endl;
    }

    std::cout << "---------------Finding ground cluster---------------" << std::endl;
    std::string ground_color;
    int ground_count = 0;
    for (const auto& pair : point_count) {
        const std::string color = pair.first;
        const int count = pair.second;
        if (count > ground_count) {
            ground_count = count;
            ground_color = color;
        }
    }
    std::cout << ground_count << std::endl;
    std::cout << ground_color << std::endl;

    std::cout << "---------------Calculate average neighbored points of each cluster---------------" << std::endl;
    std::map<std::string, double> neighbored_point;
    for (const auto& pair : clustered_group) {
        const std::string& color = pair.first;
        const std::vector<Point>& clustered_points = pair.second;

        if(color != ground_color){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::vector<Point> randomPoints;
            std::vector<int> chosenIndex;
            std::uniform_int_distribution<> distrib(0, clustered_points.size() - 1);
            while (randomPoints.size() < random_value_count) {
                int index = distrib(gen);
                if (std::find(chosenIndex.begin(), chosenIndex.end(), index) == chosenIndex.end()) {
                    randomPoints.push_back(clustered_points[index]);
                    chosenIndex.push_back(index);
                }
            }

            int neighbored_point_count = 0;
            for (int i = 0; i < randomPoints.size(); i++) {
                for (const auto& point : clustered_points) {
                    if (!(randomPoints[i] == point)) {
                        if (calculate_distance(randomPoints[i], point) <= range) {
                            neighbored_point_count += 1;
                        }
                    }
                }
            }
            neighbored_point[color] = double(neighbored_point_count) / double(random_value_count);
        }
    }

    std::cout << "---------------Calculate avrage closeset_distance of each cluster---------------" << std::endl;
    std::map<std::string, double> closest_distance;
    for (const auto& pair : clustered_group) {
        const std::string& color = pair.first;
        const std::vector<Point>& clustered_points = pair.second;

        if(color != ground_color){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::vector<Point> randomPoints;
            std::vector<int> chosenIndex;
            std::uniform_int_distribution<> distrib(0, clustered_points.size() - 1);
            while (randomPoints.size() < random_value_count) {
                int index = distrib(gen);
                if (std::find(chosenIndex.begin(), chosenIndex.end(), index) == chosenIndex.end()) {
                    randomPoints.push_back(clustered_points[index]);
                    chosenIndex.push_back(index);
                }
            }
            double distance_sum = 0;
            double distance_mean = 0;
            for (int i = 0; i < randomPoints.size(); i++) {
                std::vector<double> distance;

                for (const auto& point : clustered_points) {
                    distance.push_back(calculate_distance(randomPoints[i], point));
                }
                std::sort(distance.begin(), distance.end());
                for (int j = 0; j < 10; j++) {
                    distance_sum += distance[j];
                }
            }
            closest_distance[color] = distance_sum / (double)(randomPoints.size() * 10);
        }
    }

    std::cout << "---------------Calculate central point of each cluster---------------" << std::endl;
    std::map<std::string, Point>central_points;
    for (const auto& pair : clustered_group) {
        const std::string color = pair.first;
        const std::vector<Point>& clustered_points = pair.second;
        Point central_point;
        if(color != ground_color){
            central_point = calculate_centralpoint(clustered_points);
            central_points[color] = central_point;
        }
    }

    std::cout << "---------------Calculate distance proportional parameter of each cluster---------------" << std::endl;
    std::map<std::string, double> distance_parameter;
    for (const auto& pair : central_points) {
        const std::string color = pair.first;
        const Point central_point = pair.second;
        double dis_min = 10000;
        for (const auto& lidar_point : Lidar_trajectory) {
            double dis_curr = calculate_distance(central_point, lidar_point);
            if (dis_curr < dis_min) {
                dis_min = dis_curr;
            }
        }
        distance_parameter[color] = dis_min;
    }

    std::cout << "--------------- Apply the parameter to density ---------------" << std::endl;
    std::map<std::string, double> param_applied_neighbored_points;
    for (const auto& pair : neighbored_point) {
        const std::string color = pair.first;
        const double density = pair.second;
        double param_applied_density = density * distance_parameter[color];
        param_applied_neighbored_points[color] = param_applied_density;
    }

    std::cout << "---------------Apply the parameter to closest_distance ---------------" << std::endl;
    std::map<std::string, double> param_applied_closest_distance;
    for (const auto& pair : closest_distance) {
        const std::string color = pair.first;
        const double distance = pair.second;
        double param_applied_distance = distance / distance_parameter[color];
        param_applied_closest_distance[color] = param_applied_distance;
    }
    std::map<std::string, double> inverse_closest_distance;
    for (const auto& pair : param_applied_closest_distance) {
        const std::string color = pair.first;
        const double distance = pair.second;
        double distance_inverse = 1 / distance;
        inverse_closest_distance[color] = distance_inverse;
    }

    std::cout << "---------------Normalize density and closest distance ---------------" << std::endl;
    std::map<std::string, double> normalized_neighbored_points;
    double density_max = 0;
    for (const auto& pair : param_applied_neighbored_points) {
        const double density = pair.second;
        if (density_max <= density) {
            density_max = density;
        }
    }
    for (const auto& pair : param_applied_neighbored_points) {
        const std::string color = pair.first;
        const double density = pair.second;
        double normalized_density = density / density_max;
        normalized_neighbored_points[color] = normalized_density;
    }
    
    std::map<std::string, double> normalized_closest_distance_inverse;
    double closest_distance_inv_max = 0;
    for (const auto& pair : inverse_closest_distance) {
        const double distance = pair.second;
        if (closest_distance_inv_max <= distance) {
            closest_distance_inv_max = distance;
        }
    }
    for (const auto& pair : inverse_closest_distance) {
        const std::string color = pair.first;
        const double distance = pair.second;
        double normalized_distance = distance / closest_distance_inv_max;
        normalized_closest_distance_inverse[color] = normalized_distance;
    }

    std::cout << "---------------Plus density and closest_distance ---------------" << std::endl;
    std::map<std::string, double> The_value;
    for (const auto& pair : normalized_neighbored_points) {
        const std::string color = pair.first;
        double value = normalized_neighbored_points[color] + normalized_closest_distance_inverse[color];
        The_value[color] = value;
    }

    
    std::cout << "---------------Remove dynamic object ---------------" << std::endl;
    std::vector<std::pair<std::string, double>> sorted_value(The_value.begin(), The_value.end());
    std::sort(sorted_value.begin(), sorted_value.end(),
        [](const auto& left, const auto& right) {
            return left.second < right.second;
        });

    int remove_threshold = std::round(0.1 * sorted_value.size());

    for (int i = 0; i < remove_threshold; ++i) {
        const std::string& color_to_remove = sorted_value[i].first;
        clustered_group.erase(color_to_remove);
    }


    std::cout << "---------------Save the results to output file ---------------" << std::endl;
    //std::ofstream outputFile1(ground_removed_map);
    //if (!outputFile1) {
    //    std::cerr << "Error in opening output file: " << ground_removed_map << std::endl;
    //    return 1;
    //}
    //for (const auto& pair : clustered_group) {
    //    const std::string color = pair.first;
    //    const std::vector<Point> points = pair.second;
    //    if (color != ground_color) {
    //        for (const auto& point : points) {
    //            outputFile1 << point.x << " " << point.y << " " << point.z << " "
    //                << point.r << " " << point.g << " " << point.b << std::endl;
    //        }
    //    }
    //}
    //outputFile1.close();

    //std::ofstream outputFile2(OriginalMap_withline);
    //if (!outputFile2) {
    //    std::cerr << "Error in opening output file : " << OriginalMap_withline << std::endl;
    //    return 1;
    //}
    //for (const auto& pair : clustered_group) {
    //    const std::string color = pair.first;
    //    const std::vector<Point> points = pair.second;
    //    if (color != ground_color) {
    //        for (const auto& point : points) {
    //            outputFile2 << point.x << " " << point.y << " " << point.z << " "
    //                << point.r << " " << point.g << " " << point.b << std::endl;
    //        }
    //    }
    //}
    //for (const auto& lidar_point : Lidar_trajectory) {
    //    outputFile2 << lidar_point.x << " " << lidar_point.y << " " << lidar_point.z << " "
    //        << lidar_point.r << " " << lidar_point.g << " " << lidar_point.b << std::endl;
    //}
    //outputFile2.close();

    std::ofstream outputFile3(dynamic_removed_map);
    if (!outputFile3) {
        std::cerr << "Error in opening output file : " << dynamic_removed_map << std::endl;
        return 1;
    }
    for (const auto& pair : clustered_group) {
        const std::string color = pair.first;
        const std::vector<Point> points = pair.second;
        for (const auto& point : points) {
            //outputFile3 << point.x << " " << point.y << " " << point.z << " "
            //    << point.r << " " << point.g << " " << point.b << std::endl;
            outputFile3 << point.x << " " << point.y << " " << point.z << " "
                << point.label << std::endl;
        }
    }
    outputFile3.close();

}

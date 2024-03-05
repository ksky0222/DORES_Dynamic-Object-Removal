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
    int label;
    int order;

    bool operator==(const Point& other) const {
        return (x == other.x) && (y == other.y) && (z == other.z);
    }

    Point& operator=(const Point& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
            label = other.label;
            order = other.order;
        }
        return *this;
    }
};

double calculate_distance(Point pointA, Point pointB) {
    double distance;
    distance = sqrt(pow((pointA.x - pointB.x), 2) + pow((pointA.y - pointB.y), 2) + pow((pointA.z - pointB.z), 2));
    return distance;
}

//------------------------------MAIN------------------------------//

int main(int argc, char *argv[]) {
    std::string source_dir = "/home/airlab/catkin_dores/";
    std::string data_dir = source_dir + "src/DORES/map_data/" +argv[1] +"/";
    std::string LidarPoseFile = data_dir + argv[1] + "_lidar_odom_withorder.txt";
    std::string OriginalMapFile = data_dir + argv[1] + "_original_map_withorder.txt";
    std::string dynamic_removed_map_withorder = data_dir + argv[1] + "_dynamic_removed_withorder.txt";


    double range = 0.5;
    int random_value_count = 10;

    std::cout << "---------------Getting the LIDAR Pose--------------" << std::endl;
    std::ifstream inputFile1(LidarPoseFile);
    if (!inputFile1) {
        std::cerr << "Error in opening LidarPoseFile" << std::endl;
    }
    
    std::vector<Point> Lidar_trajectory;
    double x, y, z;
    int label, order;
    while (inputFile1>> x >> y >> z) {
        Point LidarPoint = {x,y,z,0,0};
        Lidar_trajectory.push_back(LidarPoint);
    }
    inputFile1.close();

    std::cout << "---------------Getting the Original Map---------------" << std::endl;
    std::ifstream inputFile2(OriginalMapFile);
    if (!inputFile2) {
        std::cerr << "Error in opening original map file" << std::endl;
    }

    std::vector<Point> OriginalMap;
    while (inputFile2 >> x >> y >> z >> label >> order) {
        Point OriginalMap_point = { x,y,z,label,order};
        OriginalMap.push_back(OriginalMap_point);
    }
    inputFile2.close();
    
    std::vector<Point> RemovedMap = OriginalMap;
    double grid_size = 1;
    for(int scene_num = 0; scene_num<Lidar_trajectory.size(); scene_num++){
        double lidar_x = Lidar_trajectory[scene_num].x;
        double lidar_y = Lidar_trajectory[scene_num].y;
        double lidar_z = Lidar_trajectory[scene_num].z;

        double max_x = lidar_x + 50;
        double min_x = lidar_x - 50;
        double max_y = lidar_y + 50;
        double min_y = lidar_y - 50;
        
        //seperate the point that inrange and outrange
        std::vector<Point> Point_inrange = {};
        std::vector<Point> Point_outrange = {};
        for(const auto& point : RemovedMap){
            if(point.x>=min_x && point.x <= max_x){
                if(point.y>=min_y && point.y <=min_y){
                    Point_inrange.push_back(point);
                }
                else{
                    Point_outrange.push_back(point);
                }
            }
            else{
                Point_outrange.push_back(point);
            }
        }

        // devided the inrange point on each grid
        std::map<std::string,std::vector<Point>> devided_grid;
        for(const auto& point : Point_inrange){
            int grid_x = (point.x - min_x)/grid_size;
            int grid_y = (point.y - min_y)/grid_size;
            std::string grid_x_str = std::to_string(grid_x);
            std::string grid_y_str = std::to_string(grid_y);
            std::string grid = "x"+grid_x_str+"y"+grid_y_str;
            devided_grid[grid].push_back(point);
        }

        std::cout << "---------------Judging the moving object in each grid---------------" << std::endl;
        std::map<std::string,std::vector<double>> occupancy; // store the delta_h of each scene in each grid 
        for(const auto& pair : devided_grid){
            std::string grid = pair.first;
            std::vector<Point> group = pair.second;
            std::map<int, std::vector<Point>> seperate_with_order;
            for(const auto& point : group){
                seperate_with_order[point.order].push_back(point);
            }
            for(const auto& pair2 : seperate_with_order){
                int max_h = 0;
                int min_h = 1000;
                int order = pair2.first;
                std::vector<Point> ordered_group = pair2.second;
                for(const auto& ordered_point : ordered_group){
                    if(ordered_point.z > -3){
                        if(ordered_point.z > max_h){
                            max_h = ordered_point.z;
                        }
                        if(ordered_point.z < min_h) {
                            min_h = ordered_point.z;
                        }
                    }
                }
                double delta_h = max_h-min_h;
                occupancy[grid].push_back(delta_h);
            }
        }
        
        std::map<std::string, double> occupancy_delta;
        for(const auto& pair : occupancy){
            std::string grid = pair.first;
            std::vector<double> delta = pair.second;
            double diff = 0;
            for(int i = 0; i<delta.size()-1; i++){
                double temp_diff = fabs(delta[i+1]- delta[i]);
                if(temp_diff > diff){
                    diff = temp_diff;
                }
            }
            occupancy_delta[grid] = diff;
        }
        std::vector<Point> temp_map = Point_outrange;
        for(const auto& pair : devided_grid){
            std::string grid = pair.first;
            std::vector<Point> group = pair.second;
            int min_h = 1000;
            if(occupancy_delta[grid] > 1){
                for(const auto& point : group){
                    if(point.z > -3){
                        if(point.z < min_h) {
                            min_h = point.z;
                        }
                    }                    
                }
                for(const auto& point : group){
                    if(point.z < min_h+0.1){
                        Point_outrange.push_back(point);
                    }
                }
            }
        }
        RemovedMap = temp_map;
    }
    std::cout << "---------------Saving the moving object removal map---------------" << std::endl;
    std::ofstream outputFile(dynamic_removed_map_withorder);
    if (!outputFile) {
        std::cerr << "Error in opening output file : " << dynamic_removed_map_withorder << std::endl;
        return 1;
    }
    for (const auto& point : RemovedMap) {
        outputFile << point.x << " " << point.y << " " << point.z << " "
            << point.label << std::endl;
    }
    outputFile.close();
}

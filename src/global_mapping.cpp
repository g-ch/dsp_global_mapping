#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "object_info_handler.h"

struct Vertex {
    float x, y, z;
    uint8_t r, g, b;
};


class PointCloudToPly {
public:
    PointCloudToPly(const std::string& points_topic, const std::string& pose_topic, const std::string& output_file, const float map_range_xy, const float map_range_z, const float voxel_size, bool write_color = true, std::string object_csv="") 
        : output_file_(output_file), first_message_(true), total_vertices_(0), no_message_count_(0), map_range_xy_(map_range_xy), map_range_z_(map_range_z), voxel_size_(voxel_size), write_color_(write_color), object_csv_(object_csv) {
        
        std::cout << "Output file: " << output_file_ << std::endl;
        std::cout << "Map range xy: " << map_range_xy_ << std::endl;
        std::cout << "Map range z: " << map_range_z_ << std::endl;
        std::cout << "Voxel size: " << voxel_size_ << std::endl;

        semantic_txt_file_ = output_file_.substr(0, output_file_.find_last_of(".")) + ".txt";

        if(!object_csv_.empty()){
            object_info_handler_.readObjectInfo(object_csv_);

            std::cout << "Object information csv file: " << object_csv_ << std::endl;
        }

        ros::NodeHandle nh;
        // Use message_filters to synchronize the point cloud and pose messages
        message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, points_topic, 1);
        message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, pose_topic, 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), points_sub, pose_sub);
        sync.registerCallback(boost::bind(&PointCloudToPly::pointCloudCallback, this, _1, _2));

        ros::Rate loop_rate(10);
        no_message_count_ = 0;
        while (ros::ok())
        {
            if(!first_message_){ // Start counting no message after the first message
                no_message_count_++;
            }
            
            if (no_message_count_ > 100) {
                ROS_INFO("No message received for 10 seconds. Exiting...");
                break;
            }

            ros::spinOnce();
            loop_rate.sleep();
        }

        // Write the remaining vertices to the ply file
        if (write_color_) {
            write_ply_data(full_vertices_last_frame_);
        }else{
            write_ply_no_color(full_vertices_last_frame_);
        }
        total_vertices_ += full_vertices_last_frame_.size();

        if (total_vertices_ > 0) {
            update_ply_header();
            ROS_INFO("Total vertices: %zu", total_vertices_);
        }else{
            ROS_INFO("No point cloud received. The output file is not useful.");
        }
    }

    /// @brief Callback function for the point cloud message
    /// @param points_msg The point cloud message
    /// @param pose_msg The pose message
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg, const geometry_msgs::PoseStampedConstPtr& pose_msg) {
        std::cout << "Received point cloud message" << std::endl;

        std::vector<Vertex> vertices;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*points_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*points_msg, "z");
        
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(*points_msg, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(*points_msg, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(*points_msg, "b");

        static geometry_msgs::PoseStamped last_pose;

        static int count = 0;
        count++;

        std::cout << "Step: " << count << std::endl;

        if (first_message_) 
        {
            if (write_color_) {
                write_ply_header();
            }else{
                write_ply_header_no_color();
            }
            first_message_ = false;
        }
        else
        {   
            // Find the vertices that are not in the map range of the current frame but in the map range of the last frame
            for(auto &vertex : full_vertices_last_frame_){
                // Exlcude sky and dynamic points
                if(!check_if_color_need_to_save(vertex.r, vertex.g, vertex.b)){
                    continue;
                }

                geometry_msgs::PoseStamped this_pose = *pose_msg;
                bool in_new_map_range = check_if_point_in_map_range(vertex.x, vertex.y, vertex.z, this_pose);

                if (!in_new_map_range) {
                    vertices.push_back(vertex);
                    total_vertices_++;
                }
            }

            // Write the vertices to the ply file
            std::cout << "Writing " << vertices.size() << " vertices to the ply file" << std::endl;
            if (!vertices.empty()) {
                if (write_color_) {
                    write_ply_data(vertices);
                }else{
                    write_ply_no_color(vertices);
                }
            }
                     
        }
                
        // Save the vertices of the current frame for the next frame
        full_vertices_last_frame_.clear();
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
            full_vertices_last_frame_.push_back(Vertex{ *iter_x, *iter_y, *iter_z, *iter_r, *iter_g, *iter_b });
        }   

        last_pose = *pose_msg;

        no_message_count_ = 0;
    }

    /// @brief Check if a point is within the map range
    /// @param x The x coordinate of the point
    /// @param y The y coordinate of the point
    /// @param z The z coordinate of the point
    /// @param map_pose The pose of the map
    /// @return True if the point is within the map range, false otherwise
    bool check_if_point_in_map_range(const float &x, const float &y, const float &z, geometry_msgs::PoseStamped &map_pose) {
        
        /// Check if the voxel size is valid. If not, return false. Then all the points will be saved
        if(voxel_size_ <= 0){
            return false;
        }

        static const float map_range_xy_half = map_range_xy_ / 2.f - voxel_size_;
        static const float map_range_z_half = map_range_z_ / 2.f - voxel_size_;

        if (x < map_pose.pose.position.x - map_range_xy_half || x > map_pose.pose.position.x + map_range_xy_half) {
            return false;
        }

        if (y < map_pose.pose.position.y - map_range_xy_half || y > map_pose.pose.position.y + map_range_xy_half) {
            return false;
        }

        if (z < map_pose.pose.position.z - map_range_z_half || z > map_pose.pose.position.z + map_range_z_half) {
            return false;
        }
    
        return true;
    }


private:

    /// @brief Check if the color of a point should be saved. We do not save the color of person and rider.
    /// @param r 
    /// @param g 
    /// @param b 
    /// @return true if the color should be saved, false otherwise
    bool check_if_color_need_to_save(const int &r, const int &g, const int &b){
        static bool first = true;
        static std::vector<cv::Vec3b> ignore_color;

        if(first){
            int person_id = object_info_handler_.label_id_map.at("Person");
            int rider_id = object_info_handler_.label_id_map.at("Rider");
            int sky_id = object_info_handler_.label_id_map.at("Sky");

            ignore_color.push_back(object_info_handler_.label_color_map.at(person_id));
            ignore_color.push_back(object_info_handler_.label_color_map.at(rider_id));
            ignore_color.push_back(object_info_handler_.label_color_map.at(sky_id));

            first = false;
        }

        for(auto &color : ignore_color){
            if(r == color[2] && g == color[1] && b == color[0]){
                return false;
            }
        }

        return true;
    }

    /// @brief Get the label id from the color of a point
    /// @param r 
    /// @param g 
    /// @param b 
    /// @return Label id. 0 if the color is not in the label color map
    int get_label_from_color(const int &r, const int &g, const int &b){
        for(auto &label_id_color : object_info_handler_.label_color_map){
            if(r == label_id_color.second[2] && g == label_id_color.second[1] && b == label_id_color.second[0]){
                return label_id_color.first;
            }
        }

        return 0;
    }


    /// @brief Write the ply header to the output file
    void write_ply_header() {
        std::ofstream ofs(output_file_);
        ofs << "ply\n";
        ofs << "format ascii 1.0\n";
        ofs << "element vertex 0\n"; // Placeholder for vertex count
        ofs << "property float x\n";
        ofs << "property float y\n";
        ofs << "property float z\n";
        ofs << "property uchar red\n";
        ofs << "property uchar green\n";
        ofs << "property uchar blue\n";
        ofs << "end_header\n";
        ofs.close();

        // Write the semantic information to the txt file
        std::ofstream ofs_txt(semantic_txt_file_);
        ofs_txt.close();
    }
    
    /// @brief Write the ply data to the output file
    /// @param vertices The vertices to write
    void write_ply_data(const std::vector<Vertex>& vertices) {
        std::ofstream ofs(output_file_, std::ios_base::app);
        std::ofstream ofs_txt(semantic_txt_file_, std::ios_base::app);

        for (const auto& vertex : vertices) {
            ofs << vertex.x << " " << vertex.y << " " << vertex.z << " "
                << static_cast<int>(vertex.r) << " "
                << static_cast<int>(vertex.g) << " "
                << static_cast<int>(vertex.b) << "\n";
            
            if(!object_csv_.empty()){
                int label_id = get_label_from_color(vertex.r, vertex.g, vertex.b);
                ofs_txt << label_id << "\n";
            }
        }

        ofs.close();
        ofs_txt.close();
    }

    
    /// @brief Write the ply header to the output file. Color is not included
    void write_ply_header_no_color() {
        std::ofstream ofs(output_file_);
        ofs << "ply\n";
        ofs << "format ascii 1.0\n";
        ofs << "element vertex 0\n"; // Placeholder for vertex count
        ofs << "property float x\n";
        ofs << "property float y\n";
        ofs << "property float z\n";
        ofs << "end_header\n";
        ofs.close();

        // Write the semantic information to the txt file
        std::ofstream ofs_txt(semantic_txt_file_);
        ofs_txt.close();
    }

    /// @brief  Write the ply data to the output file. Color is not included
    /// @param vertices 
    void write_ply_no_color(const std::vector<Vertex>& vertices) {
        std::ofstream ofs(output_file_, std::ios_base::app);
        std::ofstream ofs_txt(semantic_txt_file_, std::ios_base::app);

        for (const auto& vertex : vertices) {
            ofs << vertex.x << " " << vertex.y << " " << vertex.z << "\n";

            if(!object_csv_.empty()){
                int label_id = get_label_from_color(vertex.r, vertex.g, vertex.b);
                ofs_txt << label_id << "\n";
            }
        }

        ofs.close();
        ofs_txt.close();
    }

    /// @brief Update the vertex count in the ply header. Used when all the vertices are written
    void update_ply_header() {
        std::ifstream ifs(output_file_);
        std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
        ifs.close();

        std::ofstream ofs(output_file_);
        size_t pos = content.find("element vertex 0");
        if (pos != std::string::npos) {
            content.replace(pos, std::string("element vertex 0").length(), "element vertex " + std::to_string(total_vertices_));
        }

        ofs << content;
        ofs.close();
    }


    std::string output_file_;
    std::string semantic_txt_file_;
    bool first_message_;

    size_t total_vertices_;
    int no_message_count_ = 0;

    float map_range_xy_;
    float map_range_z_;
    float voxel_size_;

    bool write_color_;

    std::vector<Vertex> full_vertices_last_frame_;

    ObjectInfoHandler object_info_handler_;
    std::string object_csv_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_mapping");

    if (argc < 7) {
        ROS_ERROR("Usage: %s <points_topic> <pose_topic> <output_file> <map_range_xy> <map_range_z> <voxel_size> <write_color> <object_csv> if voxel_size<=0, all the points in one frame will be saved.", argv[0]);
        return 1;
    }

    std::string object_csv = "";
    bool if_write_color = true;
    int write_color = 1;
    if (argc >= 8) {
        write_color = std::stoi(argv[7]);
    }

    if (argc >= 9) {
        object_csv = argv[8];
    }

    if (write_color == 0) {
        if_write_color = false;
    }

    // Create the file saving folder if it does not exist
    std::string output_file = argv[3];
    std::string folder = output_file.substr(0, output_file.find_last_of("/"));
    std::string command = "mkdir -p " + folder;
    system(command.c_str());

    PointCloudToPly converter(argv[1], argv[2], argv[3], std::stof(argv[4]), std::stof(argv[5]), std::stof(argv[6]), if_write_color, object_csv);

    return 0;
}
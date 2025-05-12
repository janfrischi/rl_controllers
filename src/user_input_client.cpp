#include <rclcpp/rclcpp.hpp>
#include "messages_fr3/srv/set_pose.hpp"
#include "messages_fr3/srv/set_param.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <array>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <thread>
#include <iostream>
#include <iomanip>

class CartesianControlClient {
public:
    CartesianControlClient(std::shared_ptr<rclcpp::Node> node) : node_(node) {
        // Create clients for the pose and parameter services
        pose_client_ = node_->create_client<messages_fr3::srv::SetPose>("set_pose");
        param_client_ = node_->create_client<messages_fr3::srv::SetParam>("set_param");

        // Wait for services to be available
        while (!pose_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for pose service");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting for pose service...");
        }

        while (!param_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for param service");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Waiting for param service...");
        }

        RCLCPP_INFO(node_->get_logger(), "Services are available!");

        // Initialize predefined poses [x, y, z, roll, pitch, yaw]
        predefined_poses_["home"] = {0.5, 0.0, 0.4, M_PI, 0.0, 0.0};
        predefined_poses_["left"] = {0.5, -0.3, 0.5, M_PI, 0.0, 0.0};
        predefined_poses_["right"] = {0.5, 0.3, 0.5, M_PI, 0.0, 0.0};
        predefined_poses_["forward"] = {0.7, 0.0, 0.5, M_PI, 0.0, 0.0};
        predefined_poses_["backward"] = {0.3, 0.0, 0.5, M_PI, 0.0, 0.0};
        predefined_poses_["up"] = {0.5, 0.0, 0.7, M_PI, 0.0, 0.0};
        predefined_poses_["down"] = {0.5, 0.0, 0.3, M_PI, 0.0, 0.0};
        predefined_poses_["rotate_x"] = {0.5, 0.0, 0.5, M_PI + 0.3, 0.0, M_PI_2};
        predefined_poses_["rotate_y"] = {0.5, 0.0, 0.5, M_PI, 0.3, M_PI_2};
        predefined_poses_["rotate_z"] = {0.5, 0.0, 0.5, M_PI, 0.0, M_PI_2 + 0.3};

        // Initialize predefined parameter sets [a, b, c, d, e, f]
        predefined_params_["stiff"] = {2.0, 0.5, 0.5, 2.0, 0.5, 0.5};
        predefined_params_["soft"] = {0.5, 0.2, 0.2, 0.5, 0.2, 0.2};
        predefined_params_["medium"] = {1.0, 0.3, 0.3, 1.0, 0.3, 0.3};
        predefined_params_["custom1"] = {1.5, 0.4, 0.4, 1.5, 0.4, 0.4};
        predefined_params_["custom2"] = {2.5, 0.6, 0.6, 2.5, 0.6, 0.6};

        // Initialize sequences
        sequences_["square"] = {"home", "left", "forward", "right", "home"};
        sequences_["triangle"] = {"home", "left", "right", "home"};
        sequences_["updown"] = {"home", "up", "down", "home"};
        sequences_["rotate"] = {"home", "rotate_x", "home", "rotate_y", "home", "rotate_z", "home"};
    }

    bool send_pose(double x, double y, double z, double roll, double pitch, double yaw) {
        auto request = std::make_shared<messages_fr3::srv::SetPose::Request>();
        request->x = x;
        request->y = y;
        request->z = z;
        request->roll = roll;
        request->pitch = pitch;
        request->yaw = yaw;

        RCLCPP_INFO(node_->get_logger(), "Sending pose: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
                   x, y, z, roll, pitch, yaw);

        auto result = pose_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Pose sent successfully: %d", result.get()->success);
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service setPose");
            return false;
        }
    }

    bool send_param(double a, double b, double c, double d, double e, double f) {
        auto request = std::make_shared<messages_fr3::srv::SetParam::Request>();
        request->a = a;
        request->b = b;
        request->c = c;
        request->d = d;
        request->e = e;
        request->f = f;

        RCLCPP_INFO(node_->get_logger(), "Sending parameters: a=%.2f, b=%.2f, c=%.2f, d=%.2f, e=%.2f, f=%.2f",
                   a, b, c, d, e, f);

        auto result = param_client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Parameters sent successfully: %d", result.get()->success);
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service setParam");
            return false;
        }
    }

    bool send_predefined_pose(const std::string& pose_name) {
        if (predefined_poses_.find(pose_name) == predefined_poses_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown pose: %s", pose_name.c_str());
            return false;
        }

        const auto& pose = predefined_poses_[pose_name];
        RCLCPP_INFO(node_->get_logger(), "Sending predefined pose: %s", pose_name.c_str());
        return send_pose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    }

    bool send_predefined_param(const std::string& param_name) {
        if (predefined_params_.find(param_name) == predefined_params_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown parameter set: %s", param_name.c_str());
            return false;
        }

        const auto& params = predefined_params_[param_name];
        RCLCPP_INFO(node_->get_logger(), "Sending predefined parameters: %s", param_name.c_str());
        return send_param(params[0], params[1], params[2], params[3], params[4], params[5]);
    }

    bool execute_sequence(const std::string& sequence_name, double delay_seconds = 2.0) {
        if (sequences_.find(sequence_name) == sequences_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown sequence: %s", sequence_name.c_str());
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "Executing sequence: %s", sequence_name.c_str());
        const auto& sequence = sequences_[sequence_name];

        for (const auto& pose_name : sequence) {
            bool success = send_predefined_pose(pose_name);
            if (!success) {
                RCLCPP_ERROR(node_->get_logger(), "Sequence failed at pose: %s", pose_name.c_str());
                return false;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(delay_seconds));
        }

        RCLCPP_INFO(node_->get_logger(), "Sequence %s completed successfully", sequence_name.c_str());
        return true;
    }

    bool execute_circular_motion(double center_x = 0.5, double center_y = 0.0, double center_z = 0.5,
                                double radius = 0.1, int steps = 8, const std::string& plane = "xy") {
        RCLCPP_INFO(node_->get_logger(), "Executing circular motion in %s plane with radius %.2f",
                   plane.c_str(), radius);

        // Default orientation
        double roll = M_PI, pitch = 0.0, yaw = M_PI_2;

        for (int i = 0; i <= steps; ++i) {  // +1 to close the circle
            double angle = 2.0 * M_PI * i / steps;
            double x = center_x, y = center_y, z = center_z;

            if (plane == "xy") {
                x = center_x + radius * std::cos(angle);
                y = center_y + radius * std::sin(angle);
            } else if (plane == "xz") {
                x = center_x + radius * std::cos(angle);
                z = center_z + radius * std::sin(angle);
            } else if (plane == "yz") {
                y = center_y + radius * std::cos(angle);
                z = center_z + radius * std::sin(angle);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Unknown plane: %s", plane.c_str());
                return false;
            }

            bool success = send_pose(x, y, z, roll, pitch, yaw);
            if (!success) {
                RCLCPP_ERROR(node_->get_logger(), "Circular motion failed at step %d", i);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        RCLCPP_INFO(node_->get_logger(), "Circular motion completed successfully");
        return true;
    }

    bool interpolate_poses(const std::string& start_pose_name, const std::string& end_pose_name,
                         int steps = 10, double delay_seconds = 0.5) {
        if (predefined_poses_.find(start_pose_name) == predefined_poses_.end() ||
            predefined_poses_.find(end_pose_name) == predefined_poses_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown pose name");
            return false;
        }

        const auto& start_pose = predefined_poses_[start_pose_name];
        const auto& end_pose = predefined_poses_[end_pose_name];

        RCLCPP_INFO(node_->get_logger(), "Interpolating from %s to %s",
                   start_pose_name.c_str(), end_pose_name.c_str());

        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            std::array<double, 6> interpolated_pose;

            for (size_t j = 0; j < 6; ++j) {
                interpolated_pose[j] = start_pose[j] + t * (end_pose[j] - start_pose[j]);
            }

            bool success = send_pose(
                interpolated_pose[0], interpolated_pose[1], interpolated_pose[2],
                interpolated_pose[3], interpolated_pose[4], interpolated_pose[5]
            );

            if (!success) {
                RCLCPP_ERROR(node_->get_logger(), "Interpolation failed at step %d", i);
                return false;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(delay_seconds));
        }

        RCLCPP_INFO(node_->get_logger(), "Interpolation completed successfully");
        return true;
    }

    void list_available_options() {
        std::cout << "\n=== Available Options ===" << std::endl;
        
        std::cout << "Predefined poses:" << std::endl;
        for (const auto& [name, pose] : predefined_poses_) {
            std::cout << "  " << std::left << std::setw(10) << name << ": [";
            for (size_t i = 0; i < pose.size(); ++i) {
                std::cout << std::fixed << std::setprecision(2) << pose[i];
                if (i < pose.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
        
        std::cout << "\nPredefined parameter sets:" << std::endl;
        for (const auto& [name, params] : predefined_params_) {
            std::cout << "  " << std::left << std::setw(10) << name << ": [";
            for (size_t i = 0; i < params.size(); ++i) {
                std::cout << std::fixed << std::setprecision(2) << params[i];
                if (i < params.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
        
        std::cout << "\nSequences:" << std::endl;
        for (const auto& [name, seq] : sequences_) {
            std::cout << "  " << std::left << std::setw(10) << name << ": [";
            for (size_t i = 0; i < seq.size(); ++i) {
                std::cout << seq[i];
                if (i < seq.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<messages_fr3::srv::SetPose>::SharedPtr pose_client_;
    rclcpp::Client<messages_fr3::srv::SetParam>::SharedPtr param_client_;
    
    std::map<std::string, std::array<double, 6>> predefined_poses_;
    std::map<std::string, std::array<double, 6>> predefined_params_;
    std::map<std::string, std::vector<std::string>> sequences_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("user_input_client");
    CartesianControlClient client(node);

    int choice = 0;
    std::string input;
    
    while (rclcpp::ok()) {
        std::cout << "\n=== Cartesian Impedance Control Client ===" << std::endl;
        std::cout << "1. Send predefined pose" << std::endl;
        std::cout << "2. Send custom pose" << std::endl;
        std::cout << "3. Execute sequence" << std::endl;
        std::cout << "4. Execute circular motion" << std::endl;
        std::cout << "5. Interpolate between poses" << std::endl;
        std::cout << "6. Change impedance parameters" << std::endl;
        std::cout << "7. List available options" << std::endl;
        std::cout << "8. Exit" << std::endl;
        
        std::cout << "Enter your choice: ";
        std::cin >> choice;
        
        try {
            if (choice == 1) {
                std::cout << "Enter pose name: ";
                std::cin >> input;
                client.send_predefined_pose(input);
                
            } else if (choice == 2) {
                double x, y, z, roll, pitch, yaw;
                std::cout << "Enter x position: ";
                std::cin >> x;
                std::cout << "Enter y position: ";
                std::cin >> y;
                std::cout << "Enter z position: ";
                std::cin >> z;
                std::cout << "Enter roll (radians): ";
                std::cin >> roll;
                std::cout << "Enter pitch (radians): ";
                std::cin >> pitch;
                std::cout << "Enter yaw (radians): ";
                std::cin >> yaw;
                
                client.send_pose(x, y, z, roll, pitch, yaw);
                
            } else if (choice == 3) {
                std::cout << "Enter sequence name: ";
                std::cin >> input;
                
                double delay;
                std::cout << "Enter delay between poses (seconds): ";
                std::cin >> delay;
                
                client.execute_sequence(input, delay);
                
            } else if (choice == 4) {
                double center_x, center_y, center_z, radius;
                int steps;
                std::string plane;
                
                std::cout << "Enter center x position (default 0.5): ";
                std::cin >> center_x;
                std::cout << "Enter center y position (default 0.0): ";
                std::cin >> center_y;
                std::cout << "Enter center z position (default 0.5): ";
                std::cin >> center_z;
                std::cout << "Enter radius (default 0.1): ";
                std::cin >> radius;
                std::cout << "Enter number of steps (default 8): ";
                std::cin >> steps;
                std::cout << "Enter plane (xy, xz, yz, default xy): ";
                std::cin >> plane;
                
                client.execute_circular_motion(center_x, center_y, center_z, radius, steps, plane);
                
            } else if (choice == 5) {
                std::string start_pose, end_pose;
                int steps;
                double delay;
                
                std::cout << "Enter start pose name: ";
                std::cin >> start_pose;
                std::cout << "Enter end pose name: ";
                std::cin >> end_pose;
                std::cout << "Enter number of steps (default 10): ";
                std::cin >> steps;
                std::cout << "Enter delay between steps (default 0.5): ";
                std::cin >> delay;
                
                client.interpolate_poses(start_pose, end_pose, steps, delay);
                
            } else if (choice == 6) {
                std::cout << "Enter parameter set name or 'custom' for custom values: ";
                std::cin >> input;
                
                if (input == "custom") {
                    double a, b, c, d, e, f;
                    std::cout << "Enter a: ";
                    std::cin >> a;
                    std::cout << "Enter b: ";
                    std::cin >> b;
                    std::cout << "Enter c: ";
                    std::cin >> c;
                    std::cout << "Enter d: ";
                    std::cin >> d;
                    std::cout << "Enter e: ";
                    std::cin >> e;
                    std::cout << "Enter f: ";
                    std::cin >> f;
                    
                    client.send_param(a, b, c, d, e, f);
                } else {
                    client.send_predefined_param(input);
                }
                
            } else if (choice == 7) {
                client.list_available_options();
                
            } else if (choice == 8) {
                break;
                
            } else {
                std::cout << "Invalid choice, please try again" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}
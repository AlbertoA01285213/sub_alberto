#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/int32.hpp"
#include <vector>
#include <string>
#include <Eigen/Dense>

using namespace std::chrono_literals;

// #include "model/checks.cpp"


class BezierNode : public rclcpp::Node
{
public:
    BezierNode() : Node("bezier")
    {
        // publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("bezier_points", 10);

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 10, std::bind(&BezierNode::pose_callback, this, std::placeholders::_1)
        );
        waypoint_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "bezier_waypoints", 10, std::bind(&BezierNode::waypoint_callback, this, std::placeholders::_1)
        );

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("robot_path", 10);



        // timer_ = this->create_wall_timer(1000ms, std::bind(&BezierNode::update, this));
        
        initialize();

    }


private:
    
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_subscriber_;
    // rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::PoseStamped initialPose;
    geometry_msgs::msg::PoseArray waypoints;

    const int pointsInBetween = 5;

    void initialize() { RCLCPP_INFO(this->get_logger(), "Nodo Bezier inicializado."); }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        this->pose = *pose;
    }

    void waypoint_callback(const geometry_msgs::msg::PoseArray::SharedPtr waypoints) {
        this->waypoints = *waypoints;
        // RCLCPP_WARN(this->get_logger(), "RECIBIDOS LOS WAYPOINTS");
        generateBezierPath();
    }

    std::vector<double> cubic_bezier_segment(const std::vector<double>& P, const std::vector<double>& a, 
        const std::vector<double>& b, int i, int pointsInBtw) {
        std::vector<double> result;
        for (double t = 0.0; t <= 1.0; t += 1.0 /(pointsInBtw+1.0)) {
            double gamma = pow(1 - t, 3) * P[i] + 3 * t * pow(1 - t, 2) * a[i] + 
                    3 * pow(t, 2) * (1 - t) * b[i] + pow(t, 3) * P[i + 1];
            result.push_back(gamma);
        }
        return result;
    }

    void printMatrix(const Eigen::MatrixXd& M, const std::string& name) {
        std::ostringstream oss;
        oss << M;
        RCLCPP_INFO(this->get_logger(), "Matrix %s:\n%s", name.c_str(), oss.str().c_str());
    }



    void generateBezierPath() {

        const int size = static_cast<int>(waypoints.poses.size());

        Eigen::Matrix<double, Eigen::Dynamic, 3> Pm;
        Pm.resize(size+1, 3);
        
        // Agregar pose inicial
        Pm(0,0) = 0.0;
        Pm(0,1) = 0.0;
        Pm(0,2) = 0.0;

        for (int i = 0; i < size; i++) {
            Pm(i+1, 0) = waypoints.poses[i].position.x;
            Pm(i+1, 1) = waypoints.poses[i].position.y;
            Pm(i+1, 2) = waypoints.poses[i].position.z;
        }

        // printMatrix(Pm, "puntos");
        
        const int n = Pm.rows() - 1;  // Number of segments

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
        I *= 4;
        I(0,0) = 2;
        I(n-1,n-1) = 7;
        for (int i = 0; i < n-1; i++) {
            I(i,i+1) = 1;
            I(i+1,i) = 1;
        }
        I(n-1,n-2) = 2;

        Eigen::MatrixXd Pfm(n, 3);

        Pfm(0,0) = Pm(0,0) + 2*Pm(1,0);
        Pfm(0,1) = Pm(0,1) + 2*Pm(1,1);
        Pfm(0,2) = Pm(0,2) + 2*Pm(1,2);

        for (int i = 1; i < n - 1; i++) {
            for (int j = 0; j < 3; j++) {
                Pfm(i,j) = 2*(2*Pm(i,j) + Pm(i+1,j));
            }
        }

        Pfm(n-1,0) = 8*Pm(n-1,0) + Pm(n,0);
        Pfm(n-1,1) = 8*Pm(n-1,1) + Pm(n,1);
        Pfm(n-1,2) = 8*Pm(n-1,2) + Pm(n,2);

        Eigen::MatrixXd A = I.fullPivLu().solve(Pfm);

        Eigen::MatrixXd b(n, 3);
        for (int i=0; i < n-1; i++) {
            for (int j=0; j<3; j++) {
                b(i,j) = 2*Pm(i+1,j) - A(i+1,j);
            }
        }
        b(n-1,0) = (A(n-1,0) + Pm(n,0))/2;
        b(n-1,1) = (A(n-1,1) + Pm(n,1))/2;
        b(n-1,2) = (A(n-1,2) + Pm(n,2))/2;

        double dt = 1.0 / (pointsInBetween+1);

        nav_msgs::msg::Path path;
        path.header = waypoints.header;

        geometry_msgs::msg::PoseStamped p;
        p.header = path.header;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;

        for (int i = 0; i < n; i++) {  // loop over segments
            for (double t = 0.0; t < 1.0; t += dt) { 
                p.pose.position.x = std::pow(1-t,3) * Pm(i,0) + 3*std::pow(1-t,2)*t*A(i,0)
                    + 3*(1-t)*t*t*b(i,0) + std::pow(t,3)*Pm(i+1,0);
                p.pose.position.y = std::pow(1-t,3) * Pm(i,1) + 3*std::pow(1-t,2)*t*A(i,1)
                    + 3*(1-t)*t*t*b(i,1) + std::pow(t,3)*Pm(i+1,1);
                p.pose.position.z = std::pow(1-t,3) * Pm(i,2) + 3*std::pow(1-t,2)*t*A(i,2)
                    + 3*(1-t)*t*t*b(i,2) + std::pow(t,3)*Pm(i+1,2);

                path.poses.push_back(p);

            }
        }

        // Agregar el último waypoint
        p.pose.position.x = std::pow(0,3) * Pm(n-1,0) + 3*std::pow(0,2)*A(n-1,0)
            + 3*(0)*b(n-1,0) + std::pow(1,3)*Pm(n,0);
        p.pose.position.y = std::pow(0,3) * Pm(n-1,1) + 3*std::pow(0,2)*A(n-1,1)
            + 3*(0)*b(n-1,1) + std::pow(1,3)*Pm(n,1);
        p.pose.position.z = std::pow(0,3) * Pm(n-1,2) + 3*std::pow(0,2)*A(n-1,2)
            + 3*(0)*b(n-1,2) + std::pow(1,3)*Pm(n,2);
        path.poses.push_back(p);
        
        path_publisher_->publish(path);
    
               

    }
    

    

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BezierNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include <std_msgs/msg/byte_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include <sstream>

using asio::ip::tcp;

class RtkNode : public rclcpp::Node
{
public:
    RtkNode()
        : Node("rtk_node"), io_context_(), socket_(io_context_), is_connected_(false)
    {
        RCLCPP_INFO(this->get_logger(), "RTK Node has been started.");

        // Declare parameters
        this->declare_parameter<std::string>("caster", "nrtk.big.go.id");
        this->declare_parameter<std::string>("port", "2101");
        this->declare_parameter<std::string>("mount_point", "Nearest-rtcm3");
        this->declare_parameter<std::string>("username", "heruka");
        this->declare_parameter<std::string>("password", "HERU");

        // Get parameters
        this->get_parameter("caster", caster_);
        this->get_parameter("port", port_);
        this->get_parameter("mount_point", mount_point_);
        this->get_parameter("username", username_);
        this->get_parameter("password", password_);

        RCLCPP_INFO(this->get_logger(), "Caster: %s, Port: %s, Mount point: %s, Username: %s", 
                    caster_.c_str(), port_.c_str(), mount_point_.c_str(), username_.c_str());

        // Initialize publisher
        rtcm_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("rtcm", 10);

        // Attempt connection every 5 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&RtkNode::try_connect, this)
        );
    }

    ~RtkNode()
    {
        RCLCPP_INFO(this->get_logger(), "RTK Node is shutting down.");
    }

private:
    // ASIO networking
    asio::io_context io_context_;
    tcp::socket socket_;

    // ROS 2 utilities
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr rtcm_pub_;

    // Parameters
    std::string caster_, port_, mount_point_, username_, password_;
    bool is_connected_;

    std::string base64_encode(const std::string &in)
    {
        static const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out;
        int val = 0, valb = -6;
        for (uint8_t c : in)
        {
            val = (val << 8) + c;
            valb += 8;
            while (valb >= 0)
            {
                out.push_back(chars[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }
        if (valb > -6)
            out.push_back(chars[((val << 8) >> valb) & 0x3F]);
        while (out.size() % 4)
            out.push_back('=');
        return out;
    }

    void try_connect()
    {
        if (is_connected_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Already connected to the caster.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Attempting to connect to caster: %s", caster_.c_str());

        try
        {
            tcp::resolver resolver(io_context_);
            auto endpoints = resolver.resolve(caster_, port_);
            asio::connect(socket_, endpoints);
            RCLCPP_INFO(this->get_logger(), "Connected to caster: %s", caster_.c_str());

            send_ntrip_request();
            read_ntrip_response();
            start_rtcm_read_loop();

            is_connected_ = true;
            RCLCPP_INFO(this->get_logger(), "RTCM data stream started.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Connection failed: %s", e.what());
        }
    }

    void send_ntrip_request()
    {
        std::string auth = username_ + ":" + password_;
        std::string base64_auth = base64_encode(auth);

        std::ostringstream request_stream;
        request_stream << "GET /" << mount_point_ << " HTTP/1.1\r\n"
                       << "Host: " << caster_ << "\r\n"
                       << "Ntrip-Version: Ntrip/2.0\r\n"
                       << "User-Agent: NTRIP ROS Client\r\n"
                       << "Authorization: Basic " << base64_auth << "\r\n"
                       << "\r\n";

        std::string request = request_stream.str();
        asio::write(socket_, asio::buffer(request));
    }

    void read_ntrip_response()
    {
        asio::streambuf response;
        asio::read_until(socket_, response, "\r\n\r\n");
        std::istream response_stream(&response);
        std::string status_line;
        std::getline(response_stream, status_line);

        if (status_line.find("200") == std::string::npos && status_line.find("ICY") == std::string::npos)
        {
            RCLCPP_ERROR(this->get_logger(), "NTRIP rejected connection: %s", status_line.c_str());
            socket_.close();
            is_connected_ = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Received valid NTRIP response.");
        }
    }

    void start_rtcm_read_loop()
    {
        std::thread([this]()
                    {
            try {
                while (rclcpp::ok() && socket_.is_open()) {
                    std::vector<char> buffer(1024);
                    size_t len = socket_.read_some(asio::buffer(buffer));

                    if (len > 0) {
                        auto msg = std_msgs::msg::ByteMultiArray();
                        msg.data.insert(msg.data.end(), buffer.begin(), buffer.begin() + len);
                        rtcm_pub_->publish(msg);

                        RCLCPP_INFO(this->get_logger(), "Received %zu bytes RTCM", len);
                    }
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "RTCM read error: %s", e.what());
                socket_.close();
                is_connected_ = false;
            } })
            .detach();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RtkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

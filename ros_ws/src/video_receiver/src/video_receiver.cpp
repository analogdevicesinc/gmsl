#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <array>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <map>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>

using boost::asio::ip::udp;

typedef struct _RTPHeader {
    unsigned int csrc_count : 4;
    unsigned int extension : 1;
    unsigned int padding : 1;
    unsigned int version : 2;
    unsigned int payload_type : 7;
    unsigned int marker : 1;
    unsigned int seq : 16;
    unsigned int timestamp : 32;
    unsigned int ssrc : 32;
    uint8_t csrclist[4];
} RTPHeader;

struct PayloadHeader {
    uint16_t length;
    uint8_t F;
    uint16_t line_no;
    uint8_t C;
    uint16_t offset;
};

struct Packet {
    std::vector<uint8_t> data;
    size_t length;
    unsigned int seq;
};

class RTPVideoReceiverNode : public rclcpp::Node {
public:
    RTPVideoReceiverNode() 
    : Node("rtp_video_receiver_node"), 
      socket_(io_service_, udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 5004)),
      stop_processing_(false) {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("video_frames", 100);
        processing_thread_ = std::thread(&RTPVideoReceiverNode::process_packets, this);
    }

    ~RTPVideoReceiverNode() {
        stop_processing_ = true;
        cond_var_.notify_all();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

    void receive_packets() {
        udp::endpoint sender_endpoint; 
        std::array<uint8_t, 16334> recv_buffer; 

        while (rclcpp::ok()) {
            size_t length = socket_.receive_from(boost::asio::buffer(recv_buffer), sender_endpoint);
            if (length > 0) {
                RTPHeader rtp_header = parse_rtp_header(recv_buffer.data(), length);
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                packet_buffer_.push({std::vector<uint8_t>(recv_buffer.begin(), recv_buffer.begin() + length), length, rtp_header.seq});
                cond_var_.notify_one();
            }
        }
    }

private:
    boost::asio::io_service io_service_;
    udp::socket socket_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    int width = 1280;
    int height = 720;

    std::queue<Packet> packet_buffer_;
    std::mutex buffer_mutex_;
    std::condition_variable cond_var_;
    std::thread processing_thread_;
    bool stop_processing_;
    std::map<int, std::vector<uint8_t>> frame_data_;

    void process_packets() {
        while (!stop_processing_) {
            std::unique_lock<std::mutex> lock(buffer_mutex_);
            cond_var_.wait(lock, [this] { return !packet_buffer_.empty() || stop_processing_; });

            if (stop_processing_) break;

            Packet packet = packet_buffer_.front();
            packet_buffer_.pop();
            lock.unlock();

            std::vector<uint8_t> payload = extract_payload(packet.data.data(), packet.length);
            auto [extended_sequence_number, headers] = extract_payload_headers(payload);

            size_t payload_header_length = 2 + headers.size() * 6;
            size_t offset = payload_header_length;

            for (const auto& header : headers) {
                if (offset + header.length <= payload.size()) {
                    std::vector<uint8_t> line_data(payload.begin() + offset, payload.begin() + offset + header.length);

                    if (frame_data_.find(header.line_no) == frame_data_.end()) {
                        frame_data_[header.line_no].resize(2 * this->width);
                    }

                    if (header.length < (2 * this->width)) {
                        auto& frame_data = frame_data_[header.line_no];
                        std::copy(line_data.begin(), line_data.end(), frame_data.begin() + 2 * header.offset);
                    } else {
                        frame_data_[header.line_no] = std::move(line_data);
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Offset and length exceed payload size");
                }

                offset += header.length;

                if (header.line_no == (this->height - 1)) {
                    //RCLCPP_INFO(this->get_logger(), "End of Frame");

                    size_t total_size = 0;
                    for (const auto& pair : frame_data_) {
                        total_size += pair.second.size();
                    }

                    std::vector<uint8_t> frame_data;
                    frame_data.reserve(total_size);

                    for (const auto& pair : frame_data_) {
                        frame_data.insert(frame_data.end(), pair.second.begin(), pair.second.end());
                    }

                    cv::Mat yuv_image(this->height, this->width, CV_8UC2, frame_data.data());
                    cv::Mat bgr_image;
                    cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_UYVY);
                    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_image).toImageMsg();
                    image_pub_->publish(*img_msg);

                    frame_data_.clear();
                }
            }
        }
    }

    RTPHeader parse_rtp_header(const uint8_t* data, size_t length) {
        RTPHeader header = {};
        if (length < 12) {
            RCLCPP_ERROR(this->get_logger(), "Received packet too short to contain RTP header.");
            return header;
        }

        header.version = (data[0] >> 6) & 0x03;
        header.padding = (data[0] >> 5) & 0x01;
        header.extension = (data[0] >> 4) & 0x01;
        header.csrc_count = data[0] & 0x0F;
        header.marker = (data[1] >> 7) & 0x01;
        header.payload_type = data[1] & 0x7F;
        header.seq = (data[2] << 8) | data[3];
        header.timestamp = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
        header.ssrc = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];

        return header;
    }

    std::vector<uint8_t> extract_payload(const uint8_t* data, int length) {
        int rtp_header_length = 12;
        std::vector<uint8_t> payload(data + rtp_header_length, data + length);
        return payload;
    }

    std::pair<uint16_t, std::vector<PayloadHeader>> extract_payload_headers(const std::vector<uint8_t>& payload) {
        std::vector<PayloadHeader> headers;
        size_t payload_len = payload.size();
        size_t index = 0;

        if (payload_len < 2) {
            throw std::runtime_error("Payload is less than 2 bytes");
        }

        uint16_t extended_sequence_number = (payload[index] << 8) | payload[index + 1];
        index += 2;
        payload_len -= 2;
        bool cont = true;

        do {
            if (payload_len < 4) {
                throw std::runtime_error("Payload is less than 4 bytes");
            }

            PayloadHeader header;
            header.length = (payload[index] << 8) | payload[index + 1];
            header.F = payload[index + 2] >> 7;
            header.line_no = ((payload[index + 2] & 0x7F) << 8) | payload[index + 3];
            header.C = payload[index + 4] >> 7;
            header.offset = ((payload[index + 4] & 0x7F) << 8) | payload[index + 5];

            headers.push_back(header);
            cont = header.C;

            index += 6;
            payload_len -= 6;

        } while (cont);

        return {extended_sequence_number, headers};
    }
    
   void print_header(const RTPHeader& header) {
        RCLCPP_INFO(this->get_logger(), "RTP Header: V=%u, P=%d, X=%d, CC=%u, M=%d, PT=%u, Seq=%u, TS=%u, SSRC=%u",
                    header.version, header.padding, header.extension, header.csrc_count, header.marker, header.payload_type, header.seq, header.timestamp, header.ssrc);
    }

    void print_payload_header(const PayloadHeader& header) {
        RCLCPP_INFO(this->get_logger(), "Length: %d | F: %d | Line No: %d | C: %d | Offset: %d",
                    header.length, static_cast<int>(header.F), header.line_no, static_cast<int>(header.C), header.offset);
    }

    void write_frame_data_to_file(const std::map<int, std::vector<uint8_t>>& frame_data_) {
        std::filesystem::path file_path = std::filesystem::path(__FILE__).parent_path() / "frame_data.txt";
        std::ofstream file(file_path);
        if (file.is_open()) {
            file << "End of Frame\n";
            for (const auto& pair : frame_data_) {
                file << std::dec << "Line No: " << pair.first << " | Length: " << pair.second.size() << "\n";
                for (const auto& byte : pair.second) {
                    file << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                }
                file << "\n";
            }
            file.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RTPVideoReceiverNode>();
    std::thread receive_thread([&]() { node->receive_packets(); });
    rclcpp::spin(node);
    rclcpp::shutdown();
    receive_thread.join();
    return 0;
}

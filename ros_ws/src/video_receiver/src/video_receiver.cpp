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
    unsigned int csrclist : 32;
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
    unsigned int marker;
};

class RTPVideoReceiverNode : public rclcpp::Node {
public:
    RTPVideoReceiverNode()
    : Node("rtp_video_receiver_node"),
      ip(this->declare_parameter<std::string>("ip", "10.42.0.1")),
      port(this->declare_parameter<int>("port", 8420)),
      width(this->declare_parameter<int>("width", 1920)),
      height(this->declare_parameter<int>("height", 1280)),
      topic(this->declare_parameter<std::string>("topic", "cam0")),
      timestamp_config(this->declare_parameter<int>("timestamp_config", 0)),
      socket_(io_service_, udp::endpoint(boost::asio::ip::address::from_string(ip), port)),
      image_pub_(this->create_publisher<sensor_msgs::msg::Image>(topic, 100)),
      processing_thread_(&RTPVideoReceiverNode::process_packets, this),
      stop_processing_(false) {
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
                packet_buffer_.push({std::vector<uint8_t>(recv_buffer.begin(), recv_buffer.begin() + length), length, rtp_header.seq, rtp_header.marker});
                cond_var_.notify_one();
            }
        }
    }

private:
    std::string ip;
    int port;
    int width;
    int height;
    std::string topic;
    int timestamp_config; // 0 - default, 1 - custom (header: timestamp + ssrc + csrclist)
    boost::asio::io_service io_service_;
    udp::socket socket_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::queue<Packet> packet_buffer_;
    std::mutex buffer_mutex_;
    std::condition_variable cond_var_;
    std::thread processing_thread_;
    bool stop_processing_;

    void process_packets() {
        const size_t EXT_SEQ_NO_LENGTH = 2;
        const size_t PAYLOAD_HEADER_LENGTH = 6;
        const size_t FRAME_WIDTH = 2 * this->width;
        std::vector<uint8_t> payload;
        std::pair<int, std::vector<PayloadHeader>> payload_headers;
        size_t payload_offset;
        std::vector<uint8_t> frame_data(this->height * FRAME_WIDTH, 0);
        std::vector<std::vector<uint8_t>> frame_data_(this->height, std::vector<uint8_t>(FRAME_WIDTH, 0));
        std::vector<uint8_t> line_data;
        cv::Mat yuv_image;
        cv::Mat bgr_image;
        sensor_msgs::msg::Image::SharedPtr img_msg;

        while (!stop_processing_) {
            std::unique_lock<std::mutex> lock(buffer_mutex_);
            cond_var_.wait(lock, [this] { return !packet_buffer_.empty() || stop_processing_; });

            if (stop_processing_) {
                RCLCPP_WARN(this->get_logger(), "Processing stopped");
                break;
            }

            Packet packet = std::move(packet_buffer_.front());
            packet_buffer_.pop();

            lock.unlock();

            payload = extract_payload(packet.data.data(), packet.length);
            payload_headers = extract_payload_headers(payload);
            payload_offset = EXT_SEQ_NO_LENGTH + payload_headers.second.size() * PAYLOAD_HEADER_LENGTH;

            for (const auto& header : payload_headers.second) {
                if (payload_offset + header.length > payload.size()) {
                    RCLCPP_ERROR(this->get_logger(), "Offset and length exceed payload size");
                    continue;
                }

                line_data.assign(payload.begin() + payload_offset, payload.begin() + payload_offset + header.length);
                std::vector<uint8_t>& frame_data_line = frame_data_[header.line_no];

                if (header.length < FRAME_WIDTH) {
                    std::copy(line_data.begin(), line_data.end(), frame_data_line.begin() + 2 * header.offset);
                } else {
                    frame_data_line = std::move(line_data);
                }

                payload_offset += header.length;

                if (packet.marker == 1 || header.line_no == (this->height - 1)) {
                    for (size_t i = 0; i < frame_data_.size(); ++i) {
                        std::copy(frame_data_[i].begin(), frame_data_[i].end(), frame_data.begin() + i * FRAME_WIDTH);
                    }

                    yuv_image = cv::Mat(this->height, this->width, CV_8UC2, frame_data.data());
                    cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_UYVY);
                    img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_image).toImageMsg();
                    image_pub_->publish(*img_msg);
                }
            }
        }
    }

    RTPHeader parse_rtp_header(const uint8_t* data, size_t length) {
        RTPHeader header = {};
        int min_length = static_cast<int>((this->timestamp_config) ? 16 : 12);
        if (static_cast<int>(length) < min_length) {
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
        header.csrclist = (this->timestamp_config) ? ((data[12] << 24) | (data[13] << 16) | (data[14] << 8) | data[15]) : 0;

        return header;
    }

    std::vector<uint8_t> extract_payload(const uint8_t* data, int length) {
        int rtp_header_length = (this->timestamp_config) ? 16 : 12;
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
        RCLCPP_INFO(this->get_logger(), "RTP Header: V=%u, P=%d, X=%d, CC=%u, M=%d, PT=%u, Seq=%u, TS=%u, SSRC=%u, CSRCL=%u",
                    header.version, header.padding, header.extension, header.csrc_count, header.marker, header.payload_type, header.seq, header.timestamp, header.ssrc, header.csrclist);
    }

    void print_payload_header(const PayloadHeader& header) {
        RCLCPP_INFO(this->get_logger(), "Length: %d | F: %d | Line No: %d | C: %d | Offset: %d",
                    header.length, static_cast<int>(header.F), header.line_no, static_cast<int>(header.C), header.offset);
    }

    void print_timestamp_date(const RTPHeader& header) {
        uint64_t timestamp_in_seconds = (static_cast<uint64_t>(header.timestamp) << 16) | (header.ssrc >> 16);
        uint32_t nano_sec = (header.ssrc & 0x0000FFFF) | (header.csrclist & 0xFFFF0000);
        uint16_t fraction_nano_sec = (header.csrclist << 16) & 0xFFFF;

        std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(timestamp_in_seconds);
        std::time_t tt = std::chrono::system_clock::to_time_t(tp);
        std::tm* tm = std::gmtime(&tt);

        // Print the date
        RCLCPP_INFO(this->get_logger(), "Timestamp Date: %d-%02d-%02d %02d:%02d:%02d",
                    tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

        // Print the nano seconds and fraction nano seconds
        RCLCPP_INFO(this->get_logger(), "Nano Sec: %u | Fraction Nano Sec: %u", nano_sec, fraction_nano_sec);
    }

    void write_frame_data_to_file(const std::vector<std::vector<uint8_t>>& frame_data_) {
        std::filesystem::path file_path = std::filesystem::path(__FILE__).parent_path() / "frame_data.txt";
        std::ofstream file(file_path);
        if (file.is_open()) {
            file << "End of Frame\n";
            for (size_t i = 0; i < frame_data_.size(); ++i) {
                file << std::dec << "Line No: " << i << " | Length: " << frame_data_[i].size() << "\n";
                for (const auto& byte : frame_data_[i]) {
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

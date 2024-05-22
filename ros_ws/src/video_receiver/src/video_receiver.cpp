#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <array>
#include <iostream>
#include <fstream>
#include <filesystem>

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

class RTPVideoReceiverNode : public rclcpp::Node {
public:
    RTPVideoReceiverNode() 
    : Node("rtp_video_receiver_node"), 
      socket_(io_service_, udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 5004)) {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("video_frames", 10);
    }

void receive_packets() {
    udp::endpoint sender_endpoint; 
    std::array<uint8_t, 16334> recv_buffer; 
    PayloadHeader payload_header;
    std::map<int, std::vector<uint8_t>> frame_data_;

    while (rclcpp::ok()) {
        size_t length = socket_.receive_from(boost::asio::buffer(recv_buffer), sender_endpoint);
        if (length > 0) {
            // Extract RTP header
            RTPHeader header = parse_rtp_header(recv_buffer.data(), length);

            // Print the RTP header for debugging
            //print_header(header);
        
            // Extract payload and append to frame_data
            std::vector<uint8_t> payload = extract_payload(recv_buffer.data(), length, this->get_logger());
            // Extract the payload headers for each video data line
            auto [extended_sequence_number, headers] = extract_payload_headers(payload);

            // Calculate the initial offset 
            //(2 bytes ext seq no + 6 bytes(2B length, 2B line no., 2B offset) header for each line)
            size_t payload_header_length = 2 + headers.size() * 6;
            // Offset to the start of the payload data
            size_t offset = payload_header_length;

            for (const auto& header : headers) {
                // Print the payload header for each line
                //print_payload_header(header, this->get_logger());

                // Check if the offset and length are within the payload size
                if (offset + header.length <= payload.size()) {
                    // Extract the line from the payload
                    std::vector<uint8_t> line_data(payload.begin() + offset, payload.begin() + offset + header.length);

                    // If the line number is not in the map, initialize it
                    if (frame_data_.find(header.line_no) == frame_data_.end()) {
                        frame_data_[header.line_no].resize(2 * this->width);
                    }

                    // If the length is less than 2 * this->width, this means that the line is incomplete
                    if (header.length < (2 * this->width)) {
                        // Insert the chunk data at the position given by header.offset
                        auto& frame_data = frame_data_[header.line_no];
                        std::copy(line_data.begin(), line_data.end(), frame_data.begin() + 2 * header.offset);
                    } else {
                        // Otherwise, the line is complete, so directly copy the chunk data into frame_data_
                        frame_data_[header.line_no] = std::move(line_data);
                    }
                } else {
                    // Handle the error
                    RCLCPP_ERROR(this->get_logger(), "Offset and length exceed payload size");
                }

                // Increment the offset by header.length, move to the next line
                offset += header.length;

                if (header.line_no == (this->height - 1)) {
                    // Print the end of frame message
                    RCLCPP_INFO(this->get_logger(), "End of Frame");

                    // Convert the map to a single vector
                    std::vector<uint8_t> frame_data;
                    for (const auto& pair : frame_data_) {
                        frame_data.insert(frame_data.end(), pair.second.begin(), pair.second.end());
                    }

                    //write_frame_data_to_file(frame_data_);

                    // Convert the vector to a cv::Mat
                    cv::Mat yuv_image(this->height, this->width, CV_8UC2, frame_data.data());

                    // Convert the YUV422 image to BGR
                    cv::Mat bgr_image;
                    cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_UYVY);

                    // Convert the BGR image to a sensor_msgs::Image
                    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_image).toImageMsg();

                    // Publish the Image message
                    image_pub_->publish(*img_msg);

                    // Clear the frame_data_
                    frame_data_.clear();
                }
            }
        }
    }
}

private:
    boost::asio::io_service io_service_;
    udp::socket socket_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    int width = 1280; // later to be set in a launch file
    int height = 720;

    void write_frame_data_to_file(const std::map<int, std::vector<uint8_t>>& frame_data_) {
        // Get the directory of the current source file
        std::filesystem::path current_file_path(__FILE__);
        std::filesystem::path dir_path = current_file_path.parent_path();

        // Append the desired filename to the directory path
        std::filesystem::path file_path = dir_path / "frame_data.txt";

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

    void print_header(const RTPHeader& header) {
        RCLCPP_INFO(this->get_logger(), "RTP Header: V=%u, P=%d, X=%d, CC=%u, M=%d, PT=%u, Seq=%u, TS=%u, SSRC=%u",
                    header.version, header.padding, header.extension, header.csrc_count, header.marker, header.payload_type, header.seq, header.timestamp, header.ssrc);
    }

    std::vector<uint8_t> extract_payload(const uint8_t* data, int length, rclcpp::Logger logger) {

    int rtp_header_length = 12;
    std::vector<uint8_t> payload(data + rtp_header_length, data + length);

    // Return the payload
    return payload;
    }

    void print_payload_header(const PayloadHeader& header, rclcpp::Logger logger) {
        if (header.line_no > 0) { // print also lines to check heght
                RCLCPP_INFO(logger, "Length: %d | F: %d | Line No: %d | C: %d | Offset: %d",
                    header.length, static_cast<int>(header.F), header.line_no, static_cast<int>(header.C), header.offset);
        }
    }

    std::pair<uint16_t, std::vector<PayloadHeader>> extract_payload_headers(const std::vector<uint8_t>& payload) {
        std::vector<PayloadHeader> headers;
        size_t payload_len = payload.size();
        size_t index = 0;

        if (payload_len < 2) {
            throw std::runtime_error("Payload is less than 2 bytes");
        }

        // Extract the unique extended sequence number
        uint16_t extended_sequence_number = (payload[index] << 8) | payload[index + 1];
        index += 2;
        payload_len -= 2;
        bool cont = true;

        do {
            if (payload_len < 4) {
                throw std::runtime_error("Payload is less than 4 bytes");
            }

            PayloadHeader header;
            // Extract the header
            header.length = (payload[index] << 8) | payload[index + 1];
            header.F = payload[index + 2] >> 7;
            header.line_no = ((payload[index + 2] & 0x7F) << 8) | payload[index + 3];
            header.C = payload[index + 4] >> 7;
            header.offset = ((payload[index + 4] & 0x7F) << 8) | payload[index + 5];

            headers.push_back(header);

            // Check if there is more data
            cont = header.C;

            index += 6;
            payload_len -= 6;

        } while (cont);

        return {extended_sequence_number, headers};
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RTPVideoReceiverNode>();
    node->receive_packets();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

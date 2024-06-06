#include <petra_communication/Keyboard.h>

Keyboard::Keyboard() : Node("Keyboard")
{
    input_string_publisher_ = create_publisher<std_msgs::msg::String>("InputString", 10);

    stop_publisher_ = create_publisher<std_msgs::msg::Empty>("Stop", 10);

    display_string_subscription_ = create_subscription<std_msgs::msg::String>("DisplayString", 10, [&](const std_msgs::msg::String::SharedPtr msg)
                                                                              {
        std::stringstream stream(msg->data);

        while (stream.good())
        {
            std::string line;
            std::getline(stream, line);
            RCLCPP_INFO(get_logger(), "Recieved: '%s'", line.c_str());
        } });

    RCLCPP_INFO(get_logger(), "Keyboard node started successfully. \nType text here to publish in ROS2. \nCommands: /stop");
}

void Keyboard::io_thread()
{
    while (rclcpp::ok())
    {
        std::string input;
        std::cin >> input;
        std_msgs::msg::String message = std_msgs::msg::String();
        message.data = input;

        if (input.at(0) == '/')
        {
            if (input == "/stop")
            {
                RCLCPP_WARN(get_logger(), "Executing command: '%s'", input.c_str());

                stop_publisher_->publish(std_msgs::msg::Empty());
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Unknown command: '%s'", input.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Publish:  '%s'", input.c_str());

            input_string_publisher_->publish(message);
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    Keyboard key;
    std::thread{std::bind(&Keyboard::io_thread, &key)}.detach();
    std::shared_ptr<Keyboard> node(&key);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
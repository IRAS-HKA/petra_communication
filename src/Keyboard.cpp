#include <petra_communication/Keyboard.h>

Keyboard::Keyboard() : Node("Keyboard")
{
    input_string_publisher_ = create_publisher<std_msgs::msg::String>("InputString", 10);

    password_string_publisher_ = create_publisher<std_msgs::msg::String>("Password_str", 10);

    barcode_string_publisher_ = create_publisher<std_msgs::msg::String>("Barcode_str", 10);

    stop_publisher_ = create_publisher<std_msgs::msg::Empty>("Stop", 10);

    RCLCPP_INFO(get_logger(), "Keyboard node started, type text to publish in ROS. \nCommands: /stop, /b<barcode>, /p<password>");
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
            else if (input.at(1) == 'p')
            {
                RCLCPP_INFO(get_logger(), "Password sent");

                password_string_publisher_->publish(message);
            }
            else if (input.at(1) == 'b')
            {
                RCLCPP_INFO(get_logger(), "Barcode: '%s'", input.c_str());

                barcode_string_publisher_->publish(message);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Unknown command: '%s'", input.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Publishing: '%s'", input.c_str());

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
#include <petra_communication/Screen.h>

Screen::Screen() : Node("Screen")
{
    display_string_subscription_ = create_subscription<std_msgs::msg::String>("DisplayString", 10, [&](const std_msgs::msg::String::SharedPtr msg)
                                                                              {
        std::stringstream stream(msg->data);

        while (stream.good())
        {
            std::string line;
            std::getline(stream, line);
            std::cout << line << std::endl;
        } });
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Screen>());
    rclcpp::shutdown();

    return 0;
}
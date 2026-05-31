#include <cone_fusion/cone_fusion.hpp>
#include <unistd.h>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing cone_fusion process.\n";
        rclcpp::shutdown();
    }
}


int main(int argc, char* argv[])
{
  signal(SIGINT, handleSignal);
  /* node initialization */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ConeFusion>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;

}
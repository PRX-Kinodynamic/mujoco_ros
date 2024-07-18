#pragma once

namespace utils
{
template <typename T>
static std::vector<T> split(const std::string str, const char delimiter)
{
  std::vector<T> result;
  std::istringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delimiter))
  {
    if (token.size() > 0)
    {
      std::istringstream ti(token);
      T x;
      if ((ti >> x))
        result.push_back(x);
    }
  }
  return result;
}

template <typename T>
void print_container(const std::string& name, const T& container)
{
  std::cout << name << ": ";
  for (auto e : container)
  {
    std::cout << e << ", ";
  }
  std::cout << std::endl;
}

static std::string timestamp()
{
  auto t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::stringstream strstr{};
  strstr << std::put_time(&tm, "%y%m%d_%H%M%S");
  return strstr.str();
}

template <typename TopicConstPtr>
void shutdown_callback(const TopicConstPtr& msg)
{
  // std::cout << "Shut"
  ros::Rate rate(2);
  rate.sleep();
  ros::shutdown();
}

}  // namespace utils
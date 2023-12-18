#define GET_VARIABLE_NAME(Variable) (#Variable)
#define DBG printf("%s: %d\n", __PRETTY_FUNCTION__, __LINE__);
namespace perception
{
// template<typename M, typename = std::enable_if_t<
// typename = decltype(std::declval<std::ostream>() << std::declval<typename M::mapped_type>())>
//
template <typename T>
void get_param_and_check(ros::NodeHandle& nh, const std::string& var_name, T& var)
{
  if (!nh.getParam(var_name, var))
  {
    std::cout << var_name << " parameter is needed." << std::endl;
    exit(-1);
  }
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
}  // namespace perception
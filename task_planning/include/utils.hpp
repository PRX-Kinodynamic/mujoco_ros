#include <sstream>
#include <vector>

template <typename T>
std::vector<T> string_to_vector(const std::string& s, char delimiter)
{
  std::vector<T> tokens;
  std::string token;
  std::istringstream tokenStream(s.substr(1, s.size() - 2));
  while (std::getline(tokenStream, token, delimiter))
  {
    tokens.push_back(std::stod(token));
  }
  return tokens;
}
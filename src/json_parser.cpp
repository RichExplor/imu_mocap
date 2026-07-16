#include "imu_mocap/json_parser.hpp"
#include <regex>
#include <string>
#include <vector>

SimpleJsonParser::SimpleJsonParser(const std::string& json_str) : json_(json_str) {
}

std::string SimpleJsonParser::getString(const std::string& key, const std::string& default_val) {
  std::string pattern = "\"" + key + "\"\\s*:\\s*\"([^\"]*)\"";
  std::regex  re(pattern);
  std::smatch match;
  if (std::regex_search(json_, match, re)) {
    return match[1].str();
  }
  return default_val;
}

int SimpleJsonParser::getInt(const std::string& key, int default_val) {
  std::string pattern = "\"" + key + "\"\\s*:\\s*(-?\\d+)";
  std::regex  re(pattern);
  std::smatch match;
  if (std::regex_search(json_, match, re)) {
    return std::stoi(match[1].str());
  }
  return default_val;
}

double SimpleJsonParser::getDouble(const std::string& key, double default_val) {
  std::string pattern = "\"" + key + "\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?(?:[eE][+-]?\\d+)?)";
  std::regex  re(pattern);
  std::smatch match;
  if (std::regex_search(json_, match, re)) {
    return std::stod(match[1].str());
  }
  return default_val;
}

bool SimpleJsonParser::getBool(const std::string& key, bool default_val) {
  std::string pattern = "\"" + key + "\"\\s*:\\s*(true|false)";
  std::regex  re(pattern);
  std::smatch match;
  if (std::regex_search(json_, match, re)) {
    return match[1].str() == "true";
  }
  return default_val;
}

std::vector<std::string> SimpleJsonParser::getStringArray(const std::string& key) {
  std::vector<std::string> result;
  std::string              array_content = extractArray(key);
  if (array_content.empty())
    return result;

  std::regex item_re("\"([^\"]*)\"");
  auto       begin = std::sregex_iterator(array_content.begin(), array_content.end(), item_re);
  auto       end   = std::sregex_iterator();
  for (auto it = begin; it != end; ++it) {
    result.push_back(it->str(1));
  }
  return result;
}

std::vector<bool> SimpleJsonParser::getBoolArray(const std::string& key) {
  std::vector<bool> result;
  std::string       array_content = extractArray(key);
  if (array_content.empty())
    return result;

  std::regex item_re("(true|false)");
  auto       begin = std::sregex_iterator(array_content.begin(), array_content.end(), item_re);
  auto       end   = std::sregex_iterator();
  for (auto it = begin; it != end; ++it) {
    result.push_back(it->str(1) == "true");
  }
  return result;
}

std::vector<std::vector<double>> SimpleJsonParser::getDouble2DArray(const std::string& key) {
  std::vector<std::vector<double>> result;
  std::string                      array_content = extractArray(key);
  if (array_content.empty())
    return result;

  // 逐层解析内部数组: [...], [...], ...
  size_t i = 0;
  while (i < array_content.size()) {
    // 跳过空白和逗号
    while (i < array_content.size() &&
           (array_content[i] == ' ' || array_content[i] == ',' || array_content[i] == '\n' ||
            array_content[i] == '\t' || array_content[i] == '\r')) {
      i++;
    }
    if (i >= array_content.size() || array_content[i] != '[')
      break;

    // 通过括号深度匹配找到对应的闭合 ]
    int    depth = 0;
    size_t j     = i;
    while (j < array_content.size()) {
      if (array_content[j] == '[')
        depth++;
      else if (array_content[j] == ']') {
        depth--;
        if (depth == 0)
          break;
      }
      j++;
    }

    if (depth != 0 || j >= array_content.size())
      break;

    // 提取内部元素 [a,b,c] -> a,b,c
    std::string inner_content = array_content.substr(i + 1, j - i - 1);

    // 解析数字
    std::vector<double> row;
    std::regex          num_re("(-?\\d+(?:\\.\\d+)?(?:[eE][+-]?\\d+)?)");
    auto                num_begin = std::sregex_iterator(inner_content.begin(), inner_content.end(), num_re);
    auto                num_end   = std::sregex_iterator();
    for (auto nit = num_begin; nit != num_end; ++nit) {
      row.push_back(std::stod(nit->str(1)));
    }
    if (!row.empty()) {
      result.push_back(row);
    }

    i = j + 1;
  }

  return result;
}

std::vector<std::vector<int>> SimpleJsonParser::getInt2DArray(const std::string& key) {
  std::vector<std::vector<int>> result;
  std::string                   array_content = extractArray(key);
  if (array_content.empty())
    return result;

  size_t i = 0;
  while (i < array_content.size()) {
    while (i < array_content.size() &&
           (array_content[i] == ' ' || array_content[i] == ',' || array_content[i] == '\n' ||
            array_content[i] == '\t' || array_content[i] == '\r')) {
      i++;
    }
    if (i >= array_content.size() || array_content[i] != '[')
      break;

    int    depth = 0;
    size_t j     = i;
    while (j < array_content.size()) {
      if (array_content[j] == '[')
        depth++;
      else if (array_content[j] == ']') {
        depth--;
        if (depth == 0)
          break;
      }
      j++;
    }

    if (depth != 0 || j >= array_content.size())
      break;

    std::string inner_content = array_content.substr(i + 1, j - i - 1);

    std::vector<int> row;
    std::regex       num_re("(-?\\d+)");
    auto             num_begin = std::sregex_iterator(inner_content.begin(), inner_content.end(), num_re);
    auto             num_end   = std::sregex_iterator();
    for (auto nit = num_begin; nit != num_end; ++nit) {
      row.push_back(std::stoi(nit->str(1)));
    }
    if (!row.empty()) {
      result.push_back(row);
    }

    i = j + 1;
  }

  return result;
}

std::string SimpleJsonParser::extractArray(const std::string& key) {
  // 定位 key 的位置
  std::string key_pattern = "\"" + key + "\"\\s*:";
  std::regex  key_re(key_pattern);
  std::smatch key_match;
  if (!std::regex_search(json_, key_match, key_re)) {
    return "";
  }

  size_t pos = key_match.position() + key_match.length();

  // 跳过空白字符
  while (pos < json_.size() && (json_[pos] == ' ' || json_[pos] == '\t' || json_[pos] == '\n' || json_[pos] == '\r')) {
    pos++;
  }

  // 必须从 [ 开始
  if (pos >= json_.size() || json_[pos] != '[') {
    return "";
  }

  // 括号深度匹配，提取完整数组内容
  int    depth = 0;
  size_t start = pos + 1; // 跳过开头的 [
  size_t end   = start;
  for (size_t i = start; i < json_.size(); i++) {
    if (json_[i] == '[') {
      depth++;
    } else if (json_[i] == ']') {
      if (depth == 0) {
        end = i;
        break;
      }
      depth--;
    }
  }

  return json_.substr(start, end - start);
}

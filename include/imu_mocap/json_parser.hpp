#pragma once

#include <string>
#include <vector>

/**
 * @brief 简单的 JSON 解析器，支持从 JSON 字符串中提取字段
 *
 * 专为 imu20 原始数据格式设计，支持：
 * - 字符串、整数、浮点数、布尔值字段提取
 * - 一维字符串/布尔数组提取
 * - 二维 double/int 数组提取（支持嵌套括号匹配）
 */
class SimpleJsonParser {
public:
  explicit SimpleJsonParser(const std::string& json_str);

  /** 提取字符串字段 */
  std::string getString(const std::string& key, const std::string& default_val = "");

  /** 提取整数字段 */
  int getInt(const std::string& key, int default_val = 0);

  /** 提取双精度浮点数字段（支持科学计数法） */
  double getDouble(const std::string& key, double default_val = 0.0);

  /** 提取布尔值字段 */
  bool getBool(const std::string& key, bool default_val = false);

  /** 提取字符串数组 */
  std::vector<std::string> getStringArray(const std::string& key);

  /** 提取布尔数组 */
  std::vector<bool> getBoolArray(const std::string& key);

  /** 提取二维 double 数组（修复了嵌套括号匹配问题） */
  std::vector<std::vector<double>> getDouble2DArray(const std::string& key);

  /** 提取二维 int 数组（修复了嵌套括号匹配问题） */
  std::vector<std::vector<int>> getInt2DArray(const std::string& key);

private:
  std::string json_;

  /**
   * @brief 根据 key 提取数组的完整内容（含嵌套括号）
   * @return 最外层 [...] 内部的内容
   *         例如 "[a,[b,c],d]" 返回 "a,[b,c],d"
   */
  std::string extractArray(const std::string& key);
};

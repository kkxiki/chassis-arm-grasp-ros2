#ifndef HTTP_UTILS_H
#define HTTP_UTILS_H

#include <string>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * @brief HTTP通用工具类
 * 封装libcurl的GET/POST请求，统一处理响应解析、错误返回、超时控制
 * 程序启动时自动完成libcurl全局初始化，退出时自动释放资源
 */
class HttpUtils {
public:
    /**
     * @brief 全局初始化libcurl（已自动执行，无需手动调用）
     */
    static void init();

    /**
     * @brief 全局释放libcurl资源（已自动执行，无需手动调用）
     */
    static void cleanup();

    /**
     * @brief 发送HTTP GET请求
     * @param url 完整请求URL
     * @param connect_timeout 连接超时时间，单位：秒
     * @param total_timeout 请求总超时时间，单位：秒
     * @return JSON格式响应，失败时code=-1，msg携带错误信息
     */
    static json httpGet(const std::string& url, int connect_timeout = 5, int total_timeout = 30);

    /**
     * @brief 发送HTTP POST请求（JSON格式请求体）
     * @param url 完整请求URL
     * @param body JSON请求体
     * @param connect_timeout 连接超时时间，单位：秒
     * @param total_timeout 请求总超时时间，单位：秒
     * @return JSON格式响应，失败时code=-1，msg携带错误信息
     */
    static json httpPost(const std::string& url, const json& body, int connect_timeout = 5, int total_timeout = 30);

private:
    /**
     * @brief libcurl响应写入回调函数
     */
    static size_t writeCallback(void* contents, size_t size, size_t nmemb, std::string* s);
};

#endif // HTTP_UTILS_H
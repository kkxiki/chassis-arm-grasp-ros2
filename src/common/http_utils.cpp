#include "common/http_utils.h"
#include <curl/curl.h>
#include <string>


// 静态全局对象：程序启动时自动初始化，退出时自动清理
struct CurlGlobalHolder {
    CurlGlobalHolder() { curl_global_init(CURL_GLOBAL_ALL); }
    ~CurlGlobalHolder() { curl_global_cleanup(); }
};
static CurlGlobalHolder g_curl_holder;

void HttpUtils::init() {
    // 空实现，依赖上面的静态对象自动完成初始化
}

void HttpUtils::cleanup() {
    // 空实现，依赖静态对象析构自动清理
}

// ... 下面原有的httpGet/httpPost/writeCallback代码全部保留不动

// ============================================================
// HTTP GET请求核心实现
// ============================================================
json HttpUtils::httpGet(const std::string& url, int connect_timeout, int total_timeout) {
    // 初始化默认返回结果，默认标记为失败
    json result;
    result["code"] = -1;
    result["msg"] = "unknown error";

    // 创建curl请求句柄
    CURL* curl = curl_easy_init();
    if (!curl) {
        result["msg"] = "curl init failed";
        return result;
    }

    std::string resp_str; // 存储响应原始字符串

    // 配置curl核心参数
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());          // 设置目标URL
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback); // 绑定响应写入回调
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp_str);      // 指定响应数据存储位置
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout); // 设置连接超时
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, total_timeout);    // 设置请求总超时

    // 执行HTTP请求
    CURLcode res = curl_easy_perform(curl);
    if (res == CURLE_OK) {
        // 请求成功，尝试解析JSON响应
        try {
            result = json::parse(resp_str);
        } catch (const std::exception& e) {
            result["msg"] = "json parse error: " + std::string(e.what());
        }
    } else {
        // 请求失败，记录底层错误信息
        result["msg"] = "http error: " + std::string(curl_easy_strerror(res));
    }

    // 释放curl句柄资源
    curl_easy_cleanup(curl);
    return result;
}

// ============================================================
// HTTP POST请求核心实现
// ============================================================
json HttpUtils::httpPost(const std::string& url, const json& body, int connect_timeout, int total_timeout) {
    // 初始化默认返回结果
    json result;
    result["code"] = -1;
    result["msg"] = "unknown error";

    // 创建curl请求句柄
    CURL* curl = curl_easy_init();
    if (!curl) {
        result["msg"] = "curl init failed";
        return result;
    }

    std::string body_str = body.dump(); // 将JSON对象序列化为字符串
    std::string resp_str;               // 存储响应原始字符串

    // 配置POST请求核心参数
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);                    // 启用POST请求模式
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body_str.c_str()); // 设置请求体数据指针
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body_str.size()); // 设置请求体长度
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp_str);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, connect_timeout);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, total_timeout);

    // 设置请求头：声明请求体为JSON格式
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json;charset=UTF-8");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    // 执行HTTP请求
    CURLcode res = curl_easy_perform(curl);
    if (res == CURLE_OK) {
        // 请求成功，解析JSON响应
        try {
            result = json::parse(resp_str);
        } catch (const std::exception& e) {
            result["msg"] = "json parse error: " + std::string(e.what());
        }
    } else {
        // 请求失败，记录错误信息
        result["msg"] = "http error: " + std::string(curl_easy_strerror(res));
    }

    // 释放请求头与curl句柄
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return result;
}

// ============================================================
// libcurl响应写入回调函数
// 自动将接收到的数据追加到传入的string容器中
// ============================================================
size_t HttpUtils::writeCallback(void* contents, size_t size, size_t nmemb, std::string* s) {
    size_t new_len = size * nmemb; // 计算本次接收到的数据总长度
    try {
        s->append(static_cast<char*>(contents), new_len); // 追加数据到字符串
    } catch (...) {
        // 内存分配失败时返回0，终止当前请求
        return 0;
    }
    return new_len; // 返回成功写入的数据长度
}
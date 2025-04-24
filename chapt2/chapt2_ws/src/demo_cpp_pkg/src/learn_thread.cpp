#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <cpp-httplib/httplib.h>

class Download
{
public:
    //说明一下const std::function<void(const std::string &,const std::string &)>&callback)这部分
    /*
    1、它是函数 download 的一个参数，名字叫做 callback。
    2、类型是 const std::function<...>&
        2.1、const & 表示这个参数是以常量引用方式传入，避免拷贝，提高效率，也不允许在函数内部修改它。
        2.2、std::function<void(const std::string &,const std::string &)> 是一个 可调用对象的封装器，可以存储 lambda 表达式、函数指针、成员函数等。
            2.2.1、std::function是通用函数封装器，用统一的方式存储、传递和调用各种可调用对象（函数、lambda、函数对象、成员函数等）
            2.2.2、<void(const std::string &,const std::string &)>：模板参数，它指定了 std::function 这个模板类要封装的“函数签名”。
            2.2.3、所谓函数签名，就是函数的参数类型和返回值类型。int add(int a, int b);这个函数的签名就是：int(int, int)

    */
    void download(const std::string &host, const std::string &path, const std::function<void(const std::string &, const std::string &)> &callback)
    {
        std::cout << "线程ID: " << std::this_thread::get_id() << std::endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200)
        {
            callback(path, response->body);
        }
    }

    void start_download(const std::string &host, const std::string &path, const std::function<void(const std::string &, const std::string &)> &callback)
    {
        auto download_fun = std::bind(&Download::download, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        std::thread download_thread(download_fun, host, path, callback);
        download_thread.detach();
    }
};

int main()
{
    Download download;
    auto download_finish_callback = [](const std::string &path, const std::string &result) -> void
    {
        std::cout << "下载完成：" << path << " 共：" << result.length() << "字，内容为：" << result.substr(0, 16) << std::endl;
    };

    download.start_download("http://localhost:8000", "/novel1.txt", download_finish_callback);
    download.start_download("http://localhost:8000", "/novel2.txt", download_finish_callback);
    download.start_download("http://localhost:8000", "/novel3.txt", download_finish_callback);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 10));
    return 0;
}

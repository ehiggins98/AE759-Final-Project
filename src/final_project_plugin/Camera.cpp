#include <zbar.h>
#include <opencv2/opencv.hpp>
#include <string>

namespace gazebo
{
    class Camera
    {
    public:
        Camera(unsigned int width, unsigned int height)
        {
            this->width = width;
            this->height = height;
        }
        void ProcessImage(const char *data)
        {
            cv::Mat img(this->height, this->width, CV_8UC3, (void *)data);
            cv::imwrite("orig" + std::to_string(i) + ".png", img);
            cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
            cv::adaptiveThreshold(img, img, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 23, 25);
            cv::imwrite("test" + std::to_string(i++) + ".png", img);
        }

    private:
        void decodeQr()
        {
            zbar::ImageScanner scanner;
            scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        }

        unsigned int height;
        unsigned int width;
        unsigned int i = 0;
    };
}
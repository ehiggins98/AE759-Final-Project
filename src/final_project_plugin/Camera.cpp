#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

        std::tuple<double, double, double> *ProcessImage(const char *data)
        {
            cv::Mat img, gray;
            cv::Mat orig(this->height, this->width, CV_8UC3, (void *)data);

            cv::cvtColor(orig, gray, cv::COLOR_RGB2GRAY);
            cv::adaptiveThreshold(gray, img, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 23, 25);

            std::vector<std::vector<cv::Point>>
                contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            for (std::vector<std::vector<cv::Point>>::const_iterator itr = contours.cbegin(), end = contours.cend(); itr != end; itr++)
            {
                if (cv::contourArea(*itr) < 30)
                {
                    cv::fillPoly(img, *itr, 0);
                }
            }

            cv::morphologyEx(img, img, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));

            contours.clear();
            hierarchy.clear();
            cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            int padding = 10;
            std::vector<std::vector<cv::Point>> potentialCodes;
            for (std::vector<std::vector<cv::Point>>::const_iterator itr = contours.cbegin(), end = contours.cend(); itr != end; itr++)
            {
                cv::RotatedRect rect = cv::minAreaRect(*itr);
                double aspectRatio = std::min(rect.size.width, rect.size.height) == 0 ? 0 : std::max(rect.size.width, rect.size.height) / std::min(rect.size.width, rect.size.height);
                int size = rect.size.width * rect.size.height;
                if (size >= 800 && size <= 9000 && aspectRatio >= 0.5 && aspectRatio <= 1.5)
                {
                    rect.size.width += 20;
                    rect.size.height += 20;

                    std::vector<cv::Point2f> boxPoints(4);
                    rect.points(boxPoints.data());

                    std::vector<cv::Point> points;
                    for (std::vector<cv::Point2f>::const_iterator itr = boxPoints.cbegin(), end = boxPoints.cend(); itr != end; itr++)
                    {
                        points.push_back(cv::Point(itr->x, itr->y));
                    }

                    potentialCodes.push_back(points);
                }
            }

            for (std::vector<std::vector<cv::Point>>::const_iterator itr = potentialCodes.cbegin(), end = potentialCodes.cend(); itr != end; itr++)
            {
                std::vector<cv::Point> p = orderPoints(*itr);
                std::vector<double> point;

                std::pair<cv::Mat, cv::Mat> transformed = transform(gray, p);
                cv::Mat qr = transformed.first;
                cv::Mat M = transformed.second;

                cv::Mat blurred;
                cv::GaussianBlur(qr, blurred, cv::Size(3, 3), 3);
                cv::addWeighted(qr, 1.25, blurred, -0.75, 0.0, qr);

                std::vector<cv::Point> qrPoints;
                std::string value = detector.detectAndDecode(qr, qrPoints);
                if (value.length() > 0 && qrPoints.size() == 4)
                {
                    std::vector<cv::Point> qrPointsTransformed;
                    for (std::vector<cv::Point>::const_iterator itr = qrPoints.cbegin(), end = qrPoints.cend(); itr != end; itr++)
                    {
                        cv::Mat point(3, 1, cv::DataType<double>::type);
                        point.at<double>(0) = itr->x;
                        point.at<double>(1) = itr->y;
                        point.at<double>(2) = 1;

                        M.convertTo(M, cv::DataType<double>::type);
                        cv::Mat transformed = M.inv() * point;
                        qrPointsTransformed.push_back(cv::Point(transformed.at<double>(0) / transformed.at<double>(2), transformed.at<double>(1) / transformed.at<double>(2)));
                    }

                    point = parseQrValue(value);

                    std::tuple<double, double> pos = computePosEstimate(qr, qrPointsTransformed, point);

                    return new std::tuple<double, double, double>(std::get<0>(pos), std::get<1>(pos), i++);
                }
            }

            return nullptr;
        }

    private:
        std::vector<double> parseQrValue(std::string value)
        {
            std::vector<double> point;
            std::string buf;
            for (int i = 0; i < value.length(); i++)
            {
                if (value.at(i) == ',')
                {
                    point.push_back(std::stod(buf));
                    buf = "";
                }
                else
                {
                    buf += value.at(i);
                }
            }

            point.push_back(std::stod(buf));
            return point;
        }

        std::vector<cv::Point> orderPoints(std::vector<cv::Point> points)
        {
            std::vector<cv::Point> ordered;
            std::vector<double> norms = L1Norms(points);

            std::unordered_set<int> used;

            int argmax = -1;
            int argmin = -1;
            for (int i = 0; i < norms.size(); i++)
            {
                if (argmax < 0 || norms.at(i) > norms.at(argmax))
                {
                    argmax = i;
                }
                if (argmin < 0 || norms.at(i) < norms.at(argmin))
                {
                    argmin = i;
                }
            }

            used.insert(argmax);
            used.insert(argmin);

            std::vector<double> diffs = Diffs(points);
            int diffArgmax = -1;
            int diffArgmin = -1;
            for (int i = 0; i < norms.size(); i++)
            {
                if (used.find(i) == used.end() && (diffArgmax < 0 || diffs.at(i) > diffs.at(diffArgmax)))
                {
                    diffArgmax = i;
                }
                if (used.find(i) == used.end() && (diffArgmin < 0 || diffs.at(i) < diffs.at(diffArgmin)))
                {
                    diffArgmin = i;
                }
            }

            ordered.push_back(points.at(argmin));
            ordered.push_back(points.at(diffArgmin));
            ordered.push_back(points.at(argmax));
            ordered.push_back(points.at(diffArgmax));

            return ordered;
        }

        std::vector<double> L1Norms(std::vector<cv::Point> points)
        {
            std::vector<double> norms;

            for (std::vector<cv::Point>::const_iterator itr = points.cbegin(), end = points.cend(); itr != end; itr++)
            {
                norms.push_back(itr->x + itr->y);
            }

            return norms;
        }

        std::vector<double> Diffs(std::vector<cv::Point> points)
        {
            std::vector<double> norms;

            for (std::vector<cv::Point>::const_iterator itr = points.cbegin(), end = points.cend(); itr != end; itr++)
            {
                norms.push_back(itr->y - itr->x);
            }

            return norms;
        }

        std::pair<cv::Mat, cv::Mat> transform(cv::Mat img, std::vector<cv::Point> pts)
        {
            cv::Point tl = pts.at(0);
            cv::Point tr = pts.at(1);
            cv::Point br = pts.at(2);
            cv::Point bl = pts.at(3);

            double widthA = std::sqrt(std::pow(br.x - bl.x, 2) + std::pow(br.y - bl.y, 2));
            double widthB = std::sqrt(std::pow(tr.x - tl.x, 2) + std::pow(tr.y - tl.y, 2));
            double maxWidth = std::max((int)std::floor(widthA), (int)std::floor(widthB));

            double heightA = std::sqrt(std::pow(tr.x - br.x, 2) + std::pow(tr.y - br.y, 2));
            double heightB = std::sqrt(std::pow(tl.x - bl.x, 2) + std::pow(tl.y - bl.y, 2));
            double maxHeight = std::max((int)std::floor(heightA), (int)std::floor(heightB));

            std::vector<cv::Point2f> src;
            for (std::vector<cv::Point>::const_iterator itr = pts.cbegin(), end = pts.cend(); itr != end; itr++)
            {
                src.push_back(cv::Point2f(itr->x, itr->y));
            }

            std::vector<cv::Point2f> dst;
            dst.push_back(cv::Point2f(0, 0));
            dst.push_back(cv::Point2f(maxWidth - 1, 0));
            dst.push_back(cv::Point2f(maxWidth - 1, maxHeight - 1));
            dst.push_back(cv::Point2f(0, maxHeight - 1));

            cv::Mat M = cv::getPerspectiveTransform(src, dst);
            cv::Mat warped;
            cv::warpPerspective(img, warped, M, cv::Size(maxWidth, maxHeight));

            return std::pair<cv::Mat, cv::Mat>(warped, M);
        }

        std::tuple<double, double> computePosEstimate(cv::Mat qr, std::vector<cv::Point> corners, std::vector<double> center)
        {
            // qr images are 1m x 1m, and the QR code itself is 0.84m x 0.84m
            double qrDim = 0.84;
            std::vector<cv::Point3d> orderedWorldPoints;
            orderedWorldPoints.push_back(cv::Point3d(center.at(0) - qrDim / 2, center.at(1) + qrDim / 2, 0));
            orderedWorldPoints.push_back(cv::Point3d(center.at(0) + qrDim / 2, center.at(1) + qrDim / 2, 0));
            orderedWorldPoints.push_back(cv::Point3d(center.at(0) + qrDim / 2, center.at(1) - qrDim / 2, 0));
            orderedWorldPoints.push_back(cv::Point3d(center.at(0) - qrDim / 2, center.at(1) - qrDim / 2, 0));

            int data[9] = {277, 0, 960, 0, 277, 540, 0, 0, 1};
            cv::Mat intrinsicMatrix(3, 3, cv::DataType<int>::type, &data);

            std::vector<cv::Point2d> orderedImagePoints;
            for (std::vector<cv::Point>::const_iterator itr = corners.cbegin(), end = corners.cend(); itr != end; itr++)
            {
                orderedImagePoints.push_back(cv::Point2d(itr->x, itr->y));
            }

            cv::Mat rotation;
            cv::Mat position;
            cv::solvePnP(orderedWorldPoints, orderedImagePoints, intrinsicMatrix, std::vector<double>(), rotation, position, false, cv::SOLVEPNP_IPPE);

            position = transformPosition(rotation, position);

            return std::tuple<double, double>(position.at<double>(0), position.at<double>(1));
        }

        static bool comparator(std::pair<std::vector<cv::Point>, double> a, std::pair<std::vector<cv::Point>, double> b)
        {
            return a.second < b.second;
        }

        double dist(cv::Point a, cv::Point b)
        {
            return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
        }

        cv::Mat transformPosition(cv::Mat rotation, cv::Mat position)
        {
            cv::Mat R;
            cv::Rodrigues(rotation, R);

            R = R.t();
            position = -1 * R * position;
            return position;
        }

        cv::QRCodeDetector detector;
        unsigned int height;
        unsigned int width;
        unsigned int i = 0;
    };
}
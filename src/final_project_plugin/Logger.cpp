#include <iostream>
#include <fstream>
#include <ctime>

namespace gazebo
{
    struct LogData
    {
        double x;
        double y;
        double vx;
        double vy;
    };

    struct LogEntry
    {
        LogData estimated;
        LogData truth;
    };

    class Logger
    {
    public:
        Logger()
        {
            time_t now;
            time(&now);
            char buf[sizeof("2021-05-06T03:00:00Z")];
            strftime(buf, sizeof(buf), "%FT%TZ", gmtime(&now));
            file.open(std::string(buf) + ".csv");
            file << "Estimated x,Estimated y,Estimated vx,Estimated vy,True x,True y,True vx,True vy\n";
        }

        void WriteRecord(const LogEntry &entry)
        {
            file << entry.estimated.x << "," << entry.estimated.y << "," << entry.estimated.vx << "," << entry.estimated.vy << "," << entry.truth.x << "," << entry.truth.y << "," << entry.truth.vx << "," << entry.truth.vy << "\n";
        }

        ~Logger()
        {
            file.close();
        }

    private:
        std::ofstream file;
    };
}
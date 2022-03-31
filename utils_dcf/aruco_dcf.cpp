#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aruco.h"
#include "aruco_cvversioning.h"
#include "dcf/dcfmarkertracker.h"

#ifdef USING_ROS
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#endif

using namespace aruco;
using namespace std;

class CmdLineParser {
    int argc;
    char **argv;

   public:
    CmdLineParser(int _argc, char **_argv) : argc(_argc), argv(_argv) {}
    bool operator[](string param) {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param) idx = i;
        return (idx != -1);
    }
    string operator()(string param, string defvalue = "-1") {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param) idx = i;
        if (idx == -1)
            return defvalue;
        else
            return (argv[idx + 1]);
    }
};
struct TimerAvrg {
    std::vector<double> times;
    size_t curr = 0, n;
    std::chrono::high_resolution_clock::time_point begin, end;
    TimerAvrg(int _n = 30) {
        n = _n;
        times.reserve(n);
    }
    inline void start() { begin = std::chrono::high_resolution_clock::now(); }
    inline void stop() {
        end = std::chrono::high_resolution_clock::now();
        double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
        if (times.size() < n)
            times.push_back(duration);
        else {
            times[curr] = duration;
            curr++;
            if (curr >= times.size()) curr = 0;
        }
    }
    double getAvrg() {
        double sum = 0;
        for (auto t : times) sum += t;
        return sum / double(times.size());
    }
};

TimerAvrg timer;
DFCMarkerTracker TheTracker;

int main(int argc, char **argv) {
    try {
        CmdLineParser cml(argc, argv);
        if (argc < 2 || cml["-h"]) {
            cerr << "Usage: (invideo|cameraindex) [-c cameraParams.yml] [-s markerSize] [-start frame] [-d <dicionary>:ALL_DICTS default] [-f arucoConfig.yml]" << std::endl;
            cerr << "\tDictionaries: ";
            for (auto dict : aruco::Dictionary::getDicTypes())
                cerr << dict << " ";
            cerr << endl;
            cerr << "\t Instead of these, you can directly indicate the path to a file with your own generated "
                    "dictionary"
                 << endl;
            cout << "Example to work with apriltags dictionary : video.avi -d TAG36h11" << endl
                 << endl;
            return 0;
        }

        aruco::CameraParameters CamParam;
        // read camera parameters if passed
        if (cml["-c"])
            CamParam.readFromXMLFile(cml("-c"));

        float MarkerSize = std::stof(cml("-s", "-1"));

        cv::VideoCapture video;
        
        // open video/camera
        try {
            int camIndex = std::stoi(argv[1]);
            video.open(camIndex);
        } catch (const std::exception &e) {
            video.open(argv[1]);
        }
        if (!video.isOpened()) throw std::runtime_error("Could not open video");

        cv::Mat image;

        int frameid = std::stoi(cml("-start", "0"));

        video.set(cv::CAP_PROP_POS_FRAMES, frameid);
        while (image.empty()) video >> image;

        if (cml["-f"])  // uses a configuration file. YOu can create it from aruco_test application
            TheTracker.loadParamsFromFile(cml("-f"));
        else
            TheTracker.setDictionary(cml("-d", "ALL_DICTS"), 0.f);

        if (CamParam.isValid() && MarkerSize != -1) {
            CamParam.resize(image.size());
            TheTracker.setParams(CamParam, MarkerSize);
        }

        int maxNFrames = video.get(cv::CAP_PROP_FRAME_COUNT) - 1;


        #ifdef USING_ROS
        // configure ROS
        ros::init(argc, argv, "aruco_dcf", ros::init_options::NoSigintHandler);
        ros::NodeHandle nh;
        ros::start();

        image_transport::ImageTransport it(nh);
        image_transport::Publisher rgbPub = it.advertise("aruco/detection", 1);
        ros::Publisher targetPub = nh.advertise<geometry_msgs::PoseStamped>("aruco/target", 1000);
        #endif

        geometry_msgs::PoseStamped target;
        char key = 0;
        int waitTime = 10;
        do {
            video.retrieve(image);

            timer.start();
            std::map<int, cv::Ptr<TrackerImpl>> setTrackers = TheTracker.track(image);
            TheTracker.estimatePose();
            timer.stop();
            std::vector<float> position = {0.0f, 0.0f, 0.0f};
            for (auto m : setTrackers){
                std::cout << m.second->getMarker() << " W=" << m.second->getTrustVal() << std::endl;
                // position = 
            }
            std::cout << "|@ Frame:" << frameid++ << "/" + to_string(maxNFrames) << ", fps:" << 1. / timer.getAvrg() << std::endl;
            TheTracker.draw(image);
            
            if(!setTrackers.empty()){
                cv::Ptr<TrackerImpl> trackedTag = setTrackers.begin()->second; 
                cv::Rect2d bbox = trackedTag->getBoundingBox();
                
                #ifdef USING_ROS
                std_msgs::Header header;
                header.stamp = ros::Time::now();
                header.frame_id = "camera";

                target.header = header;
                target.pose.position.x = bbox.x + bbox.width/2;
                target.pose.position.y = bbox.y + bbox.height/2;
                target.pose.position.z = 0;
                targetPub.publish(target);

                const sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
                rgbPub.publish(rgb_msg);
                ros::spinOnce();
                #endif
            }

            cv::resize(image, image, cv::Size(1200, 600));
            cv::imshow("image", image);
            key = cv::waitKey(waitTime);  // wait for key to be pressed
            if (key == 's')
                waitTime = waitTime == 0 ? 10 : 0;

        } while (video.grab() && key != 27);

    } catch (const std::exception &ex) {
        cerr << ex.what() << endl;
    }
}

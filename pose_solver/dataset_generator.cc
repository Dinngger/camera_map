#include "LightMatch.hpp"
#include <string>
#include <fstream>
#include <sstream>
#define path0a "/home/dinger/mine/Dataset/videos/disp_low2.avi"
#define path0b "/home/dinger/mine/Dataset/videos/output_high.avi"
#define path1a "/media/sentinel/ENIGMATICS/cv_output.avi"
#define path2a "/home/sentinel/videos/disp_low2.avi"
#define path2b "/home/sentinel/videos/output_high.avi"
#define path3a "/home/zhao/output_low.avi"
#define path4 "/home/xjturm/rm2020/videos/disp_low2.avi"

int mouse_pos[4];
bool mouse_on = false;
bool new_data = false;

void mouseHandle(int event, int x, int y, int flag, void* param)
{
    switch (event) {
	case cv::EVENT_MOUSEMOVE: //移动
		mouse_pos[2] = x;
        mouse_pos[3] = y;
		break;
	case cv::EVENT_LBUTTONDOWN://左键按下
		mouse_pos[0] = x;
        mouse_pos[1] = y;
        mouse_on = true;
		break;
	case cv::EVENT_LBUTTONUP://左键抬起
		mouse_pos[2] = x;
        mouse_pos[3] = y;
        new_data = true;
        mouse_on = false;
		break;
	}
}

int main(int argc, char* argv[]) {
    std::ofstream outFile;
    if (argc < 2)
        printf("usage: dataset_generator out_file_name\n");
    outFile.open(argv[1], std::ios::out);
    cv::VideoCapture cap(path3a);
    if (!cap.isOpened()) {
        printf("Unable to open video.\n");
        return 0;
    }
    double totalFrameNumber = cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout<<"frame: ";
    std::cout<<totalFrameNumber<<std::endl;

    cv::Mat frame;
    LightMatch match;
    cv::namedWindow("disp");
	cv::setMouseCallback("disp", mouseHandle, nullptr);
    for (int w = 0; w < totalFrameNumber; w++) {
        std::cout << "\033[32m" << "frame: " << w << "\033[0m\n";
        cap.read(frame);
		if(frame.empty())
            break;
        match.saveImg(frame);
        match.findPossible();
        assert(match.possibles.size() <= 13);
        char car_id[13] = {0,};
        cv::Mat casturing(frame.size(), CV_8U, cv::Scalar(0));
        #define LINE_THICKNESS 6
        for (size_t i = 0; i < match.possibles.size(); ++i) {
            cv::line(casturing, match.possibles[i].box.vex[0], match.possibles[i].box.vex[1], cv::Scalar(i + 1), LINE_THICKNESS);
        }
        std::set<int> selected;
        while (1) {
            cv::Mat show_frame = frame.clone();
            char str_lb[5];
            char str_car[2] = {'0',};
            for (size_t i = 0; i < match.possibles.size(); ++i) { 
                cv::Scalar color = selected.count(i) ? aim_deps::RED : aim_deps::CYAN;
                cv::line(show_frame, match.possibles[i].box.vex[0], match.possibles[i].box.vex[1], color, LINE_THICKNESS);
                snprintf(str_lb, 4, "%lu", i);
                str_car[0] = '0' + car_id[i];
                cv::putText(show_frame, str_car, match.possibles[i].box.vex[0] + cv::Point2f(1, 1),
                    cv::FONT_HERSHEY_PLAIN, 1.5, aim_deps::ORANGE);
                cv::putText(show_frame, str_lb, match.possibles[i].box.vex[1] + cv::Point2f(-4, 4),
                    cv::FONT_HERSHEY_PLAIN, 1.5, aim_deps::YELLOW);
                if (mouse_on) {
                    cv::Point2i p1(mouse_pos[0], mouse_pos[1]);
                    cv::Point2i p2(mouse_pos[2], mouse_pos[3]);
                    cv::rectangle(show_frame, cv::Rect2i(p1, p2), aim_deps::GREEN);
                }
            }
            cv::imshow("disp", show_frame);
            char key = cv::waitKey(20);
            if (key == 27) {
                goto END;
            } else if (key >= '0' && key <='4') {
                int _car = int(key - '0');
                for (int select : selected) {
                    car_id[select] = _car;
                }
            } else if (key == 'd') {
                for (size_t i=0; i<match.possibles.size(); i++) {
                    cv::Point2f center;
                    cv::Point2f _p;
                    center = (match.possibles[i].box.vex[0]+match.possibles[i].box.vex[1])/2;
                    if(match.possibles[i].box.vex[0].y>match.possibles[i].box.vex[1].y)
                        _p = match.possibles[i].box.vex[0]-center;
                    else
                        _p = match.possibles[i].box.vex[1]-center;
                    outFile << center.x<<','<<center.y<<','<<_p.x<<','<<_p.y<<',';
                }
                for (int i=match.possibles.size(); i<13; i++) {
                    outFile << "0,0,0,0,";
                }
                char str[2] = {0,};
                for (int i=0; i<13; i++) {
                    str[0] = '0' + car_id[i];
                    outFile << str << ",";
                }
                for (size_t i=0; i<match.possibles.size(); i++)
                    if (i == 12)
                        outFile << "1";
                    else
                        outFile << "1,";
                for (size_t i=match.possibles.size(); i<13; i++) {
                    if (i == 12)
                        outFile << "0";
                    else
                        outFile << "0,";
                }
                outFile << "\n";
                break;
            } else if (new_data) {
                new_data = false;
                selected.clear();
                int x_min = std::min(mouse_pos[1], mouse_pos[3]);
                int x_max = std::max(mouse_pos[1], mouse_pos[3]);
                int y_min = std::min(mouse_pos[0], mouse_pos[2]);
                int y_max = std::max(mouse_pos[0], mouse_pos[2]);
                for (int i=x_min; i<=x_max; i++) {
                    for (int j=y_min; j<=y_max; j++) {
                        int lb_id = (int)casturing.at<char>(i, j);
                        if (lb_id > 0) {
                            selected.insert(lb_id - 1);
                        }
                    }
                }
            }
        }
	}
END:
    outFile.close();
	cv::destroyAllWindows();
    return 0;
}

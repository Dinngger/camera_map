#include "../include/CarMatch.hpp"

// const std::string head_path = "/home/sentinel/camera_map/fifos/";
const std::string head_path = "/home/zhao/camera_map/fifos/";
// const std::string head_path = "/home/dinger/mine/RoboMaster/camera_map/fifos/";

CarMatch::~CarMatch() {;}

CarMatch::CarMatch() {
    const std::string input = head_path + "input.pipe";
    const std::string output = head_path + "output.pipe";
    if (access(input.c_str(), F_OK) != 0 || access(output.c_str(), F_OK) != 0) {
        printf("Error: broken pipe.\n");
        exit(-1);
    }
    mkfifo(input.c_str(), 0777);
    mkfifo(output.c_str(), 0777);
    in_fd = open(input.c_str(), O_SYNC | O_RDWR);
    out_fd = open(output.c_str(), O_RDONLY);
}

void CarMatch::transformerMatch(std::vector<aim_deps::Light> &lights) {
    carsPossible.clear();
    std::sort(lights.begin(), lights.end(), 
        [&](const aim_deps::Light& l1, const aim_deps::Light& l2) {
            return l1.box.center.x < l2.box.center.x;
        }
    );
    memset(w_trans.data, 0, BUFFER_SIZE * sizeof(float));
    for (size_t i = 0; i < lights.size(); i++) {
        const cv::Point2f& center = lights[i].box.center;
        cv::Point2f vec = lights[i].box.vex[0] - center;
        w_trans.data[4 * i] = center.x;
        w_trans.data[4 * i + 1] = center.y; 
        w_trans.data[4 * i + 2] = vec.x;
        w_trans.data[4 * i + 3] = vec.y;
        w_trans.data[EXIST_INDEX + i] = 1;
    }
    if (write(in_fd, (void *)w_trans.buf, 4 * BUFFER_SIZE) == -1) {
        std::cerr << "No data sent.\n";
    }

    /// TODO: 超市检测
    size_t total_num = 0;
    std::map<int, int> car_map;
    while (true) {
        total_num += read(out_fd, (void * )(r_trans + total_num), 13 - total_num);
        if (total_num < 13) {
            usleep(10);
            continue;
        }
        for (size_t i = 0; i < lights.size(); i++) {
            if (r_trans[i] > 0) {
                if (car_map.count(r_trans[i]) == 0) {
                    CarPossible cp;
                    if (lights[i].isLeft != -1) {
                        cp.first = lights[i].isLeft ? 0 : 1;
                        cp.first_index = 0;         // 第一个的灯条对应index 0
                    }
                    cp.lightPossibles.emplace_back(lights[i]);
                    carsPossible.emplace_back(cp);
                    car_map[r_trans[i]] = carsPossible.size() - 1;
                } else {
                    int index = car_map[r_trans[i]];
                    if (lights[i].isLeft != -1 && carsPossible[index].first == -1) {
                        carsPossible[index].first = lights[i].isLeft ? 0 : 1;
                        carsPossible[index].first_index = carsPossible[index].lightPossibles.size();
                    }
                    carsPossible[index].lightPossibles.emplace_back(lights[i]);
                }
            }
        }
        printf("\033[34mResult received, light %lu: [", lights.size());
        for (int i = 0; i < 13; i++) {
            printf("%d, ", r_trans[i]);
        }
        printf("]\n\033[0m");
        break;
    }
}
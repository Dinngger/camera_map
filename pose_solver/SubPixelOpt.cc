#include <iostream>
#include <sstream>
#include <vector>
#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace SubPixelOpt {

#define print_var(var) (std::cout << #var << ": " << var << std::endl)

template <typename T>
void fill(T* p, const T& a, const T& b) {
    p[0] = a;
    p[1] = b;
}

template <typename T>
T tdot(const T* const p1, const T* const p2) {
    return p1[0] * p2[0] + p1[1] * p2[1];
}

template <typename T>
T pow2(const T& x) {
    return x * x;
}

template <typename T>
T getPointDist(const T* const p1, const T* const p2){
    return ceres::sqrt(pow2(p1[0]-p2[0]) + pow2(p1[1]-p2[1]));
}

template <typename T>
T getRadiusLength(const T* const p1, const T* const p2) {
    T d = ceres::abs(p1[0]*p2[1] - p2[0]*p1[1]) / getPointDist(p1, p2) + T(2.0);
    // print_var(d);
    T v1[2] = {p2[0] - p1[0], p2[1] - p1[1]};
    T v1norm = ceres::sqrt(tdot(v1, v1));
    v1[0] /= v1norm;
    v1[1] /= v1norm;
    T p1v1 = tdot(p1, v1);
    T l2 = v1norm + p1v1;
    T l1 = v1norm - l2;
    return (ceres::atan2(l1, d) + ceres::atan2(l2, d)) / d;
}

int radiusLengthTest(double* point) {
    printf("res: %f\n", getRadiusLength(point, point+2));
    return 0;
}

template <typename T>
T getCrossoverX(T y, const T* const p1, const T* const p2){
    if ((y <= p1[1] && y >= p2[1]) || (y >= p1[1] && y <= p2[1]))
        return ((y - p2[1]) * p1[0] + (p1[1] - y) * p2[0]) / (p1[1] - p2[1]);
    else
        return T(-1);
}

template <typename T>
T getCrossoverY(T x, const T* const p1, const T* const p2){
    if ((x <= p1[0] && x >= p2[0]) || (x >= p1[0] && x <= p2[0]))
        return ((x - p2[0]) * p1[1] + (p1[0] - x) * p2[1]) / (p1[0] - p2[0]);
    else
        return T(-1);
}

template <typename T>
bool inRect(const T* const p, const T* const size) {
    return p[0] >= T(0) && p[0] < size[1] && p[1] >= T(0) && p[1] < size[0];
}

template <typename T>
T getCrossoverLength(const T* const p1, const T* const p2, const T* const size) {
    T cross_points[2][2];
    int valid_cnt = 0;
    T temp = getCrossoverX(T(0), p1, p2);
    if (temp >= T(0) && temp < size[1])
        fill(cross_points[valid_cnt++], temp, T(0));
    temp = getCrossoverX(size[0], p1, p2);
    if (temp > T(0) && temp <= size[1])
        fill(cross_points[valid_cnt++], temp, size[0]);
    temp = getCrossoverY(T(0), p1, p2);
    if (temp > T(0) && temp <= size[0])
        fill(cross_points[valid_cnt++], T(0), temp);
    temp = getCrossoverY(size[1], p1, p2);
    if (temp >= T(0) && temp < size[0])
        fill(cross_points[valid_cnt++], size[1], temp);
    if (valid_cnt == 0) {
        if (inRect(p1, size))
            return getPointDist(p1, p2);
        else
            return T(0);
    } else if (valid_cnt == 1) {
        if (inRect(p1, size))
            return getPointDist(p1, cross_points[0]);
        else
            return getPointDist(p2, cross_points[0]);
    } else {
        return getPointDist(cross_points[0], cross_points[1]);
    }
}

template <typename T>
T computeScore(const cv::Mat& img, const T* const p1, const T* const p2) {
    T score(0);
    for (int i=0; i<img.rows; i++) {
        for (int j=0; j<img.cols; j++) {
            T _p1[2] = {p1[0] - T(0.5 + j), p1[1] - T(0.5 + i)};
            T _p2[2] = {p2[0] - T(0.5 + j), p2[1] - T(0.5 + i)};
            score += getRadiusLength(_p1, _p2) * (1.0 - img.at<float>(i, j));
        }
    }
    T norm = getPointDist(p1, p2);
    T size[2] = {T(img.rows), T(img.cols)};
    T crossLength = getCrossoverLength(p1, p2, size);
    score /= crossLength + T(1e-5);
    // score += T(1) / (crossLength + T(1e-2));
    if (norm - crossLength > T(0))
        score += norm - crossLength;
    return score;
}

void threshold(const cv::Mat& src, cv::Mat& dst) {
    cv::Mat srcSplit[3];
    cv::split(src, srcSplit);
    cv::Mat blueMat = srcSplit[0] - srcSplit[2];
    cv::Mat filtered;
    cv::threshold(blueMat, filtered, 20.0 / 255, 1, cv::THRESH_BINARY_INV);
    dst = srcSplit[0] - filtered;
	cv::threshold(dst, dst, 92.0 / 255, 1, cv::THRESH_BINARY);
}

bool isGoodBoundingBox(const cv::Rect &rect){
	/// 根据rect.area 来判定合适的height - width 最小允许值
	if (rect.area() > 120){
		return (rect.height - rect.width >= 1);
	} else if (rect.area() > 10){
		/// 一个拟合函数
		float w = (float)rect.width, h = (float)rect.height, a = (float)rect.area();
		return h - w >= - 1.83525e-04 * a * a +  6.04567e-02 * a - 3.62164;
	} else{
		return (rect.height - rect.width >= - 3);
	}
}

bool extendRect(cv::Rect &rect, const cv::Size& sz, const cv::Size& mat_size) {
	bool within = true;
	if (rect.x - sz.width < 0) {
		rect.x = 0;
		within = false;
	} else
        rect.x -= sz.width;
	if (rect.y - sz.height < 0) {
		rect.y = 0;
		within = false;
	} else
        rect.y -= sz.height;
	if (rect.x + 2 * sz.width + rect.width >= mat_size.width) {
		rect.width = mat_size.width - 1 - rect.x;
		within = false;
	} else
        rect.width += 2 * sz.width;
	if (rect.y + 2 * sz.height + rect.height >= mat_size.height) {
		rect.height = mat_size.height - 1 - rect.y;
		within = false;
	} else
        rect.height += 2 * sz.height;
	return within;
}

float getPointDist(const cv::Point2f &p1, const cv::Point2f &p2){
    return sqrt(pow2(p1.x-p2.x) + pow2(p1.y-p2.y));
}

void getMidPoints(const cv::RotatedRect &rect, cv::Point2f &p1, cv::Point2f &p2){
    cv::Point2f tmp_p1, tmp_p2, corners[4];                                     //找出角点
    rect.points(corners);
    float d1 = getPointDist(corners[0], corners[1]);            //0/1点距离的平方
	float d2 = getPointDist(corners[1], corners[2]);            //1/2点距离的平方
	int i0 = d1 > d2? 1 : 0;								    //长所在边第一个顶点的位置
    tmp_p1 = (corners[i0] + corners[i0 + 1]) / 2;			    //获得旋转矩形两条短边上的中点
	tmp_p2 = (corners[i0 + 2] + corners[(i0 + 3) % 4]) / 2;
    if(tmp_p1.y > tmp_p2.y){                                    //保证输出点的顺序
        p2 = tmp_p1;
        p1 = tmp_p2;
    }
    else{                                                       //必须是p1是处于上方的点，p2处于下方（y轴更大）
        p1 = tmp_p1;
        p2 = tmp_p2;
    }
}

struct SubPixelError {
    const cv::Mat img;
    const double offsetX, offsetY;
    SubPixelError(
        const cv::Mat& img,
        const double& offsetX,
        const double& offsetY
    ) : img(img), offsetX(offsetX), offsetY(offsetY) {}
    template <typename T>
    bool operator()(const T* const pt1, const T* const pt2, T* residuals) const {
        residuals[0] = computeScore(img, pt1, pt2);
        return true;
    }
    static ceres::CostFunction *Create(
        const cv::Mat img,
        const double& offsetX,
        const double& offsetY
    ) {
        return new ceres::AutoDiffCostFunction<SubPixelError, 1, 2, 2>(
            new SubPixelError(img, offsetX, offsetY)
        );
    }
};

bool computeLine(const cv::Mat& img, cv::Point2f &p1, cv::Point2f &p2, int channel) {
    cv::Mat imgSplit[3];
    cv::split(img, imgSplit);
    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    double p1a[2] = {p1.x, p1.y}, p2a[2] = {p2.x, p2.y};
    for (int i=0; i<1; i++) {
        ceres::CostFunction* cost = SubPixelError::Create(imgSplit[channel], 0, 0);
        problem.AddResidualBlock(cost, loss_function, p1a, p2a);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-5;
    options.max_num_iterations = 100;
    options.num_threads = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    p1 = cv::Point2f(p1a[0], p1a[1]);
    p2 = cv::Point2f(p2a[0], p2a[1]);
    return true;
}

bool isLowExposure(cv::Mat &src) {
	cv::Scalar mean_values;
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    int row_sampling = 16;
    int col_sampling = 16;
    int sampling_sum = 0;
    for(int i=0;i<row_sampling;++i){
        for(int j=0;j<col_sampling;++j){
            int pix_value = gray.at<uchar>(gray.rows*0.1+i*gray.rows*0.8/row_sampling, gray.cols*0.1+j*gray.cols*0.8/col_sampling);
            if(pix_value>30) sampling_sum++;
            // cv::Point2f smp = cv::Point2f(gray.cols*0.1+i*gray.cols*0.8/col_sampling, gray.rows*0.1+j*gray.rows*0.8/row_sampling);
            // cv::circle(src, smp, 3, cv::Scalar(0, 0, 255), -1);
        }
    }
    if(sampling_sum < row_sampling*col_sampling*0.3)
	    return true;
    else
        return false;
}

int test() {
    cv::VideoCapture cap("/home/dinger/mine/Dataset/videos/cv_output5.avi");
    while (true) {
        cv::Mat frame, binary;
        cap.read(frame);
        if (frame.empty())
            return 0;
        if(!isLowExposure(frame))
            continue;
        frame.convertTo(frame, CV_32FC3, 1.0 / 255);
        threshold(frame, binary);
        binary.convertTo(binary, CV_8UC3, 255.0);
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        int light_bar_num = 0;
	    for (size_t contour_cnt = 0; contour_cnt < contours.size(); contour_cnt++) {
            float area = cv::contourArea(contours[contour_cnt]);
            cv::Rect bbox = cv::boundingRect(contours[contour_cnt]);
            if (bbox.area() <= 20 * area && area >= 4.0 && bbox.height >= 4 && isGoodBoundingBox(bbox)) {
                // cv::rectangle(frame, bbox, cv::Scalar(0, 0, 0.5), 0.5, cv::LINE_AA);
                extendRect(bbox, cv::Size(3, 3), frame.size());
                cv::Mat lbp_mat = frame(bbox);
                cv::RotatedRect rr = cv::minAreaRect(contours[contour_cnt]);
                cv::Point2f pt1, pt2;
                getMidPoints(rr, pt1, pt2);
                cv::Point2f bbox_ref(bbox.x, bbox.y);
                pt1 -= bbox_ref + cv::Point2f(0.5, 0);
                pt2 -= bbox_ref + cv::Point2f(0.5, 0);
                print_var(pt1);
                print_var(pt2);

                cv::Mat imgSplit[3];
                cv::split(lbp_mat, imgSplit);

                for (int channel=0; channel<3; channel++) {
                    // 可视化
                    cv::Mat show_mat(imgSplit[channel].size() * 50, imgSplit[channel].type());
                    for (int i=0; i<imgSplit[channel].rows; i++)
                        for (int j=0; j<imgSplit[channel].cols; j++)
                            for (int _i=0; _i<50; _i++)
                                for (int _j=0; _j<50; _j++)
                                    show_mat.at<float>(i*50+_i, j*50+_j) = imgSplit[channel].at<float>(i, j);
                    cv::line(show_mat, pt1 * 50, pt2 * 50, 0.5, 1, cv::LINE_AA);

                    cv::Point2f p1 = pt1, p2 = pt2;
                    double p1a[2] = {p1.x, p1.y}, p2a[2] = {p2.x, p2.y};
                    computeLine(lbp_mat, p1, p2, channel);
                    print_var(p1);
                    print_var(p2);
                    // cv::Scalar color(0, 0, 0);
                    // color(i) = 1;
                    cv::line(show_mat, p1 * 50, p2 * 50, 1, 1, cv::LINE_AA);
                    cv::Mat scoreMat(show_mat.size(), show_mat.type());
                    for (int i=0; i<scoreMat.rows; i++)
                        for (int j=0; j<scoreMat.cols; j++) {
                            p2a[0] = 1.0 / 50 * j;
                            p2a[1] = 1.0 / 50 * i;
                            scoreMat.at<float>(i, j) = computeScore(imgSplit[channel], p1a, p2a);
                        }
                    cv::normalize(scoreMat, scoreMat, 1, 0, CV_MINMAX);
                    std::stringstream ss;
                    ss << "lbp" << channel;
                    std::string mat_name;
                    ss >> mat_name;
                    cv::imshow(mat_name, show_mat);
                    cv::imshow(mat_name + "score", scoreMat);
                }
                if (cv::waitKey(0) == 'q')
                    return 0;
                light_bar_num++;
            }
        }
        print_var(light_bar_num);
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    double p[4];
    if (argc < 5)
        return SubPixelOpt::test();
    for (int i=0; i<4; i++)
        p[i] = atof(argv[i+1]);
    return SubPixelOpt::radiusLengthTest(p);
}
/*
灯条检测模块-找到图上所有合适的灯条,并且将其匹配为装甲板
实现者：雷达组-zlq
代码思路：雷达组-dxd
修改：雷达组-dxd
封装：hqy
last date of modification:2020.4.7
*/

#include "LightMatch.hpp"

LightMatch::LightMatch(){
	#ifdef LIGHT_CNT_TIME		
		time_sum 		= 0.0;
		_cnt 			= 0;
	#endif // LIGHT_CNT_TIME
	low_exposure 	= false;
	mean_val 		= aim_deps::LIGHT_mean;
	setEnemyColor(true);												//初始设置敌人颜色
}

LightMatch::~LightMatch(){
	#ifdef LIGHT_CNT_TIME
		std::cout<< "Time for threshold:" << (double)(time_sum / _cnt) << std::endl;
	#endif
}

void LightMatch::setEnemyColor(const bool _enemy_blue){
	enemy_blue = _enemy_blue;
	if(enemy_blue){								//设置颜色阈值
		thresh_low = aim_deps::light_params.blue_thresh_low;
		thresh_high = aim_deps::light_params.blue_thresh_high;	
	}
	else{
		thresh_low = aim_deps::light_params.red_thresh_low;
		thresh_high = aim_deps::light_params.red_thresh_high;
	}
}

void LightMatch::reset(){
	matches.clear();
	possibles.clear();
	trapezoids.clear();
}

void LightMatch::findPossible(){								//找出所有可能灯条，使用梯形匹配找出相匹配的灯条对
	reset();
	cv::Mat binary(1080, 1440, CV_8UC1);
	threshold(binary, 120);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);	//寻找图上轮廓
	int _cnt = 0;
	for (std::vector<cv::Point> contour: contours) {	
		float area = cv::contourArea(contour);
		doubleThresh(_cnt, area, contour, binary);
	}
	#ifdef LIGHT_MATCH_DEBUG
		cv::imshow("binary", binary);
	#endif
	std::vector<cv::Point2f *> _vexes;
	for (aim_deps::Light light: possibles) {
		cv::Point2f* corners = new cv::Point2f[2];
		corners[0] = light.box.vex[0];
		corners[1] = light.box.vex[1];
		_vexes.emplace_back(corners);
		getTrapezoids(corners);								
	}
	getRealLight(possibles.size(), _vexes);
}

void LightMatch::getRealLight(const int size, const std::vector<cv::Point2f *> vexes){
	bool flag[size][size];					//两灯条是否满足要求,是个对称矩阵，当[i][j],[j][i]为真时，两灯条匹配
	for (int i = 0; i < size; ++i) {		//初始化		
        for(int j = 0; j < size; ++j)
		    flag[i][j] = false;
	}
	for (int i = 0; i < size; ++i) {		//梯形包含匹配
		for (int j = 0; j<size*2; ++j) {
			if (isInTrapezoid(vexes[i], trapezoids[j])) {
				flag[i][j/2] = true;
			}
		}
		delete vexes[i];
	}
	for (int i = 0; i<size; ++i) {						//将可能的匹配结果放入容器matches中
		for (int j = i+1; j<size; ++j) {
			if (flag[i][j] == true && flag[j][i] == true) {
				flag[i][j] = false;
				matches.emplace_back(cv::Point(i, j));		//最后只保存配对的信息，灯条只保存一次
			}
		}
	}
}

void LightMatch::doubleThresh(int &index, const float area, std::vector<cv::Point> ct, cv::Mat &bin){
	cv::Rect bbox = cv::boundingRect(ct);
	/// printf("index %d\n", index);
	if(bbox.area() > 10 * area){
		/// printf("False selection.\n");
		/// printf("Area: %f, ratio: %f for index %d\n", bbox.area() / area, (float)(bbox.height / bbox.width), index);
		return;
	}
	cv::Mat tmp;
	if(enemy_blue) tmp = proced[0](bbox);
	else tmp = proced[2](bbox);
	cv::Mat _bin;
	cv::RotatedRect tmp_rec = cv::minAreaRect(ct);
	float _a = tmp_rec.size.area();
	if(_a < 4.0) return;			///面积过小舍去
	// printf("Light %d area: %f\n", index, _a);
	int th = getDoubleThresh(_a);
	cv::threshold(tmp, _bin, th, 255, cv::THRESH_BINARY);
	cv::erode(_bin, _bin, cv::getStructuringElement(0, cv::Size(1, 1)));
	//_bin.copyTo(bin(bbox));				//不需要把二次阈值化的二值图绘制在原二值图上时取消注释
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(_bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	for(std::vector<cv::Point> contour: contours){
		cv::RotatedRect light = cv::minAreaRect(contour);
		if(light.size.area() > 0.1 * bbox.area()){		//与boundingBox面积不匹配的可能是噪点
			// 二次阈值化会带来一定的长度损失
			if(light.size.height >= light.size.width){
				light.size.height *= 1.1;
			}
			else light.size.width *= 1.1;
			light.center.x += bbox.x;
			light.center.y += bbox.y;
			aim_deps::Light _l(index, light, tmp_rec.center);
			cv::Point2f vect = _l.box.vex[0] - _l.box.vex[1];
			if( abs(vect.y) < 4.0) return;
			possibles.emplace_back(_l);
			//printf("Light %d: outter_box angle:%f, inner_angle: %f\n", index,
			//	tmp_rec.angle, possibles.back().box.angle);
			++index;
			return;
		}
	}
}

//匹配的灯条其梯形将会互相包含
/// TODO: 重构需要的修改
void LightMatch::getTrapezoids(const cv::Point2f corners[2]){
	std::vector<cv::Point2f> left_trapezoid;
	std::vector<cv::Point2f> right_trapezoid;
	cv::Point2f midpoint = (corners[0] + corners[1]) /2;
	cv::Point2f direction_vector;							//方向向量
	float d = getDistance(corners[0], corners[1]);
	cv::Point2f vertical_vector = cv::Point2f(corners[1].y - corners[0].y,
			corners[0].x - corners[1].x);						//获得垂直长方向的方向向量
	vertical_vector = d * aim_deps::LIGHT_PARAM1 * vertical_vector /
			sqrt(vertical_vector.x * vertical_vector.x + vertical_vector.y * vertical_vector.y);
	direction_vector = corners[1] - corners[0];					//平行于长边方向的方向向量
	left_trapezoid.emplace_back(corners[0]);
	left_trapezoid.emplace_back(corners[1]);
	left_trapezoid.emplace_back(midpoint - vertical_vector + aim_deps::LIGHT_PARAM2 * direction_vector);
	left_trapezoid.emplace_back(midpoint - vertical_vector - aim_deps::LIGHT_PARAM2 * direction_vector);
	right_trapezoid.emplace_back(corners[0]);
	right_trapezoid.emplace_back(corners[1]);
	right_trapezoid.emplace_back(midpoint + vertical_vector + aim_deps::LIGHT_PARAM2 * direction_vector);
	right_trapezoid.emplace_back(midpoint + vertical_vector - aim_deps::LIGHT_PARAM2 * direction_vector);
	trapezoids.emplace_back(left_trapezoid);						//灯条左右两边将会拓展出两个梯形
	trapezoids.emplace_back(right_trapezoid);
}

void LightMatch::drawLights(cv::Mat &src){
	char str[2];
	for(size_t i = 0; i < possibles.size() ; ++i){
		cv::line(src, possibles[i].box.vex[0], possibles[i].box.vex[1], cv::Scalar(0, 0, 255), 3);
		//snprintf(str, 4, "%lu", j);
		//cv::putText(src, str, pts[j]+cv::Point2f(1,1), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
		snprintf(str, 2, "%lu", i);
		cv::putText(src, str, possibles[i].box.vex[0] + cv::Point2f(1, 1), 
					cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
	}
}

void LightMatch::threshold(cv::Mat &dst, const int thresh, const int diff_thresh){
	#ifdef LIGHT_CNT_TIME
		double start_t = cv::getTickCount();
	#endif
	cv::Mat _filter;
	if(enemy_blue){
		_filter = proced[2] - proced[0];
	}else{
		_filter = proced[0] - proced[2];
	}
	// 以敌人蓝色为例，r - b 得到_filter, filter 内大于15的全设置为255
	// 因为 r - b, 红色的部分值为正，b - r 本来应该为负，但是因为是uchar 并且个人估计会进行saturate_cast
	// 负数变成 0, 那么进行阈值操作, 所有大于15的值设为255，其他的为0，把图像减去filter,就是过滤掉红色后的图
	// 再进行一次threshold操作，得到最终的二值图
	// 并且速度比我自己写的threshold 快很多（1ms）。。。我三线程都不如人家快啊(4.5ms) opencv threshold (0.001ms)
	cv::threshold(_filter, _filter, diff_thresh, 255, cv::THRESH_BINARY);		
	if(enemy_blue){
		dst = proced[0] - _filter;
	}
	else{
		dst = proced[2] - _filter;
	}
	cv::threshold(dst, dst, thresh, 255, cv::THRESH_BINARY);
	#ifdef LIGHT_CNT_TIME
		double end_t = cv::getTickCount();
		time_sum += (end_t - start_t) / cv::getTickFrequency() * 1000; 
		++_cnt;
	#endif
}

bool LightMatch::isInTrapezoid(cv::Point2f corners[2], const std::vector<cv::Point2f> &trapezoid){
	for (int i = 0; i<2; ++i) {
		if (!(cv::pointPolygonTest(trapezoid, corners[i], false) >= 0))
			return false;	
	}
	/// 2个点都找到，才能返回true
	return true;
}

float LightMatch::getDistance(const cv::Point p1, const cv::Point p2){
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

int LightMatch::getDoubleThresh(const float _a){
	if(_a >= 50.0f) return 209.0f;			//原39
	else if(_a <= 16.3f) return 119.0f;
	return (int)( - 0.000164f * powf(_a, 4) + 0.02328f * powf(_a, 3) - 1.2157f * _a * _a +
			29.8154f * _a - 127.85307f); 
}

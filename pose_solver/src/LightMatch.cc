/*
灯条检测模块-找到图上所有合适的灯条,并且将其匹配为装甲板
实现者：雷达组-zlq
代码思路：雷达组-dxd
修改：雷达组-dxd
封装：hqy
last date of modification:2020.4.7
*/

#include "../include/LightMatch.hpp"

LightMatch::LightMatch(){
	#ifdef LIGHT_CNT_TIME		
		time_sum 		= 0.0;
		cnt 			= 0;
	#endif // LIGHT_CNT_TIME
	#ifdef ANALYSIS
		lg.init("/home/sentinel", "reflection", rmlog::REAL_TIME_W);
		cnter = 0;			//计算帧数
	#endif // ANALYSIS
	low_exposure 	= false;
	mean_val 		= aim_deps::LIGHT_mean;
	setEnemyColor(true);												//初始设置敌人颜色
}

LightMatch::~LightMatch(){
	#ifdef LIGHT_CNT_TIME
		std::cout<< "Time for threshold:" << (double)(time_sum / cnt) << std::endl;
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
	#ifdef ANALYSIS
		cnter += 2;
	#endif
	#ifdef LIGHT_CNT_TIME
		double start_t = cv::getTickCount();
	#endif
	reset();
	cv::Mat binary(1080, 1440, CV_8UC1);
	threshold(binary, 95);			//120
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);	//寻找图上轮廓
	int _cnt = 0;
	for (std::vector<cv::Point> contour: contours) {	
		float area = cv::contourArea(contour);
		cv::Rect bbox = cv::boundingRect(contour);
		if(bbox.area() <= 20 * area && bbox.area() >= 4 && bbox.height > 3){
			if(bbox.area() < 400){
				cv::Mat cnt_mat;
				if(enemy_blue){
					cv::threshold(proced[0](bbox), cnt_mat, 210, 255, cv::THRESH_BINARY);
				}
				else{
					cv::threshold(proced[2](bbox), cnt_mat, 210, 255, cv::THRESH_BINARY);
				}
				int num = cv::countNonZero(cnt_mat);				// 排除反光灯条的影响
				// 反光灯条的特征是：灯条内亮度过小，灰度值基本不大于210，大于的也绝大多数只有一点
				bool valid = (num >= 2);
				doubleThresh(_cnt, contour, bbox, valid);
			}
			else{		// 选框够大，说明灯条无需二次阈值，亚像素检测以及角度修正
				cv::RotatedRect l = cv::minAreaRect(contour);
				aim_deps::Light _l(_cnt, l, l.center, cv::max(l.size.height, l.size.width), true);		
				if( std::abs(_l.box.angle) < 40.0){
					possibles.emplace_back(_l);
					++_cnt;
				}
			}
		}
	}
	//#ifdef LIGHT_MATCH_DEBUG
	// cv::imshow("binary", binary);
	//#endif
	std::vector<cv::Point2f *> _vexes;
	for (aim_deps::Light light: possibles) {
		cv::Point2f* corners = new cv::Point2f[2];
		corners[0] = light.box.vex[0];
		corners[1] = light.box.vex[1];
		_vexes.emplace_back(corners);
		getTrapezoids(corners);								
	}
	getRealLight(possibles.size(), _vexes);
	#ifdef LIGHT_CNT_TIME
		double end_t = cv::getTickCount();
		time_sum += (end_t - start_t) / cv::getTickFrequency() * 1000; 
		++cnt;
	#endif
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

void LightMatch::doubleThresh(int &index, std::vector<cv::Point> ct, cv::Rect bbox, const bool valid){
	cv::RotatedRect tmp_rec = cv::minAreaRect(ct);
	cv::Size subpix_sz = decideSize(tmp_rec);
	bool do_subpixel = extendRect(bbox, subpix_sz + cv::Size(2, 2));
	cv::Mat tmp;
	if(enemy_blue) tmp = proced[0](bbox);
	else tmp = proced[2](bbox);
	cv::Mat _bin;
	float _a = tmp_rec.size.area() * 0.95 ;			//本来以120的阈值进行一次阈值化，确定了getDoubleThresh函数 现在改成100
	if(_a < 6.0) return;							///面积过小舍去
	int th = getDoubleThresh(_a);
	cv::threshold(tmp, _bin, th, 255, cv::THRESH_BINARY);
	cv::erode(_bin, _bin, cv::getStructuringElement(0, cv::Size(1, 1)));
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(_bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	for(std::vector<cv::Point> c: contours){
		cv::RotatedRect rot_box = cv::minAreaRect(c);
		if(rot_box.size.area() > 0.015 * bbox.area()){		//与boundingBox面积不匹配的可能是噪点
			std::vector<cv::Point2f> contour;
			convertVector(c, contour);						//将c(cv::Point)转化为cv::Point2f类型
			match_debug(rmlog::F_BLUE, "Light", index, " l before subpixel:",
						cv::max(rot_box.size.width, rot_box.size.height));
			if(contour.size() < 72 && do_subpixel){
				cv::TermCriteria criteria = cv::TermCriteria(
						cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.01);
				cv::cornerSubPix(tmp, contour,
						subpix_sz, cv::Size(-1, -1), criteria);
			}
			else{
				do_subpixel = false;
			}
			cv::RotatedRect light = cv::minAreaRect(contour);
			if(light.size.height >= light.size.width && !do_subpixel){
				light.size.height *= 1.1;
			}
			else if ( !do_subpixel ){
				light.size.width *= 1.1;
			}
			light.center.x += bbox.x;
			light.center.y += bbox.y;
			aim_deps::Light _l(index, light, tmp_rec.center,
					cv::max(tmp_rec.size.height, tmp_rec.size.width), valid);
			if( !isAngleValid(_l.box) ) return;		//灯条角度不符合要求
			std::vector<cv::Point2f> direction(4);
			bool judge = extractDirect(tmp, direction, _l.box.length);
			if(judge == true)			
				readjustAngle(c, _l, cv::Point(bbox.x, bbox.y), &direction);
			else
				readjustAngle(c, _l, cv::Point(bbox.x, bbox.y));
			//printf("Light %d: len %f, angle %f\n", index, _l.box.length, _l.box.angle);
			possibles.emplace_back(_l);
			++index;
			return;
		}
	}
	aim_deps::Light _l(index, tmp_rec, tmp_rec.center,
					cv::max(tmp_rec.size.height, tmp_rec.size.width), valid);
	if( !isAngleValid(_l.box) ) return;		//灯条角度不符合要求
	if( _l.box.length > 8){							//长度过小，不必修正角度
		readjustAngle(ct, _l, cv::Point(0, 0), nullptr, true);		//一次阈值化的外包络轮廓点不够准，削弱角度调整
	}
	//printf("(*)Light %d: len %f, angle %f\n", index, _l.box.length, _l.box.angle);
	possibles.emplace_back(_l);
	++index;
}

//匹配的灯条其梯形将会互相包含
void LightMatch::getTrapezoids(const cv::Point2f corners[2]){
	std::vector<cv::Point2f> left_trapezoid;
	std::vector<cv::Point2f> right_trapezoid;
	cv::Point2f midpoint = (corners[0] + corners[1]) /2;
	cv::Point2f direction_vector;							//方向向量
	float d = sqrt(aim_deps::getPointDist(corners[0], corners[1]));
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

void LightMatch::drawLights(cv::Mat &src) const{
	char str[2];
	for(size_t i = 0; i < possibles.size() ; ++i){
		if(possibles[i].valid){				//可能是反光的灯条不绘制
			cv::line(src, possibles[i].box.vex[0], possibles[i].box.vex[1], cv::Scalar(0, 0, 255), 1);
		}
		else{
			cv::line(src, possibles[i].box.vex[0], possibles[i].box.vex[1], cv::Scalar(255, 10,  100), 1);
		}
		//snprintf(str, 4, "%lu", j);
		//cv::putText(src, str, pts[j]+cv::Point2f(1,1), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
		snprintf(str, 2, "%lu", i);
		cv::putText(src, str, possibles[i].box.vex[0] + cv::Point2f(1, 1), 
					cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 100, 255));
		cv::circle(src, possibles[i].box.vex[0], 0, cv::Scalar(255, 0, 255), -1);
		cv::circle(src, possibles[i].box.vex[1], 0, cv::Scalar(255, 0, 255), -1);
		match_debug(rmlog::F_GREEN, "Light ", possibles[i].index, " with lenth: ", possibles[i].box.length,
			", angle: ", possibles[i].box.angle);
	}
}

void LightMatch::threshold(cv::Mat &dst, const int thresh, const int diff_thresh) const{
	cv::Mat _filter;
	if(enemy_blue){
		_filter = proced[0] - proced[2];
	}else{
		_filter = proced[0] - proced[2];
	}
	cv::threshold(_filter, _filter, diff_thresh, 255, cv::THRESH_BINARY_INV);		
	if(enemy_blue){
		dst = proced[0] - _filter;
	}
	else{
		dst = proced[2] - _filter;
	}
	cv::threshold(dst, dst, thresh, 255, cv::THRESH_BINARY);
}

bool LightMatch::extractDirect(cv::Mat src, std::vector<cv::Point2f> &vexes, const float len){
	cv::Mat direct;
	cv::threshold(src, direct, 237, 255, cv::THRESH_BINARY);			//这个阈值(经验阈值)
	std::vector<std::vector<cv::Point> > d_cts;
	cv::findContours(direct, d_cts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	for(std::vector<cv::Point> c : d_cts){
		cv::RotatedRect dir = cv::minAreaRect(c);
		float max_len = cv::max(dir.size.height, dir.size.width);
		if(max_len >= 4.5){								// 最大长太小，说明是噪点的包络
			if(len / max_len >= 2.8) return false;		// 方向矩形的长度过短，置信度低，不使用
			if(dir.size.height >= dir.size.height){		//如果方向矩形的宽度太小，无法包络足够的轮廓点，则增大之
				dir.size.height *= 1.6;
				if(dir.size.width <= 2.7){
					dir.size.width = 2.7;
				}
				dir.size.width *= 1.2;
			}
			else{
				dir.size.width *= 1.6;
				if(dir.size.height <= 2.7){
					dir.size.height = 2.7;
				}
				dir.size.height *= 1.2;
			}
			cv::Point2f corners[4];
			dir.points(corners);
			for(int i = 0; i < 4; ++i){
				vexes[i] = corners[i];
			}
			return true;
		}
	}
	return false;
}

bool LightMatch::isInTrapezoid(cv::Point2f corners[2], const std::vector<cv::Point2f> &trapezoid){
	for (int i = 0; i<2; ++i) {
		if (cv::pointPolygonTest(trapezoid, corners[i], false) < 0)
			return false;	
	}
	/// 2个点都找到，才能返回true
	return true;
}

int LightMatch::getDoubleThresh(const float _a){
	if(_a >= 50.0f) return 209.0f;			//209
	else if(_a <= 16.3f) return 119.0f;			// 119
	return (int)( - 0.000164f * powf(_a, 4) + 0.02328f * powf(_a, 3) - 1.2157f * _a * _a +
			29.8154f * _a - 127.85307f); 			//127.85307f
}

bool LightMatch::extendRect(cv::Rect &rect, const cv::Size sz)	{		/// 长宽按照中心扩大
	bool within = true;							//选框是否超出边界？超出则无法获得足够大的ROI进行亚像素检测
	if(rect.x - sz.width < 0){
		rect.x = 0;
		within = false;
	}
	else rect.x -= sz.width;
	if(rect.y - sz.height < 0){
		rect.y = 0;
		within = false;
	}
	else rect.y -= sz.height;
	if( rect.x + 2 * sz.width + rect.width >= 1440){
		rect.width = 1439 - rect.x;
		within = false;
	}
	else rect.width += 2 * sz.width;
	if( rect.y + 2 * sz.height + rect.height >= 1080){
		rect.height = 1079 - rect.y;
		within = false;
	}
	else rect.height += 2 * sz.height;
	return within;
}

/// ====================灯条优化的实现尝试====================== /// 
void LightMatch::readjustAngle(
	std::vector<cv::Point> contour,
	aim_deps::Light &l,
	cv::Point offset,
	std::vector<cv::Point2f> *contain,
	const bool weaken
){
	// 对于特征点较多的灯条，我们认为minAreaRect计算准确
	// 对于特征点较少的灯条，我们认为不能使用优化（很可能点集中在中点处，造成优化出错）
	if(contour.size() > 71 && contour.size() <= 6) {
		return;	
	}
	double angle = 0.0;
	cv::Point2f nv0;
	cv::Point2f lv0 = (l.box.vex[1] - l.box.center) /
			sqrt(aim_deps::getPointDist(l.box.center, l.box.vex[1]));		
	nv0.x = lv0.y;
	nv0.y = -lv0.x;		
	std::vector<bool> valids(contour.size());
	for(size_t i = 0; i < contour.size(); ++i){
		valids[i] = true;
	}
	if(contain != nullptr){
		for(size_t i = 0; i < contour.size(); ++i){
			if(cv::pointPolygonTest(*contain, contour[i], false) < 0)
				valids[i] = false;
		}
	}										
	for(int i = 0; i < 30; ++i){
		double diff_sum = 0.0, diff2_sum = 0.0;						// error_sum = 0.0;
		cv::Point2f lv = aim_deps::Rotate(lv0, angle);				
		for(size_t i = 0; i < contour.size(); ++i){
			if(valids[i] == true){
				diff_sum += calcDiff(lv, l.box.center, contour[i] + offset);		//计算一阶导
				diff2_sum += calcDiff2(lv, l.box.center, contour[i] + offset);	//计算二阶导(二阶的效果显著好于一阶)
			}
		}		
		// printf("Light %d iter %d: diff_sum: %f, diff2_sum: %f\n", l.index, i, diff_sum, diff2_sum);
		if(diff2_sum == 0.0) break;
		angle -= diff_sum / diff2_sum;				// 牛顿迭代
		if(std::abs(diff_sum) <= 0.5) 
		{
			break;
		}
	}
	if(weaken){
		l.box.rotate(angle / 2);
	}
	else{
		l.box.rotate(angle);
	}
	#ifdef DRAW_CONTOUR
		for(size_t i = 0; i < contour.size(); ++i){
			contour[i] += offset;
		}
		cts_todraw.emplace_back(contour);
		cts_valids.emplace_back(valids);
	#endif
}

bool LightMatch::isAngleValid(const aim_deps::LightBox &lb){
	if(lb.length < 4) return false;
	if(lb.length < 8)
		return (std::abs(lb.angle) <= 10.0);
	else
		return (std::abs(lb.angle) <= 40.0);
}

double LightMatch::calcError(cv::Point2f _vec, cv::Point2f ctr, cv::Point p){
	cv::Point2f diff = cv::Point2f(p) - ctr;			// 计算点到定点的向量
	double res = diff.ddot(_vec);						// 与法向量做内积
	return res * res / 2;
}

/// 暂时把 _vec 认为是 (p, q) 如果有问题再回来改
double LightMatch::calcDiff(cv::Point2f _vec, cv::Point2f ctr, cv::Point p){
	cv::Point2f d = cv::Point2f(p) - ctr;
	double diff = pow(d.x, 2) * _vec.x * _vec.y - pow(d.y, 2) * _vec.x * _vec.y +
			d.x * d.y * pow(_vec.y, 2) - d.x * d.y * pow(_vec.x, 2);	// 导数公式
	return diff;
}

double LightMatch::calcDiff2(cv::Point2f _vec, cv::Point2f ctr, cv::Point p){
	cv::Point2f d = cv::Point2f(p) - ctr;
	double diff = pow( d.x * _vec.x + d.y * _vec.y, 2) - pow( d.x * _vec.y - d.y * _vec.x, 2); 
	return diff;
}

cv::Size LightMatch::decideSize(const cv::RotatedRect rect){
	float maxi = 0.0, mini = 0.0;
	float angle = std::abs( cv::min(rect.angle, 90 + rect.angle) ) * aim_deps::DEG2RAD;
	if(rect.size.width > rect.size.height){
		maxi = rect.size.width;
		mini = rect.size.height;
	}
	else{
		maxi = rect.size.height;
		mini = rect.size.width;
	}
	int x = 1, y = 2;
	if(maxi >= 24) y = 9;								// 过大则固定
	else if(maxi > 5){									// 一个拟合函数
		y = std::round(-0.0062138 * maxi * maxi + 0.5680351 * maxi + -1.50083226);
		
	}
	if(mini == 1) x = 1;
	else if(mini < 2) x = 2;
	else x = 3;
	y -= (int)(1 - cos(angle)) * 0.5 ;			//稍微考虑矩形旋转角影响
	x += (int) 0.5 * sin(angle);				
	return cv::Size(x, y);
}

void LightMatch::convertVector(std::vector<cv::Point> src, std::vector<cv::Point2f> &dst){
	dst.resize(src.size());
	for(size_t i = 0; i < src.size(); ++i){
		dst[i] = cv::Point2f(src[i]);
	}
}

#ifdef DRAW_CONTOUR
void LightMatch::drawContour(cv::Mat &src){
	for(size_t i = 0; i < cts_todraw.size(); ++i){
		for(size_t j = 0; j < cts_todraw[i].size(); ++j){
			if(cts_valids[i][j] == true){
				cv::circle(src, cts_todraw[i][j], 1, cv::Scalar(0, 255, 255));
			}
		}
	}
	cts_todraw.clear();
}
#endif	// DRAW_CONTOUR
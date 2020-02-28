# camera_map_pose_solver
Positioning using fixed camera and calibrated map. 
> * 2020.2.25 修改  
>> * 加入了一个branch文件夹，其中有最新的ArmorPlate.hpp, LightMatch.hpp, GetPos.hpp  
>> * 其中还有其他的文件，比如:AimDeps.cc，储存有装甲板结构体的定义，以及GimbalCtrl.hpp，是弹道模型模块  
>> * 主函数发生了改动，其中，“///”这样的注释都是我加的，每一处修改过的位置都有  
>> * 修改了有关amp与match的接口，以及GetPos实例放在了ArmorPlate外部,解算与检测分离  
>> * 更改了rMats与tMats的获取方法  
>> 2020.2.25 还需要做：装甲板检测角度匹配修改  

> * 2020.2.26 修改
>> * 修改了装甲板匹配算法ArmorPlate.hpp, 加入了两个点的角度计算，重新设置了比例以及角度阈值(ANGLE_THRESH),但是还是会有误判情况（比原来好了很多）（误判出现在视频后半段）
>> * 修改了AimDeps.cc, 把装甲板结构体内的一个变量r_vec改为了Mat, 方便camera_map的操作
> * 2020.2.27 & 2020.2.28 早上修改
>> * 加入LightMatch敌方颜色更改，AimDeps.hpp整合了一些参数

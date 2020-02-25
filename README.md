# camera_map
Positioning using fixed camera and calibrated map. 
> 2020.2.25 修改  
>> * 加入了一个branch文件夹，其中有最新的ArmorPlate.hpp, LightMatch.hpp, GetPos.hpp  
>> * 其中还有其他的文件，比如:AimDeps.cc，储存有装甲板结构体的定义，以及GimbalCtrl.hpp，是弹道模型模块  
>> * 主函数发生了改动，其中，“///”这样的注释都是我加的，每一处修改过的位置都有  
>> * 修改了有关amp与match的接口，以及GetPos实例放在了ArmorPlate外部,解算与检测分离  
>> * 更改了rMats与tMats的获取方法  
> 2020.2.25 还需要做：装甲板检测角度匹配修改

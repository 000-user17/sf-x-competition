顺丰sf-x竞赛项目，代码文件在/code/中，最终最好的版本为submit_version11，全国半决赛排名15，总决赛排名：
思路：
1.将地图转化为python表示的数据结构
2.在搬运货物前先判断是否有货物放在非目标货架上，如果有则把货物先放下来
3.构建地图
4.利用bfs算法将AGV运送货物的非冲突路径保存（非冲突：1.每一条路径不包含任意一条路径的终点，每一条路径的前一时刻，后一时刻，当前时刻不包含在相同时刻其他AGV经过的相同位置）
5.反复执行AGV寻路，运送过程
6.如果没有找到路径，进入死锁安全模式，所有AGV随机运动，直到解除死锁
7.安全模式寻路，每一个AGV的路径经过的点互不相同
8.运输完成

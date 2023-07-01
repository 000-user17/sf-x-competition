'''由test.py复制过来'''
import random
def class_init(map_info):
    
    '''存储agv相关信息'''
    class Agv:
        def __init__(self, agv_id, payload, cap, loc=[-1,-1], occupy=-1):
            self.loc = loc #列表类型，如[1,1]，表示坐标
            self.id = agv_id  #agv的id
            self.payload = payload #是否载货，初始化为-1
            self.cap = cap  #容量，能载几个
            self.occupy = occupy #是否正在前去/已经装载货物的编号，初始值为-


    '''存储货物相关信息'''
    class Cargo:
        def __init__(self, cargo_id, target, loc=[-1,-1], load=-1):
            self.loc = loc #列表类型，如[1,1]，表示坐标
            self.id = cargo_id  #cargo的id
            self.target = target  #目标货架
            self.load = load #是否正要被前往装载，或者已经被AVG装载，初始化为-1

    class Shelf:
        def __init__(self, shelf_id, payload, loc=[-1,-1]):
            self.loc = loc #列表类型，如[1,1]，表示坐标
            self.id = shelf_id  #shelf的id
            self.payload = payload
            
            
    class sf:
        
        def __init__(self, map_json):
            self.map_json = map_json
            self.ACTIONS_SEQ1 = []
            
            self.map_data = [] #存放map数据，先随机指定类型
            self.map_width = 0 #地图宽度
            self.map_height = 0 #地图高度
            self.max_steps = 0 #最大步数
            self.timeiout = 0 #单次决策最大时间
            
            

            self.shelfs = {} #存放货架相关信息,id作为key
            self.agvs = {} #存放Agv相关信息, id作为key
            self.cargos = {} #存放cargo相关信息， target作为key
            self.obstacles = set() #存放walls,以及其他物体的的坐标,都可以视为障碍物
            
            self.path_obstacles = set() #用于bfs的障碍推断
            self.agvs_paths = {} #用于保存各个agvs的运货路径
            
            self.position_dict = {}  #保存每一个方格对应的下标
            self.position_dict_rever = {} #每一个下标对应的方格坐标
            self.graph = []

        '''
        优化方向：agv运输的次序, 序号优先，还是距离最近的优先行动，或者是随机顺序行动
        寻路方式，除了bfs，优化的bfs，A*算法
        寻找距离最短的agv去运输货物
        死锁处理方式
        '''
        
        
        '''构建graph'''
        def construct_graph(self):
            self.graph = []
            '''将网格地图转化为图的形式'''
            pos_idx = 0
            for i in range(self.map_height):
                for j in range(self.map_width):
                    self.position_dict[(i, j)] = pos_idx
                    self.position_dict_rever[pos_idx] = (i, j)
                    pos_idx += 1
            '''
            0 1 2 3 4 5 6
            7 8 9 10 11 12 13
            ...这样的序号
            '''
            inf = float('inf')
            for vertex in self.position_dict:
                #print(vertex)
                self.graph.append([inf for k in range(len(self.position_dict))])
                idx = len(self.graph)-1
                # if vertex in self.obstacles: #如果该点是障碍物，则跳过，该点不能到任何周围位置
                #     #print(self.graph[idx])
                #     continue  
                for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                    (pos_x, pos_y) = (vertex[0]+dx, vertex[1]+dy)
                    if (pos_x, pos_y) in self.position_dict and (pos_x, pos_y) not in self.obstacles:
                        self.graph[idx][self.position_dict[(pos_x, pos_y)]] = 1 #相邻顶点连通，且距离为1
                self.graph[idx][idx] = 0
                #print(self.graph[idx])
            #print(self.graph)
            
            # for i in range(len(self.graph)):
            #     for j in range(len(self.graph)):
            #         if self.graph[i][j] == 0:
            #             print(str(i) +" "+str(j) )
                
                
                
        '''将新的路径作用于graph, 添加path为障碍物'''
        def update_graph(self, path):
            inf = float('inf')
            for obs in path:
                #如果某一点是障碍，则周围的点都不能走到该点
                obs_idx = self.position_dict[obs]
                for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                    obs_around = (obs[0]+dx, obs[1]+dy)
                    if obs_around in self.position_dict:
                        obs_around_idx = self.position_dict[obs_around]
                        self.graph[obs_around_idx][obs_idx] = inf
                        self.graph[obs_idx][obs_around_idx] = inf
            


        '''通过地图的json文件构造地图,存放AGV货物等的信息'''
        def map_construct(self):
            self.map_data = self.map_json
            self.map_width = self.map_data['value']['map_attr']['width']
            self.map_height = self.map_data['value']['map_attr']['height']
            self.max_steps = self.map_data['value']['map_attr']['max_steps']
            self.timeiout = self.map_data['value']['map_attr']['timeout']
            
                

            '''存放地图中agv信息'''
            for agv_info in self.map_data['value']['map_state']['agvs']:

                agv_id = agv_info['id']
                payload = (-1 if agv_info['payload'] is None else agv_info['payload']) #如果初始shelf没有货物，则为-1
                cap = agv_info['cap']
                self.agvs[agv_id] = Agv(agv_id, payload, cap)

            '''存放地图中cargo信息'''
            for cargo_info in self.map_data['value']['map_state']['cargos']:
                cargo_id = cargo_info['id']
                target = cargo_info['target']
                self.cargos[cargo_id] = Cargo(cargo_id, target)
            
            '''存放地图中shelf信息'''
            for shelf_info in self.map_data['value']['map_state']['shelves']:
                shelf_id = shelf_info['id']
                payload = (-1 if shelf_info['payload'] is None else shelf_info['payload']) #如果初始shelf没有货物，则为-1
                self.shelfs[shelf_id] = Shelf(shelf_id, payload)
            
            '''将各个物体的位置信息加入'''
            for loc_info in self.map_data['value']['map_state']['map']:
            
                if loc_info['type'] == 'agv':

                    agv_id = loc_info['id']
                    x = loc_info['y']
                    y = loc_info['x']
                    self.agvs[agv_id].loc = [x, y]
                    self.obstacles.add((x, y))
                    
                elif loc_info['type'] == 'cargo':
                    cargo_id = loc_info['id']
                    x = loc_info['y']
                    y = loc_info['x']
                    self.cargos[cargo_id].loc = [x, y]
                    self.obstacles.add((x, y))
                    
                elif loc_info['type'] == 'shelf':
                    shelf_id = loc_info['id']
                    x = loc_info['y']
                    y = loc_info['x']
                    self.shelfs[shelf_id].loc = [x, y]
                    self.obstacles.add((x, y))
                else:
                    x = loc_info['y']
                    y = loc_info['x']
                    self.obstacles.add((x, y))
            
            for i in range(self.map_height):
                self.obstacles.add((self.map_height, i))
            for i in range(self.map_width):
                self.obstacles.add((i, self.map_width))
                
                    
            for i in self.cargos:
                if self.cargos[i].loc == [-1, -1]:
                    for j in self.shelfs:
                        if self.shelfs[j].payload == i:
                            self.cargos[i].loc = self.shelfs[j].loc
                            break
            
            self.construct_graph()
    
    
        
        '''AGV运输情况'''
        def before_load(self, shelf_id):
            agv_id = 0
                        
            while agv_id<len(self.agvs) :
                if self.agvs[agv_id].payload != -1 or self.agvs[agv_id].occupy != -1: #当前AVG在装载或是正在去装载的路上
    
                    #if self.agvs_paths[agv_id] != []:      
                    agv_id += 1
                    continue

                start = self.agvs[agv_id].loc
                end = self.shelfs[shelf_id].loc
                # print(agv_id, start)
                # print(cargo_id, end)
                #寻找可行的一个货物周围的位置，从而运输 
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    #print(self.path_obstacles)
                    
                    path = self.bfs(start, [end[0]+dx, end[1]+dy]) 
                    if path != []: #找到了货物周围的一条可行线路
                        end = [end[0]+dx, end[1]+dy]
                        break
                self.agvs_paths[agv_id] = path #保存对应AGV的运输线路[(1,1), (2,2),...]
                if path == []:
                    agv_id += 1
                    continue   #无可行路径，不予分配agv和cargo
                #正常分配agv给cargo
                self.agvs[agv_id].occupy = self.shelfs[shelf_id].payload
                self.cargos[self.shelfs[shelf_id].payload].load = agv_id
                return agv_id

            

        
        
        def shelf_occupy(self):
            for i in self.shelfs:
                if self.shelfs[i].payload != -1:
                    return True
            return False
        
        def deload_from_shelf(self):
            
            while self.shelf_occupy():
                for shelf_id in self.shelfs:
                    if self.shelfs[shelf_id].payload == -1:
                        continue
                    print("1")
                    
                    agv_occu = self.before_load(shelf_id)
                    
                    print("2")
                    print(agv_occu)
                    print(self.agvs_paths[agv_occu])
                    if agv_occu < len(self.agvs):
                        while self.agvs_paths[agv_occu] != []:
                            self.forward()
                        '''已经装货'''
                        
                        self.shelfs[shelf_id].payload = -1 #增加一条
                        self.agvs[agv_occu].payload = self.agvs[agv_occu].occupy
                        print("AGV"+str(agv_occu)+"运送的货物"+ str(self.agvs[agv_occu].payload))
                        
                        
                        start = self.agvs[agv_occu].loc
                        (end_x, end_y) = tuple(start)
                        
                            
                        print(self.obstacles)
                        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  
                            next_x = end_x + dx
                            next_y = end_y + dy
                            next_point = (next_x, next_y)
                            print(next_point)
                            if next_point not in self.obstacles:
                                
                                action_idx = len(self.ACTIONS_SEQ1)
                                self.ACTIONS_SEQ1.append([])
                                for i in range(len(self.agvs)): #本轮所有agv的操作字典
                                    self.ACTIONS_SEQ1[action_idx].append({"type":"STAY"})
                                self.ACTIONS_SEQ1[action_idx][agv_occu]["type"] = "DELIVERY"
                                #判断卸货方向
                                direct = ""
                                agv_loc = self.agvs[agv_occu].loc
                                print(4)
                                    
                                if next_point[0]-agv_loc[0] != 0:
                                    if next_point[0]-agv_loc[0] == 1:
                                        direct = "DOWN"
                                    else:
                                        direct = "UP"
                                elif next_point[1] != agv_loc[1]:
                                    if next_point[1]-agv_loc[1] == 1:
                                        direct = "RIGHT"
                                    else:
                                        direct = "LEFT"
                                self.ACTIONS_SEQ1[action_idx][agv_occu]["dir"] = direct
                                print("装货的AGV编号"+str(agv_occu))
                                self.cargos[self.agvs[agv_occu].payload].loc = list(next_point)
                                self.obstacles.add(next_point)
                                break
                        
                        
                        
                        self.cargos[self.agvs[agv_occu].payload].load = -1
                        self.agvs[agv_occu].payload = -1
                        self.agvs[agv_occu].occupy = -1
                        #手动卸货
                        
                        
                        
                

        '''对每一张地图根据指令执行AVG各项操作'''
        def map_process(self):
            self.map_data = self.map_json #获取地图号为map_id地图信息
            '''存放地图信息'''
            self.map_construct()
            
            '''把货架上已经存在的货物搬下来'''
            
            self.paths_init()
            self.path_obstacles = self.obstacles
            self.deload_from_shelf()
            self.path_obstacles = set()
            self.paths_init()
                
            
            self.process()
            

        '''
            寻找某一位置到另一位置的可行路径
            start: 原位置，如[0,0]
            end: 目的位置，如[10,10]
            obstacles:障碍物list 如((1,1), (0,0), (2,2))
            width:地图宽度
            height:地图高度
        '''

        def bfs(self, start, end):
            if end[0]<0 or end[0]>=self.map_height or end[1]<0 or end[1]>=self.map_width: #end位置不在地图内,地图的最短宽和高都为1
                # print("边界")
                # print(str(start) +" "+ str(end))
                return []
            start = tuple(start)
            end = tuple(end)  #元组化，从而可以作为字典的键
            queue = [start]   
            visited = set(self.path_obstacles) #visit与obstacles内存不同
            distance = {start: 0}     
            parent = {start: None}   

            while queue:  
                current = queue.pop(0)  
                #print(current)
                #visited.remove(current)
                if current == end:  
                    break

                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:  
                    next_x = current[0] + dx
                    next_y = current[1] + dy
                    next_point = (next_x, next_y)

                    if next_point not in visited and (next_x >= 0) and (next_x < self.map_height) and (next_y >= 0) and (next_y < self.map_width):
                        visited.add(next_point) 
                        queue.append(next_point) 
                        distance[next_point] = distance[current] + 1 
                        parent[next_point] = current  

            if end not in parent:  
                return []
            
            path = [end]
            while path[-1] != start:
                path.append(parent[path[-1]])
            #self.path_obstacles.union(set(path))
            #print(self.path_obstacles)
            return list(reversed(path))
        

        def Dijkstra(self, start):
            # 输入是从 0 开始，所以起始点减 1
            inf = float('inf')
            node_num = len(self.graph)
            # visited 代表哪些顶点加入过
            visited = [0] * node_num
            # 初始顶点到其余顶点的距离
            dis = {node: self.graph[start][node] for node in range(node_num)}
            # parents 代表最终求出最短路径后，每个顶点的上一个顶点是谁，初始化为 -1，代表无上一个顶点
            parents = {node: -1 for node in range(node_num)}
            # 起始点加入进 visited 数组
            visited[start] = 1
            # 最开始的上一个顶点为初始顶点
            last_point = start

            for i in range(node_num - 1):
                # 求出 dis 中未加入 visited 数组的最短距离和顶点
                min_dis = inf
                for j in range(node_num):
                    if visited[j] == 0 and dis[j] < min_dis:
                        min_dis = dis[j]
                        # 把该顶点做为下次遍历的上一个顶点
                        last_point = j
                # 最短顶点假加入 visited 数组
                visited[last_point] = 1
                # 对首次循环做特殊处理，不然在首次循环时会没法求出该点的上一个顶点
                if i == 0:
                    parents[last_point] = start
                for k in range(node_num):
                    if self.graph[last_point][k] < inf and dis[k] > dis[last_point] + self.graph[last_point][k]:
                        # 如果有更短的路径，更新 dis 和 记录 parents
                        dis[k] = dis[last_point] + self.graph[last_point][k]
                        parents[k] = last_point
            # 因为从 0 开始，最后把顶点都加 1
            return {key: values for key, values in dis.items()}, {key: values for key, values in parents.items()}

        '''在执行完bfs后, 寻找到起点到某一终点的坐标路径和距离'''
        def findpath(self, dists, parents, start, end):
            cur = end
            path_idx = []
            dis = 0
            while cur != start:
                dis += dists[cur]
                path_idx.append(cur)
                cur = parents[cur]
                if cur == -1:
                    break
            if cur == -1:
                path_idx = []
            else:
                path_idx.append(start)
                list.reverse(path_idx)
            path = []
            for idx in path_idx:
                path.append(self.position_dict_rever[idx])
            return path, dis
        
        '''根据得到的路径，进行排序，并且选择出最短的一条可行路径'''
        def sortpath(self, paths, dises, cargo_ids):
            cargo_id = -1
            #先根据距离对路径和cargo_ids进行排序
            sorted_pairs = sorted(zip(dises, paths, cargo_ids))
            dises, paths, cargo_ids = zip(*sorted_pairs)
            min_dist_path = []
            
            for i in range(len(paths)):
                if paths[i] == []:
                    continue
                available = True
                for j in range(len(paths[i])):
                    if paths[i][j] in self.path_obstacles:
                        available = False #该路径不能到达
                        continue
                if available == True:
                    min_dist_path = paths[i] #找到一条最短路径，可以退出循环了
                    cargo_id = cargo_ids[i]
                    break
            return min_dist_path, cargo_id
    



        '''检测是否完成该地图,即所有货物均放在了货架上'''
        def isComplete(self):

            for cargo_id in self.cargos:
                shelf_id = self.cargos[cargo_id].target
                if cargo_id != self.shelfs[shelf_id].payload:
                    return False
            return True



        def paths_init(self):
            #初始化agvs的路径
            for agv_id in range(len(self.agvs)):
                self.agvs_paths[agv_id] = []
            
                
        '''以随即顺序对所有的agv进行调度'''
        def forward(self):
            action_idx = len(self.ACTIONS_SEQ1)
            self.ACTIONS_SEQ1.append([])
            for i in range(len(self.agvs)): #本轮所有agv的操作字典
                self.ACTIONS_SEQ1[action_idx].append({})
            
            #max_steps -= 1#地图最大步数
            is_loc = True #判断是否死锁，即没有一个AGV能够运动,都要stay
            agvs_ids_list = [i for i in range(len(self.agvs))]
            #random.shuffle(agvs_ids_list) #AGV运输顺序为随机
            for agv_id in agvs_ids_list:
                if self.agvs_paths[agv_id] == []:
                    #print('路径为空')
                    #agvs没有要行走的路径不动
                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                    self.agvs_paths[agv_id]=[]
                    continue
                    
                elif self.agvs_paths[agv_id][0] == self.agvs_paths[agv_id][-1]: #AGV到达了终点
                    #print('agv到达终点')
                    is_loc = False
                    if self.agvs[agv_id].payload != -1:
                        #卸货
                        
                        '''如果说防止货物时候，货架上有物体，则暂停'''
                        # cargo_id = self.agvs[agv_id].payload
                        # shelf_id = self.cargos[cargo_id].target
                        # if self.shelfs[shelf_id].payload != -1 and self.shelfs[shelf_id].payload != cargo_id:
                        #     self.agvs_paths[agv_id] == []
                        #     self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                        #     continue
                        
                        cargo_id = self.agvs[agv_id].payload
                        self.shelfs[self.cargos[cargo_id].target].payload = cargo_id
                        shelf_id = self.cargos[cargo_id].target
                        self.cargos[cargo_id].load = -1 #货物卸载不在AGV上了
                        self.agvs[agv_id].occupy = -1#agv卸货，没有货物了
                        self.agvs[agv_id].payload = -1#agv当前没有货物运输
                        self.cargos[cargo_id].loc = self.shelfs[self.cargos[cargo_id].target].loc #货物的位置发生变化，被卸货
                        #obtucles不需要改变，因为是set并且没有新的obtacles加入
                        self.agvs_paths[agv_id] = [] #把当前路径清空，已经到达终点
                        
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "DELIVERY"
                        #判断卸货方向
                        direct = ""
                        shelf_loc = self.shelfs[shelf_id].loc
                        agv_loc = self.agvs[agv_id].loc
                        
                            
                        if shelf_loc[0]-agv_loc[0] != 0:
                            if shelf_loc[0]-agv_loc[0] == 1:
                                direct = "DOWN"
                            else:
                                direct = "UP"
                        elif shelf_loc[1] != agv_loc[1]:
                            if shelf_loc[1]-agv_loc[1] == 1:
                                direct = "RIGHT"
                            else:
                                direct = "LEFT"
                        self.ACTIONS_SEQ1[action_idx][agv_id]["dir"] = direct
                        print('agv'+str(agv_id)+'卸货货物'+str(cargo_id)+'到货架'+str(shelf_id))
                        continue
                    
                    if self.agvs[agv_id].occupy != -1:
                        #装货
                        cargo_id = self.agvs[agv_id].occupy
                        cargo_loc = self.cargos[cargo_id].loc
                        
                        is_remove = True
                        for shelf in self.shelfs:
                            if self.shelfs[shelf].loc == self.cargos[cargo_id].loc:
                                is_remove = False
                        if is_remove and tuple(self.cargos[cargo_id].loc) in self.obstacles:
                            self.obstacles.remove(tuple(self.cargos[cargo_id].loc))
                        self.cargos[cargo_id].loc = self.agvs[agv_id].loc #货物装上agv
                        self.agvs[agv_id].payload = cargo_id #agvs开始运输,
                        self.agvs_paths[agv_id] = [] #agv已经到达终点，等待deliver分配新的路线
                        
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "PICKUP"
                        #判断装货方向
                        direct = ""
                        agv_loc = self.agvs[agv_id].loc
                        if cargo_loc[0]-agv_loc[0] != 0:
                            if cargo_loc[0]-agv_loc[0] == 1:
                                direct = "DOWN"
                            else:
                                direct = "UP"
                        elif cargo_loc[1] != agv_loc[1]:
                            if cargo_loc[1]-agv_loc[1] == 1:
                                direct = "RIGHT"
                            else:
                                direct = "LEFT"
                        self.ACTIONS_SEQ1[action_idx][agv_id]["dir"] = direct
                        print('agv'+str(agv_id)+'装货'+str(cargo_id))
                        continue
                    
                    else:
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                    
                else:
                    #print('agv路径还没走完')
                    #agv路径还没走完
                    next_loc = list(self.agvs_paths[agv_id][1])  #()=>[]
                    if tuple(next_loc) in self.obstacles: #[]=>()
                        #agv不动,stay
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                        self.agvs[agv_id].loc = self.agvs[agv_id].loc
                        continue
                    agv_loc = self.agvs[agv_id].loc
                    self.obstacles.remove(tuple(agv_loc)) #更新障碍物
                    self.agvs[agv_id].loc = next_loc #前进一步
                    
                    self.agvs_paths[agv_id] = self.agvs_paths[agv_id][1:] #更新路径
                    self.obstacles.add(tuple(next_loc))
                    is_loc = False #非死锁情况
                    

                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "MOVE"
                    #移动方向
                    direct = ""
                    if next_loc[0]-agv_loc[0] != 0:
                        if next_loc[0]-agv_loc[0] == 1:
                            direct = "DOWN"
                        else:
                            direct = "UP"
                    elif next_loc[1] - agv_loc[1] != 0:
                        if next_loc[1]-agv_loc[1] == 1:
                            direct = "RIGHT"
                        else:
                            direct = "LEFT"
                    self.ACTIONS_SEQ1[action_idx][agv_id]["dir"] = direct
                    
            return is_loc
                
                
        '''AGV运输情况'''
        def onload(self):
            agvs_ids_list = [i for i in range(len(self.agvs))]
            random.shuffle(agvs_ids_list) #AGV运输顺序为随机
            for agv_id in self.agvs:
                real_agv_id = agv_id
                if self.agvs[real_agv_id].payload != -1 or self.agvs[real_agv_id].occupy != -1: #当前AVG在装载或是正在去装载的路上
                    #if self.agvs_paths[agv_id] != []:      
                    continue
                
                start = self.agvs[real_agv_id].loc
                start_idx = self.position_dict[tuple(start)]
                dists, parents = self.Dijkstra(start_idx)
                
                ends = [] #使得本agv不重复寻找相同终点的路径
                paths_agv = []
                diss_agv = []
                cargo_agv_ids = []
                
                '''被初选中的最短路径和要运输的货物id'''
                min_path = []
                cargo_id_select = -1
                
                
                for cargo_id in self.cargos:
                    if self.shelfs[self.cargos[cargo_id].target].payload == cargo_id or self.cargos[cargo_id].load != -1:#货物已经在货架上了或是正在被AVG前往装载
                        cargo_id += 1
                        continue
                    # print(agv_id, start)
                    # print(cargo_id, end)
                    end = self.cargos[cargo_id].loc #本次循环的货物路径
                    #寻找可行的一个货物周围的位置，从而运输 
                    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                        #如果货物周围的店不是障碍物并且
                        if (end[0]+dx, end[1]+dy) in self.position_dict:
                            end_idx = self.position_dict[(end[0]+dx, end[1]+dy)]
                            if end_idx not in ends:
                                ends.append(end_idx)
                                path, dis = self.findpath(dists, parents, start_idx, end_idx)
                                #print(path)
                                
                                paths_agv.append(path)
                                diss_agv.append(dis)
                                cargo_agv_ids.append(cargo_id)
                        
                    min_path, cargo_id_select = self.sortpath(paths_agv, diss_agv, cargo_agv_ids)
                    
                    if min_path != []: #找到了货物周围的一条可行线路
                        print(min_path)
                        break
                    
                
                if path == []:
                    end_x = random.randint(0, self.map_height-1)
                    end_y = random.randint(0, self.map_width-1)
                    while tuple([end_x, end_y]) in self.path_obstacles:
                        end_x = random.randint(0, self.map_height-1)
                        end_y = random.randint(0, self.map_width-1)
                    #print("path为空")
                    path = self.bfs(start, [end_x, end_y])
                    agv_id += 1
                    continue   #无可行路径，不予分配agv和cargo
                
                #正常分配agv给cargo
                self.path_obstacles = self.path_obstacles.union(set(path)) #将该路径的所有经过的点都放入障碍物中
                self.update_graph(set(min_path)) #更新邻接矩阵
                
                self.agvs_paths[agv_id] = min_path
                self.agvs[real_agv_id].occupy = cargo_id_select
                self.cargos[cargo_id_select].load = real_agv_id
                print("AVG"+str(real_agv_id) + "运输" + str(cargo_id) +"货物")
        


        '''AGV'''
        def delivery(self):
            for agv_id in self.agvs:
                if self.agvs[agv_id].payload == -1 or (self.agvs[agv_id].payload != -1 and self.agvs_paths[agv_id] != []): #当前AVG没在运输,或者是当前agv在运输中，但是已经分配了路径
                    continue
                start = self.agvs[agv_id].loc
                cargo_id = self.agvs[agv_id].payload
                end = self.shelfs[self.cargos[cargo_id].target].loc
                
                start_idx = self.position_dict[tuple(start)]
                dists, parents = self.Dijkstra(start_idx)
                
                paths_agv = []
                diss_agv = []
                shelf_agv_ids = []
                
                '''被初选中的最短路径和要运输的货物id'''
                min_path = []
                cargo_id_select = -1
                
                
                #寻找可行的一个货物周围的位置，从而运输 
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]: 
                    if (end[0]+dx, end[1]+dy) in self.position_dict:
                        end_idx = self.position_dict[(end[0]+dx, end[1]+dy)]
                        path, dis = self.findpath(dists, parents, start_idx, end_idx)
                        
                        paths_agv.append(path)
                        diss_agv.append(dis)
                        shelf_agv_ids.append(-1) #这里不需要shelf的id，因为agv只向一个确定的货架进行deliver
                    
                min_path, _ = self.sortpath(paths_agv, diss_agv, shelf_agv_ids)
                
                if min_path == []: #没有可行路径，暂时停止
                    continue
                
                self.path_obstacles = self.path_obstacles.union(set(min_path)) #将该路径的所有经过的点都放入障碍物中
                self.update_graph(set(min_path))
                self.agvs_paths[agv_id] = min_path #保存对应AGV的运输线路    
                
        
        def isDeliver(self): #判断一次送货/卸货是否完毕
            for agv_id in self.agvs_paths:
                if len(self.agvs_paths[agv_id]) != 0:
                    return False
            return True
        

        '''随机运动，不装货不卸货'''
        def random_foward(self):
            action_idx = len(self.ACTIONS_SEQ1)
            self.ACTIONS_SEQ1.append([])
            for i in range(len(self.agvs)): #本轮所有agv的操作字典
                self.ACTIONS_SEQ1[action_idx].append({})
            agvs_ids_list = [i for i in range(len(self.agvs))]
            #random.shuffle(agvs_ids_list) #AGV运输顺序为随机
            for agv_id in agvs_ids_list:
                if self.agvs_paths[agv_id] == []:
                    #print('路径为空')
                    #agvs没有要行走的路径不动
                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                    self.agvs_paths[agv_id]=[]
                    continue
                    
                elif self.agvs_paths[agv_id][0] == self.agvs_paths[agv_id][-1]: #AGV到达了终点
                    #print('agv到达终点')
                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                    self.agvs_paths[agv_id] = [] #路径清空
                        
                    
                else:
                    #print('agv路径还没走完')
                    #agv路径还没走完
                    next_loc = list(self.agvs_paths[agv_id][1])  #()=>[]
                    if tuple(next_loc) in self.obstacles: #[]=>()
                        #agv不动,stay
                        self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "STAY"
                        self.agvs[agv_id].loc = self.agvs[agv_id].loc
                        continue
                    agv_loc = self.agvs[agv_id].loc
                    self.obstacles.remove(tuple(agv_loc)) #更新障碍物
                    self.agvs[agv_id].loc = next_loc #前进一步
                    self.agvs_paths[agv_id] = self.agvs_paths[agv_id][1:] #更新路径
                    self.obstacles.add(tuple(next_loc))
                    is_loc = False #非死锁情况
                    

                    self.ACTIONS_SEQ1[action_idx][agv_id]["type"] = "MOVE"
                    #移动方向
                    direct = ""
                    if next_loc[0]-agv_loc[0] != 0:
                        if next_loc[0]-agv_loc[0] == 1:
                            direct = "DOWN"
                        else:
                            direct = "UP"
                    elif next_loc[1] - agv_loc[1] != 0:
                        if next_loc[1]-agv_loc[1] == 1:
                            direct = "RIGHT"
                        else:
                            direct = "LEFT"
                    self.ACTIONS_SEQ1[action_idx][agv_id]["dir"] = direct
            
        '''给已经装载了货物的重新规划路径'''
        def random_deliver_and_onload(self):
            for agv_id in range(len(self.agvs)):
                if self.agvs[agv_id].payload != -1:
                    start = self.agvs[agv_id].loc
                    end = self.shelfs[self.cargos[self.agvs[agv_id].payload].target].loc

                    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:

                        path = self.bfs(start, [end[0]+dx, end[1]+dy]) 
                        if path != []: #找到了货物周围的一条可行线路
                            end = [end[0]+dx, end[1]+dy]
                            break
                    self.agvs_paths[agv_id] = path #保存对应AGV的运输线路[(1,1), (2,2),...]
                    if path == []:
                        continue   #无可行路径，不予分配agv和cargo
                    #正常分配agv给cargo
                    self.path_obstacles = self.path_obstacles.union(set(path)) #将该路径的所有经过的点都放入障碍物中
                
                if self.agvs[agv_id].payload == -1 and self.agvs[agv_id].occupy != -1:
                    '''消除货物与AGV（前去运输）的关系'''
                    self.cargos[self.agvs[agv_id].occupy].load = -1 
                    self.agvs[agv_id].occupy = -1
                    
        
        def random_move(self):
            self.path_obstacles = set(self.obstacles)
            for i in self.agvs_paths:
                ranx = random.randint(0, self.map_height)
                rany = random.randint(0, self.map_width)
                while (ranx, rany) in self.obstacles:
                    ranx = random.randint(0, self.map_height)
                    rany = random.randint(0, self.map_width)
                self.agvs_paths[i] = self.bfs(self.agvs[i].loc ,[ranx, rany])
                self.path_obstacles = self.path_obstacles.union(self.agvs_paths[i])
            for i in range(10):
                self.random_foward()
            self.paths_init()
            self.path_obstacles = set(self.obstacles)
            self.random_deliver_and_onload()
            

        def process(self):
            self.paths_init()
            step = 1000
            while not self.isComplete() and step>=0:
                step-=1
                
                self.path_obstacles = set(self.obstacles)
                #一次送货卸货
                self.onload() 
                #print(self.agvs_paths)
                for path in self.agvs_paths:
                    print(str(path) + " "+ str(self.agvs_paths[path]))
                while not self.isDeliver():
                    locked = self.forward()
                    #print('运输中')
                    if locked:
                        print("死锁")
                        break
                self.construct_graph() #执行完成一次之后，重构路径，因为path全部执行完了
                        
                self.path_obstacles = set(self.obstacles)
                #print("运输完成")
                self.delivery()
                #print("正在前往装货")
                #print(self.agvs_paths)
                for path in self.agvs_paths:
                    print(str(path) + " "+ str(self.agvs_paths[path]))
                while not self.isDeliver():
                    #print("运货中")
                    locked = self.forward()
                    if locked:
                        print("死锁")
                        break
                self.construct_graph() #执行完成一次之后，重构路径，因为path全部执行完了
                
            if self.isComplete():
                print("全部运输完成")
            else:
                print("没有运输成功") 

                for i in range(len(self.cargos)):
                    if self.shelfs[self.cargos[i].target].payload != self.cargos[i].id:
                        print("货物"+str(self.cargos[i].id) +"没有被运输到货架上")
                
                
                self.random_move() #没有找到路线，死锁则重新随机运动寻找路径
                step=10000
                while not self.isComplete() and step>=0:
                    step-=1
                    lock_onload = False
                    lock_deliver = False
                     
                    self.path_obstacles = set(self.obstacles)
                    #一次送货卸货
                    self.onload()
                    
                    for i in self.agvs_paths:
                        print(self.agvs_paths[i])
                    print("开始新的运送")
                    
                    if self.isDeliver():
                        lock_onload = True
                        
                        
                    while not self.isDeliver():
                        locked = self.forward()
                        print('\n')
                        #print('运输中')
                        if locked:
                            print("死锁")
                            break
                    self.construct_graph() #执行完成一次之后，重构路径，因为path全部执行完了
                            
                    self.path_obstacles = set(self.obstacles)
                    #print("运输完成")
                    self.delivery()
                    #print("正在前往装货")
                    #print(self.agvs_paths)
                    
                    if self.isDeliver():
                        lock_deliver = True
                        
                    while not self.isDeliver():
                        #print("运货中")
                        locked = self.forward()
                        if locked:
                            print("死锁")
                            break
                        
                    self.construct_graph() #执行完成一次之后，重构路径，因为path全部执行完了
                    
                    if lock_onload and lock_deliver:
                        self.random_move() #没有找到路线，死锁则重新随机运动寻找路径
                    
                if self.isComplete():
                    print("全部运输完成")   
                
                
    return sf(map_info)
import numpy as np
import os, csv


class space_time_network:
    def __init__(self, this_train_num, r):
        self.__inf = float('inf')
        self.__train = r.train_list[this_train_num - 1]
        self.__station_num = r.station_num
        self.__train_num = r.train_num
        self.__t_s_time = r.t_s_time
        self.__t_p_time = r.t_p_time
        self.__s_time = r.s_time
        self.__e_time = r.e_time
        self.__time_len = r.time_len
        self.__sum_node = r.sum_node
        self.__departure = r.departure_list[this_train_num - 1]
        self.__stop = r.stop_list[this_train_num - 1]
        self.__max_waiting = r.max_waiting_time_list[this_train_num - 1]
        self.__min_waiting = r.min_waiting_time_list[this_train_num - 1]
        self.__time_interval = r.time_interval
        self.stn = np.ones((r.sum_node + 2, r.sum_node + 2)) * r.inf  # 第一个点为逻辑起点,第二点为逻辑终点
        self.direction = r.direction_list[this_train_num - 1]
        if self.direction == 1:
            self.__start_station = 1
            self.__end_station = r.station_num
        else:
            self.__start_station = r.station_num
            self.__end_station = 1
        # 初始化矩阵
        for i in range(len(self.stn[0])):
            for j in range(len(self.stn[0])):
                if i == j:
                    self.stn[i, j] = 0

    @staticmethod
    def Dijkstra(D, s, t):
        i = s
        r = D.shape[0]
        List = list(range(1, r + 1))
        pred = [s] * r
        d = [float("inf")] * r
        d[i - 1] = 0
        pred[i - 1] = i
        del List[i - 1]
        while len(List) != 0:
            for k in range(1, len(List) + 1):
                j = List[k - 1]
                if d[j - 1] > d[i - 1] + D[i - 1, j - 1]:
                    d[j - 1] = d[i - 1] + D[i - 1, j - 1]
                    pred[j - 1] = i
            d_temp = []
            for l in List:
                d_temp.append(d[l - 1])
            index = d_temp.index(min(d_temp))
            i = List[index]
            del List[index]
        if d[t - 1] != float('inf'):
            Path = [t]
            now_node = t
            while now_node != s:
                pred_node = pred[now_node - 1]
                Path.insert(0, pred_node)
                now_node = pred_node
        else:
            Path = float('inf')
        return Path

    def __return_node_num(self, station, state, time):
        if self.__start_station == self.__station_num:
            if station == self.__station_num:
                state = 1
            if station == 1:
                state = 2
        if station == 1:
            return time - self.__s_time + 1 + 2
        else:
            return self.__time_len + (station - 2) * 3 * self.__time_len + (
                    state - 1) * self.__time_len + time - self.__s_time + 1 + 2

    def __link_station(self, now_station, next_station, running_time, now_state, next_state):
        if now_state < 3:
            running_time += self.__t_s_time
        if next_state < 3:
            running_time += self.__t_p_time
        for now_time in range(self.__s_time, self.__e_time + 1):
            now_node = space_time_network.__return_node_num(self, now_station, now_state, now_time)
            next_time = now_time + running_time
            if next_time > self.__e_time:
                break
            next_node = space_time_network.__return_node_num(self, next_station, next_state, next_time)
            self.stn[now_node - 1, next_node - 1] = running_time

    def __link_instation(self, now_station, max_waiting_time, min_waiting_time):
        if min_waiting_time == 0:
            min_waiting_time = 1
        for now_time in range(self.__s_time, self.__e_time + 1):
            now_node = space_time_network.__return_node_num(self, now_station, 1, now_time)
            for next_time in range(now_time + min_waiting_time, now_time + max_waiting_time + 1):
                if next_time > self.__e_time:
                    break
                next_node = space_time_network.__return_node_num(self, now_station, 2, next_time)
                self.stn[now_node - 1, next_node - 1] = next_time - now_time
        return self.stn

    def generate_network(self):
        # 连接逻辑起点
        earliest_time = self.__departure[0]
        latest_time = self.__departure[1]
        for t in range(earliest_time, latest_time + 1):
            now_node = space_time_network.__return_node_num(self, self.__start_station, 2, t)
            self.stn[0, now_node - 1] = t - earliest_time + 1
        # 连接逻辑终点
        for t in range(self.__s_time, self.__e_time + 1):
            now_node = space_time_network.__return_node_num(self, self.__end_station, 1, t)
            self.stn[now_node - 1, 1] = 0
        # 连接中间点-连接所有站间
        if self.__start_station == 1:
            station_range = range(1, self.__station_num)
            station_range2 = range(2, self.__station_num)
        else:
            station_range = range.__reversed__(range(2, self.__station_num + 1))
            station_range2 = range.__reversed__(range(2, self.__station_num))
        for now_station in station_range:
            if self.__start_station == 1:
                running_time = self.__train[now_station - 1]
                next_station = now_station + 1
            else:
                running_time = self.__train[now_station - 2]
                next_station = now_station - 1
            if now_station == self.__start_station:  # 连接出发站与下一站
                if self.__station_num > 2:
                    for next_state in (1, 3):  # state:1到达,2出发,3通过
                        if next_station in self.__stop:
                            if next_state == 3:
                                continue
                        space_time_network.__link_station(self, now_station, next_station, running_time, 2, next_state)
                else:
                    space_time_network.__link_station(self, now_station, next_station, running_time, 2, 1)
            elif next_station == self.__end_station:  # 连接终点站与前一站
                for now_state in (2, 3):
                    space_time_network.__link_station(self, now_station, next_station, running_time, now_state, 1)
            else:
                for now_state in (2, 3):
                    for next_state in (1, 3):
                        if next_station in self.__stop:
                            if next_state == 3:
                                continue
                        space_time_network.__link_station(self, now_station, next_station, running_time, now_state, next_state)
        # 连接中间点-连接所有站内
        for now_station in station_range2:
            max_waiting_time = self.__max_waiting[now_station - 2]
            min_waiting_time = self.__min_waiting[now_station - 2]
            space_time_network.__link_instation(self, now_station, max_waiting_time, min_waiting_time)

    def __node_num2station_state_time(self, node_num):
        num = (node_num - 2) // self.__time_len
        time = (node_num - 2) % self.__time_len
        if time == 0:
            time = self.__time_len
        if num >= 1:
            station = (num - 1) // 3 + 2
            state = num - 1 - (station - 2) * 3 + 1
        else:
            station = 1;
            state = 2
        if self.__start_station == 1:
            return (station, state, time)
        else:
            if station == self.__start_station:
                state = 2
            if station == self.__end_station:
                state = 1
            return (station, state, time)

    def forbid_node(self, path0, same_next_direction):
        path = path0[:]
        del path[0]
        del path[len(path) - 1]
        forbid_node_list = []
        for i in range(len(path)):
            (station, state, time) = space_time_network.__node_num2station_state_time(self, path[i])
            if i >= 1:
                (last_station, last_state, last_time) = space_time_network.__node_num2station_state_time(self, path[i - 1])
                running_time = time - last_time
            else:
                running_time = None
            forbid_node_list.append(space_time_network.get_forbid_node(self, station, state, time, running_time, same_next_direction))
        return forbid_node_list

    def get_forbid_node(self, station, state, time, running_time, same_next_direction):
        T_bu = self.__time_interval[self.__train_num - 1][0]
        T_hui = self.__time_interval[self.__train_num - 1][1]
        T_lian = self.__time_interval[self.__train_num - 1][2]
        T_daofa = self.__time_interval[self.__train_num - 1][3]
        T_fadao = self.__time_interval[self.__train_num - 1][4]
        T_butong = self.__time_interval[self.__train_num - 1][5]
        forbid_node_list = []
        time = time + self.__s_time - 1
        if self.__start_station == 1:
            last_station = station - 1 if station - 1 >= 1 else 0
        else:
            last_station = station + 1 if station + 1 <= self.__start_station else 0
        if state == 2:
            if same_next_direction:
                if station != self.__start_station:
                    for now_time in range(time, time + T_fadao if time + T_fadao < self.__e_time + 1 else self.__e_time + 1):  # 不同时发到间隔
                        forbid_node_list.append(self.__return_node_num(station, 1, now_time))
                    for now_time in range(time - T_daofa + 1 if time - T_daofa + 1 > self.__s_time else self.__s_time, time + 1):  # 不同时到发
                        forbid_node_list.append(self.__return_node_num(station, 1, now_time))
            else:
                for now_time in range(time - T_hui + 1 if time - T_hui + 1 > self.__s_time else self.__s_time, time + 1):  # 会车时间间隔
                    if station != self.__start_station:
                        forbid_node_list.append(self.__return_node_num(station, 1, now_time))
                        forbid_node_list.append(self.__return_node_num(station, 3, now_time))
                    else:
                        forbid_node_list.append(self.__return_node_num(station, 1, now_time))
        if state == 1:
            if same_next_direction:
                if last_station != 0:
                    for now_time in range(time - running_time, time + T_lian if time + T_lian < self.__e_time + 1 else self.__e_time + 1):  # 连发间隔时间
                        if now_time < self.__s_time:
                            continue
                        forbid_node_list.append(self.__return_node_num(last_station, 2, now_time))
                if station != self.__end_station:
                    for now_time in range(time, time + T_daofa if time + T_daofa < self.__e_time + 1 else self.__e_time + 1):  # 不同时到发
                        forbid_node_list.append(self.__return_node_num(station, 2, now_time))
                    for now_time in range(time - T_fadao + 1 if time - T_fadao + 1 > self.__s_time else self.__s_time, time + 1):  # 不同时发到
                        forbid_node_list.append(self.__return_node_num(station, 2, now_time))
            else:
                for now_time in range(time - T_bu + 1 if time - T_bu + 1 > self.__s_time else self.__s_time,
                                      time + T_bu if time + T_bu < self.__e_time + 1 else self.__e_time + 1):  # 不同时到达
                    if station != self.__end_station:
                        forbid_node_list.append(self.__return_node_num(station, 1, now_time))
                        forbid_node_list.append(self.__return_node_num(station, 3, now_time))
                for now_time in range(time, time + T_hui if time + T_hui < self.__e_time + 1 else self.__e_time + 1):  # 会车间隔
                    if station == self.__end_station:
                        forbid_node_list.append(self.__return_node_num(station, 1, now_time))
                    else:
                        forbid_node_list.append(self.__return_node_num(station, 2, now_time))
        if state == 3:
            if same_next_direction:
                if last_station != 0:
                    for now_time in range(time - running_time if time - running_time > self.__s_time else self.__s_time,
                                          time + T_lian if time + T_lian < self.__e_time+1 else self.__e_time+1):  # 连发间隔时间
                        forbid_node_list.append(self.__return_node_num(last_station, 2, now_time))
            else:
                for now_time in range(time - T_butong - 1 if time - T_butong - 1 > self.__s_time else self.__s_time,
                                      time + T_butong if time + T_butong < self.__e_time +1else self.__e_time+1):  # 不同时通过
                    forbid_node_list.append(self.__return_node_num(station, 3, now_time))
                for now_time in range(time, time + T_hui if time + T_hui < self.__e_time +1else self.__e_time+1):  # 会车间隔
                    forbid_node_list.append(self.__return_node_num(station, 2, now_time))
                for now_time in range(time - T_bu + 1 if time - T_bu + 1 > self.__s_time else self.__s_time, time + 1):  # 不同时到达
                    forbid_node_list.append(self.__return_node_num(station, 1, now_time))

        return forbid_node_list

    def remove_arc(self, path, next_stn, same_next_direction):
        forbid_node_list = self.forbid_node(path, same_next_direction)
        for forbid in forbid_node_list:
            for node in forbid:
                for i in range(len(self.stn[0])):
                    if i == node - 1:
                        continue
                    next_stn[node - 1, i] = self.__inf
        self.forbid_long_arc_cross_short_arc(path, next_stn)

    def forbid_long_arc_cross_short_arc(self, path0, next_stn):
        path = path0[:]
        del path[0]
        del path[len(path) - 1]
        for now_node in range(1, len(path)):
            (now_station, now_state, now_time) = self.__node_num2station_state_time(path[now_node - 1])
            (next_station, next_state, next_time) = self.__node_num2station_state_time(path[now_node])
            # 同向
            if now_node != 1:
                f_state_range = (2, 3)
            else:
                f_state_range = range(2, 3)
            if now_node + 1 != len(path):
                p_state_range = (1, 3)
            else:
                p_state_range = range(1, 2)
            for f_time in range(self.__s_time, self.__s_time + now_time):
                for p_time in range(next_time + self.__s_time - 1, self.__e_time + 1):
                    for ns in f_state_range:
                        f_node = self.__return_node_num(now_station, ns, f_time)
                        for nns in p_state_range:
                            p_node = self.__return_node_num(next_station, nns, p_time)
                            next_stn[f_node - 1, p_node - 1] = self.__inf
            # 反向
            if now_node != 1:
                p_state_range = (1, 3)
            else:
                p_state_range = range(2, 3)
            if now_node + 1 != len(path):
                f_state_range = (2, 3)
            else:
                f_state_range = range(1, 2)
            for f_time in range(self.__s_time, self.__s_time + next_time):
                for p_time in range(now_time + self.__s_time - 1, self.__e_time + 1):
                    for ns in f_state_range:
                        f_node = self.__return_node_num(next_station, ns, f_time)
                        for nns in p_state_range:
                            p_node = self.__return_node_num(now_station, nns, p_time)
                            next_stn[f_node - 1, p_node - 1] = self.__inf

    @staticmethod
    def __get_node_id(station, time):
        time_min = time % 60
        time_hour = time // 60
        if time_hour < 10 and time_min < 10:
            node_id = str(station) + '000' + str(time_hour) + '0' + str(time_min)
        elif time_hour >= 10 and time_min < 10:
            node_id = str(station) + '00' + str(time_hour) + '0' + str(time_min)
        elif time_hour < 10 and time_min >= 10:
            node_id = str(station) + '000' + str(time_hour) + str(time_min)
        else:
            node_id = str(station) + '00' + str(time_hour) + str(time_min)
        return int(node_id)

    @staticmethod
    def __get_time(time):
        hour = time // 60
        min = time % 60
        if hour < 10 and min < 10:
            return '0' + str(hour) + '0' + str(min)
        elif hour >= 10 and min < 10:
            return str(hour) + '0' + str(min)
        elif hour < 10 and min >= 10:
            return '0' + str(hour) + str(min)
        else:
            return str(hour) + str(min)

    @staticmethod
    def __get_zone_id(station, time, zone_list):
        for i in range(len(zone_list)):
            if zone_list[i][0] <= time <= zone_list[i][1] and station == zone_list[i][2]:
                return i + 1

    @staticmethod
    def generate_nexta(station_num, s_time, e_time, zone_list, path_list, path_type):
        str_path = os.path.dirname(os.path.realpath(__file__)) + '\\output_file\\node.csv'
        beginning_or_end = []
        for p in path_list:
            beginning_or_end.append(p[0])
            beginning_or_end.append(p[len(p) - 1])
        with open(str_path, 'w+', newline='') as f:
            w = csv.writer(f)
            header = ['name', 'physical_node_id', 'time', 'node_id', 'zone_id', 'node_type', 'control_type', 'x_coord',
                      'y_coord']
            w.writerow(header)
            for now_station in range(1, station_num + 1):
                for now_time in range(s_time, e_time + 1):
                    line = []
                    now_time_min = now_time % 60
                    now_time_hour = now_time // 60
                    node_id = space_time_network.__get_node_id(now_station, now_time)
                    zone_id = space_time_network.__get_zone_id(now_station, now_time, zone_list)
                    if [now_station, now_time - s_time + 1] in beginning_or_end:
                        line.extend(
                            [None, now_station, str(now_time_hour) + ':' + str(now_time_min), node_id, zone_id, 1,
                             None, (now_time - s_time + 1) * 100, now_station * 1000, None])
                    else:
                        line.extend(
                            [None, now_station, str(now_time_hour) + ':' + str(now_time_min), node_id, zone_id, 0, None,
                             (now_time - s_time + 1) * 100, now_station * 1000, None])
                    w.writerow(line)
        f.close()
        str_path = os.path.dirname(os.path.realpath(__file__)) + '\\output_file\\road_link.csv'
        with open(str_path, 'w+', newline='') as f:
            w = csv.writer(f)
            header = ['name', 'road_link_id', 'from_node_id', 'to_node_id', 'facility_type', 'dir_flag', 'length',
                      'lanes', 'capacity', 'free_speed', 'link_type', 'cost']
            w.writerow(header)
            road_link_id = 1
            for p in path_list:
                for i in range(1, len(p)):
                    line = []
                    now_station = p[i - 1][0]
                    now_time = p[i - 1][1] + s_time - 1
                    from_node_id = space_time_network.__get_node_id(now_station, now_time)
                    next_station = p[i][0]
                    next_time = p[i][1] + s_time - 1
                    to_node_id = space_time_network.__get_node_id(next_station, next_time)
                    cost = next_time - now_time
                    line.extend([None, road_link_id, from_node_id, to_node_id, None, 1, cost, 1, 1, 1, 1, cost])
                    road_link_id += 1
                    w.writerow(line)
        f.close()
        str_path = os.path.dirname(os.path.realpath(__file__)) + '\\output_file\\agent.csv'
        with open(str_path, 'w+', newline='') as f:
            header = ['agent_id', 'o_zone_id', 'd_zone_id', 'o_node_id', 'd_node_id', 'agent_type', 'time_period', 'volume', 'cost', 'travel_time', 'distance', 'node_sequence',
                      'time_sequence']
            w = csv.writer(f)
            w.writerow(header)
            agent_id = 1
            for p in path_list:
                now_train_type = path_type[path_list.index(p)]
                line = []
                o_station = p[0][0]
                d_station = p[len(p) - 1][0]
                o_time = p[0][1] + s_time - 1
                d_time = p[len(p) - 1][1] + s_time - 1
                time_period = space_time_network.__get_time(o_time) + '_' + space_time_network.__get_time(d_time)
                o_zone_id = space_time_network.__get_zone_id(o_station, o_time, zone_list)
                d_zone_id = space_time_network.__get_zone_id(d_station, d_time, zone_list)
                o_node_id = space_time_network.__get_node_id(o_station, o_time)
                d_node_id = space_time_network.__get_node_id(d_station, d_time)
                travel_time = d_time - o_time
                node_sequence = str(o_node_id) + ';'
                time_sequence = space_time_network.__get_time(o_time) + ';'
                for i in range(2, len(p)):
                    now_station = p[i - 1][0]
                    now_time = p[i - 1][1] + s_time - 1
                    now_node = space_time_network.__get_node_id(now_station, now_time)
                    node_sequence += str(now_node) + ';'
                    time_sequence += space_time_network.__get_time(now_time) + ';'
                node_sequence += str(d_node_id)
                time_sequence += space_time_network.__get_time(d_time)
                line.extend(
                    [agent_id, o_zone_id, d_zone_id, o_node_id, d_node_id, now_train_type, time_period, 1, travel_time, travel_time, travel_time, node_sequence, time_sequence])
                w.writerow(line)
                agent_id += 1
        f.close()

    def path2station_time(self, path0):
        path = path0[:]
        del path[0]
        del path[len(path) - 1]
        new = []
        for i in path:
            (station, state, time) = self.__node_num2station_state_time(i)
            new.append([station, time])
        return new


class read:
    @staticmethod
    def __read_direction():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\direction.csv'
        lines = read.__read_file(path)
        direction_list = []
        for l in lines:
            direction_list.append(int(l[1]))
        return direction_list

    @staticmethod
    def __read_file(path):
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            for i in range(len(lines)):
                lines[i] = lines[i].replace('\n', '')
                lines[i] = lines[i].split(',')
        f.close
        return lines

    @staticmethod
    def __str_time2int_time(time):
        time = time.split(':')
        return int(time[0]) * 60 + int(time[1])

    @staticmethod
    def __read_train():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\train running time.csv'
        lines = read.__read_file(path)
        del lines[0]
        trian_running_time_list = []
        for l in lines:
            t = []
            for i in range(2, len(l) + 1):
                t.append(int(l[i - 1]))
            trian_running_time_list.append(t)
        return (trian_running_time_list, len(trian_running_time_list[0]) + 1)

    @staticmethod
    def __read_time():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\time information.csv'
        line = read.__read_file(path)
        del line[0]
        for i in range(len(line[0])):
            if i <= 1:
                line[0][i] = int(line[0][i])
            else:
                line[0][i] = read.__str_time2int_time(line[0][i])
        return line[0]

    @staticmethod
    def __read_max_waiting_time():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\max waiting time.csv'
        lines = read.__read_file(path)
        del lines[0]
        max_waiting_time_list = []
        for l in lines:
            t = []
            for i in range(2, len(l) + 1):
                t.append(int(l[i - 1]))
            max_waiting_time_list.append(t)
        return max_waiting_time_list

    @staticmethod
    def __read_min_waiting_time():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\min waiting time.csv'
        lines = read.__read_file(path)
        del lines[0]
        min_waiting_time_list = []
        for l in lines:
            t = []
            for i in range(2, len(l) + 1):
                t.append(int(l[i - 1]))
            min_waiting_time_list.append(t)
        return min_waiting_time_list

    @staticmethod
    def __read_departure_time():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\departure time range.csv'
        lines = read.__read_file(path)
        del lines[0]
        departure_list = []
        for l in lines:
            t = []
            for i in range(1, 3):
                t.append(read.__str_time2int_time(l[i]))
            departure_list.append(t)
        return departure_list

    @staticmethod
    def __read_time_interval():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\intervals of trains.csv'
        lines = read.__read_file(path)
        del lines[0]
        time_interval = []
        for l in lines:
            t = []
            for i in range(1, 7):
                t.append(int(l[i]))
            time_interval.append(t)
        return time_interval

    @staticmethod
    def __read_sequence_of_stops():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\sequence of stops.csv'
        lines = read.__read_file(path)
        del lines[0]
        stop_sequence = []
        for l in lines:
            t = []
            if l[1] != '':
                l[1] = l[1].split(';')
                for i in l[1]:
                    t.append(int(i))
            else:
                t.append(0)
            stop_sequence.append(t)
        return stop_sequence

    @staticmethod
    def __read_zone():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\zone.csv'
        lines = read.__read_file(path)
        del lines[0]
        zone_list = []
        for l in lines:
            zone_s_time = read.__str_time2int_time(l[1])
            zone_e_time = read.__str_time2int_time(l[2])
            zone_station = int(l[3])
            zone_list.append([zone_s_time, zone_e_time, zone_station])
        return zone_list

    @staticmethod
    def __read_type():
        path = os.path.dirname(os.path.realpath(__file__)) + '\\input_file\\train type.csv'
        lines = read.__read_file(path)
        del lines[0]
        train_type = []
        for l in lines:
            l[1] = l[1].split(';')
            for i in l[1]:
                train_type.insert(int(i) - 1, l[0].replace('\t', ''))
        path = os.path.dirname(os.path.realpath(__file__)) + '\\output_file\\agent_type.csv'
        with open(path, 'w+', newline='') as f:
            w = csv.writer(f)
            header = ['agent_type', 'name']
            w.writerow(header)
            for l in lines:
                w.writerow([l[0], l[0]])
        f.close()
        return train_type

    def __init__(self):
        self.inf = float('inf')
        (self.train_list, self.station_num) = read.__read_train()
        self.train_num = len(self.train_list)
        [self.t_s_time, self.t_p_time, self.s_time, self.e_time] = read.__read_time()
        self.time_len = self.e_time - self.s_time + 1
        self.sum_node = self.time_len * ((self.station_num - 2) * 3 + 2)
        self.departure_list = read.__read_departure_time()
        self.stop_list = read.__read_sequence_of_stops()
        self.max_waiting_time_list = read.__read_max_waiting_time()
        self.time_interval = read.__read_time_interval()
        self.direction_list = read.__read_direction()
        self.min_waiting_time_list = read.__read_min_waiting_time()
        self.zone_list = read.__read_zone()
        self.train_type = read.__read_type()


if __name__ == '__main__':
    print('正在计算。。。')
    r = read()
    path_list = []
    train_num = r.train_num
    train_list = []
    path_type = []
    for now_train in range(1, train_num + 1):
        train_list.append(space_time_network(now_train, r))
        train_list[now_train - 1].generate_network()
    for now_train in range(1, train_num + 1):
        path = space_time_network.Dijkstra(train_list[now_train - 1].stn, 1, 2)
        path_type.append(r.train_type[now_train - 1])
        if now_train <= train_num - 1:
            for next_train in range(now_train + 1, train_num + 1):
                if train_list[next_train - 1].direction == train_list[now_train - 1].direction:
                    same_next_direction = True
                else:
                    same_next_direction = False
                train_list[now_train - 1].remove_arc(path, train_list[next_train - 1].stn, same_next_direction)
        path = train_list[now_train - 1].path2station_time(path)
        path_list.append(path)
    zone_list = r.zone_list
    space_time_network.generate_nexta(r.train_num, r.s_time, r.e_time, zone_list, path_list, path_type)
    print('求解完毕，请打开NEXTA软件查看')

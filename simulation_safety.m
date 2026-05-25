% 场景1 安全水平分析 混合交通流
clc;clear;
dbstop if error;
%------------------基本参数-----------------%
global ca_max cv_max  ch_safe cd_max cd_safe ta_max tv_max td_max th_safe td_safe 
global  car_length road_length simtime truck_length
global k1 k2 ta kp kd tc
global vlimit xlimito xlimitd v_initial ttc_threshold ttc_threshold_t time_step pre_time vd_crit
%------------------道路尺寸-----------------%
car_length = 4;               %汽车车长 
truck_length = 12;            %货车车长
road_length = 7000;           %路长
simtime = 1500;               % 最大仿真步长
time_step = 0.1;                   % 单位仿真步长/s
pre_time = 300;
%----------------车辆参数----------------%
v_initial = 80/3.6;                         %车辆初始速度
ca_max = 1.25;                                  %汽车最大加速度1m/s^2
cv_max = 120/3.6;                            %汽车最大速度33.3m/s
cd_max = 2.09;                                  %汽车舒适减速度
ch_safe = 1.5;                               %汽车安全车头时距
cd_safe = 2;                                 %汽车最小安全距离
ta_max = 0.4;                               %货车最大加速度0.8m/s^2
tv_max = 80/3.6;                            %货车最大速度22.2m/s
td_max = 1.77;                               %货车舒适减速度
th_safe = 1.5;                                %货车安全车头时距
td_safe = 3;                                %货车最小安全距离
k1 = 0.0561;                                   %ACC货车控制参数
k2 = 0.3393;                                   %ACC货车控制参数
ta = 2.0;                                    %ACC货车车头时距
kp = 0.0074;                                 %CACC货车控制参数
kd = 0.0805;                                 %CACC货车控制参数
tc = 1.2;                                  %CACC货车车头时距
vlimit = 10/3.6;                             %限速区车速限制
vd_crit = -3.5;                          %最大减速度
xlimito = 3000;                              %限速区起始位置
xlimitd = 4000;                              %限速区终止位置
ttc_threshold = 1.5;                         %TTC阈值
ttc_threshold_t = 1.5;
%--------------仿真设置参数--------------%
%                                                                                                                                                                                                                   
num_epochs = 1;                % 循环次数
num_totals = 100;% 仿真车辆数 
p_car=0.8;
q_cacc=[0:0.1:0.2];
platoon_length = [2:5];
%p_car = [0,0.2,0.5,0.8];                       % 人驾车辆渗透率
%q_cacc = [0 1 1 0.8
    %0 0.8 0.8 0.6
    %0 0.4 0.5 0.4
    %0 0.1 0.2 0.2];            %智能网联货车占货车比
%platoon_length = [2 2 5 5
    %2 2 4 4
    %2 2 3 3
    %2 2 2 5];                    %编队长度
flow_rate = 1400;
h_ori = 3600/flow_rate*v_initial;                                % 初始车头间距
%--------------收集器--------------%
dimension1 = length(q_cacc);                                   %维度1计算cacc渗透率
dimension2 = length(platoon_length);                           %维度2计算编队长度
v = zeros(simtime/time_step,num_totals+1,dimension1,dimension2,num_epochs);            %车辆速度矩阵，存储每辆车运行的速度
x = v;                                                %车辆位置矩阵，存储每辆车运行的位置
a = v;                                                %车辆加速度矩阵，存储每辆车运行的加速度
%end_time = zeros(1,dimension1);                        %场景终止时间存储
data_x = x;                                           %存储位置分析数据
data_v = v;                                           %存储速度分析数据
data_a = a;                                           %存储加速度分析数据
data_f = zeros(1,num_totals,dimension1,dimension2,num_epochs);
for epoch = 1:num_epochs   
    for p_jth = 1:dimension1
        for q_jth = 1:dimension2
            q_cacc_i = q_cacc(p_jth);
            platoon_length_i = platoon_length(q_jth);
            [car_vector] = type(p_car,q_cacc_i,platoon_length_i,num_totals); %随机生成车辆，具体见子函数；
            [car] = initialize(num_totals,car_vector,h_ori);               %初始化设置，具体见子函数；;
            data_f(1,:,p_jth,q_jth,epoch) = car_vector(1:num_totals);
            for time = time_step:time_step:simtime
                time_i = round(time/time_step);
                [car] = car_following(car,time_step);      %车辆跟驰行为，具体见子函数；
                a(time_i,:,p_jth,q_jth,epoch) = a(time_i,:,p_jth,q_jth,epoch) + car.a(1:end);  %车辆加速度信息存储
                v(time_i,:,p_jth,q_jth,epoch) = v(time_i,:,p_jth,q_jth,epoch) + car.v(1:end);  %车辆速度信息存储
                x(time_i,:,p_jth,q_jth,epoch) = x(time_i,:,p_jth,q_jth,epoch) + car.x(1:end);   %车辆位置信息存储
           end
        end
    end
end
[m,n,h,w,z]=size(x);
for i = 1:m
    for j = 1:n
        for k = 1:h
            for p = 1:w
                for q = 1:z
                    if x(i,j,k,p,q)>=0&&x(i,j,k,p,q)<=road_length
                        data_x(i,j,k,p,q) = x(i,j,k,p,q);             %采集路段数据作为分析数据
                        data_v(i,j,k,p,q) = v(i,j,k,p,q);
                        data_a(i,j,k,p,q) = a(i,j,k,p,q);
                    end
                end                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
            end
        end
    end
end
traj1=trajectory(data_x(:,:,1,1,1),data_v(:,:,1,1,1),[700,2300]);

ttc_save = ttc_calculate(data_x,data_v,data_f); %计算TTC；
tet_save = tet_calculate(ttc_save,data_f);                          %计算TET；
tit_save = tit_calculate(ttc_save,data_f);                          %计算TIT；
sd_save = sd_calculate(data_v);
tet = mean(tet_save,3);                                       %取所有重复实验的均值作为输出
tit = mean(tit_save,3);
sd = mean(sd_save,3);
%% 车辆初始化函数

%%
function [car] = initialize(num_totals,car_vector,h_ori)
global v_initial pre_time
%----------calculate num_cars-----------%
car_martix = zeros(1,num_totals+1);                    %最后一辆为虚拟车辆，模拟限速区入口
car = struct('a',car_martix,'a_pre',car_martix,'v',car_martix,'x',car_martix,'f',car_martix,'d',car_martix);

%----------initialize velocity and acclerate-----------%
car.a(:,:) =  0;
car.v(:,:) =  v_initial;
car.d(:,:) =  0;
car.x = -1*(num_totals-1)*h_ori-v_initial*pre_time:h_ori:h_ori-v_initial*pre_time;
car.f = [car_vector 4];                                %虚拟车辆车型标记为4，不进行计算。
end
%% 随机车辆初始化矩阵
function [car_vector] = type(p_car,q_cacc,platoon_length,num_totals)
num_human_car = p_car*num_totals;
num_human_truck = (1-p_car-q_cacc)*num_totals;
num_truck_platoon = floor(q_cacc*num_totals/platoon_length);
num_types = round(num_human_car+num_human_truck+num_truck_platoon);                                %计算车型总量（整个货车编队算一种车型）
z = [];
cacc_follow = zeros(1,platoon_length-1);
truck_platoon = [cacc_follow 2];
z_num = randperm(num_types);
pos_human_car = z_num(1:num_human_car);
pos_truck_platoon = z_num(num_human_car+1:num_human_car+num_truck_platoon);
pos_human_truck = z_num(num_human_car+num_truck_platoon+1:num_types);
for i = 1:num_types
    if ismember(i,pos_human_car) == 1
        z = [3 z];                                                     %人驾汽车车型为[3]
    elseif ismember(i,pos_truck_platoon) == 1
        z = [truck_platoon z];                                         %货车编队车型为[002]
    elseif ismember(i,pos_human_truck) == 1
        z=[1 z];                                                       %人驾货车车型为[1]
    end
end
vector_length = length(z);
if vector_length >= num_totals
    car_vector = z(vector_length-num_totals+1:vector_length);
elseif vector_length < num_totals
    truck_fill = fliplr(truck_platoon);
    vector_fill = fliplr(truck_fill(1:num_totals-vector_length));
    car_vector = [vector_fill z];
end
end

%% 跟驰函数
function [car] = car_following(car,time_step)
global tv_max ta_max cv_max ca_max xlimito xlimitd vlimit vd_crit num_totals
car_n_last = length(car.x);
sub_martix = zeros(1,num_totals+1);
sub = struct('a',sub_martix,'v',sub_martix,'x',sub_martix);
    sub.x = car.x;
    sub.v = car.v;
    sub.a = car.a;
for car_id = length(car.x):-1:1
    car_n = car_id;
    if car_n == car_n_last
        if car.x(car_n) >= xlimito && car.x(car_n) <= xlimitd
            v_max = vlimit;
        else
            v_max = tv_max;
        end
        car.a(car_n) = max(vd_crit,min(ta_max,(v_max-sub.v(car_n))/time_step));
        car.x(car_n) = sub.v(car_n)*time_step + (car.a(car_n)*time_step^2)/2 + sub.x(car_n);
        car.v(car_n) = sub.v(car_n) + car.a(car_n)*time_step;
    else
        car_n_front = car_n + 1;
        if car.f(car_n) == 3
            [a_n] = max(vd_crit,min(ca_max,cidm_calculate(sub,car_n,car_n_front)));
            v_next = max(0,min([cv_max,a_n*time_step + sub.v(car_n)]));
        elseif car.f(car_n) == 2
            [a_n] = max(vd_crit,min(ta_max,acc_calculate(sub,car_n,car_n_front)));
            v_next = max(0,min([tv_max,a_n*time_step + sub.v(car_n)]));
        elseif car.f(car_n) == 1
            [a_n] = max(vd_crit,min(ta_max,tidm_calculate(sub,car_n,car_n_front)));
            v_next = max(0,min([tv_max,a_n*time_step + sub.v(car_n)]));
        elseif car.f(car_n) == 0
            [a_n] = max(vd_crit,min(ta_max,cacc_calculate(sub,car_n,car_n_front)));
            v_next = max(0,min([tv_max,a_n*time_step + sub.v(car_n)]));
        end
        car.a(car_n) = (v_next - sub.v(car_n))/time_step;
        car.x(car_n) = sub.v(car_n)*time_step + car.a(car_n)*time_step^2/2 + sub.x(car_n);
        car.v(car_n) = v_next;
         %if  car.x(car_n) >= xlimito&&car.x(car_n) <= xlimitd                                  %限速区车速限制
            % car.a(car_n) = vlimit - car.v(car_n);
             %car.v(car_n) = min(vlimit,car.v(car_n));
             %car.x(car_n) = sub.v(car_n)*time_step + car.a(car_n)*time_step^2/2 + sub.x(car_n);
         %end
    end
end
end

%% 车辆加速度计算函数,cIDM模型
function [a_n] = cidm_calculate(car,car_n,car_n_front)
global car_length cd_safe ch_safe ca_max cd_max cv_max
    delta_x_real = car.x(car_n_front) - car.x(car_n) - car_length;
    delta_x = delta_x_real;
    delta_v = car.v(car_n)-car.v(car_n_front);
e_dis = cd_safe+car.v(car_n)*ch_safe+(car.v(car_n)*delta_v)/(2*(ca_max*cd_max)^0.5);
d_car_n = (e_dis/delta_x)^2;
a_n = ca_max*(1-(car.v(car_n)/cv_max)^4-d_car_n);
end

%% 车辆加速度计算函数,tIDM模型
function [a_n] = tidm_calculate(car,car_n,car_n_front)
global td_max th_safe truck_length tv_max ta_max td_safe
    delta_x_real = car.x(car_n_front) - car.x(car_n) - truck_length;
    delta_x = delta_x_real;
    delta_v = car.v(car_n)-car.v(car_n_front);
e_dis = td_safe+car.v(car_n)*th_safe+(car.v(car_n)*delta_v)/(2*(ta_max*td_max)^0.5);
d_car_n = (e_dis/delta_x)^2;
a_n = ta_max*(1-(car.v(car_n)/tv_max)^4-d_car_n);
end

%% 车辆加速度计算函数,ACC模型
function [a_n] = acc_calculate(car,car_n,car_n_front)
global truck_length td_safe ta k1 k2
    delta_x_real = car.x(car_n_front) - car.x(car_n) - truck_length;
    delta_x = delta_x_real;
    delta_v = car.v(car_n_front)-car.v(car_n);
e_dis = delta_x - td_safe - ta*car.v(car_n);
a_n = k1*e_dis+k2*delta_v;
end

%% 车辆加速度计算函数,CACC模型
function [a_n] = cacc_calculate(car,car_n,car_n_front)
global truck_length td_safe kp kd tc
    delta_x = car.x(car_n_front) - car.x(car_n)-truck_length;
    delta_v = car.v(car_n_front)-car.v(car_n);
e_dis = delta_x-td_safe-tc*car.v(car_n);
a_n = (kp*e_dis+kd*delta_v)/(kd*tc+0.01);
end

%% Time to collision
function [ttc_n] = ttc_calculate(data_x,data_v,data_f)
global car_length truck_length
[m,n,h,w,z] = size(data_x);
ttc = zeros(m,n,h,w,z);
for i = 1:m
    for j = 1:n-1
        for k = 1:h
            for p = 1:w
                for q = 1:z
                    if data_v(i,j,k,p,q) > data_v(i,j+1,k,p,q)
                        if data_f(1,j,k,p,q) == 3
                            ttc(i,j,k,p,q) = (data_x(i,j+1,k,p,q)-data_x(i,j,k,p,q)-car_length)/(data_v(i,j,k,p,q)-data_v(i,j+1,k,p,q));
                        else
                            ttc(i,j,k,p,q) = (data_x(i,j+1,k,p,q)-data_x(i,j,k,p,q)-truck_length)/(data_v(i,j,k,p,q)-data_v(i,j+1,k,p,q));
                        end
                    end
                end
            end
        end
    end
end
ttc_n = ttc;
end

%% Time exposed time-to-collision
function [tet_n] = tet_calculate(ttc_n,data_f)
global ttc_threshold time_step ttc_threshold_t
[m,n,h,w,z] = size(ttc_n);
tet = zeros(m,n,h,w,z);
for i = 1:m
    for j = 1:n-1
        for k = 1:h
            for p = 1:w
                for q = 1:z
                    if data_f(1,j,k,p,q) == 3
                        if ttc_n(i,j,k,p,q)>0&&ttc_n(i,j,k,p,q)<=ttc_threshold
                            tet(i,j,k,p,q) = 1;
                        end
                    else
                        if ttc_n(i,j,k,p,q)>0&&ttc_n(i,j,k,p,q)<=ttc_threshold_t
                            tet(i,j,k,p,q) = 1;
                        end
                    end
                end
            end
        end
    end
end
sum_tet = zeros(h,w,z);
for i = 1:h
    for j = 1:w
        for k = 1:z
            tet_part = tet(:,:,i,j,k);
            sum_tet(i,j,k) = sum(tet_part(:));
        end
    end
end
tet_n = sum_tet.*time_step;
end

%% Time integrated time-to-collision(yao,2020)
function [tit_n] = tit_calculate(ttc_n,data_f)                   %采用姚老师2020年论文中的计算公式
global ttc_threshold time_step ttc_threshold_t
[m,n,h,w,z] = size(ttc_n);
tit = zeros(m,n,h,w,z);
for i = 1:m
    for j = 1:n-1
        for k = 1:h
            for p = 1:w
                for q = 1:z
                    if data_f(1,j,k,p,q) == 3
                        if ttc_n(i,j,k,p,q)>0&&ttc_n(i,j,k,p,q)<=ttc_threshold
                            tit(i,j,k,p,q) = ttc_threshold-ttc_n(i,j,k,p,q);
                        end
                    else
                        if ttc_n(i,j,k,p,q)>0&&ttc_n(i,j,k,p,q)<=ttc_threshold_t
                            tit(i,j,k,p,q) = ttc_threshold_t-ttc_n(i,j,k,p,q);
                        end
                    end
                end
            end
        end
    end
end
sum_tit = zeros(h,w,z);
for i = 1:h
    for j = 1:w
        for k = 1:z
            tit_part = tit(:,:,i,j,k);
            sum_tit(i,j,k) = sum(tit_part(:));
        end
    end
end
tit_n = sum_tit.*time_step;
end

%% standard deviation of vehicle speed
function [sd_n] = sd_calculate(data_v)
[m,n,h,w,z] = size(data_v);
sd = zeros(h,w,z);
for i = 1:h
    for j = 1:w
        for k = 1:z
            sd_part = data_v(:,1:n-1,i,j,k);
            sd(i,j,k) = std2(sd_part(sd_part~=0));
        end
    end
end
sd_n = sd;
end
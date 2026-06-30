function obs = make_obstacles_case1()
%MAKE_OBSTACLES_CASE1 构造论文第 VI.C 节第一场景的三个椭圆柱障碍物。

data = [7, -12, 7, 4; ...
        23, -10, 5, 8; ...
        25, -25, 10, 4];
obs = repmat(struct('center',zeros(3,1),'P',zeros(3), ...
    'ac',0,'bc',0,'xc',0,'yc',0), 1, size(data,1));
for i = 1:size(data,1)
    xc = data(i,1); yc = data(i,2); ac = data(i,3); bc = data(i,4);
    obs(i).center = [xc; yc; 0];
    obs(i).P = diag([1/ac^2, 1/bc^2, 0]);
    obs(i).ac = ac; obs(i).bc = bc;
    obs(i).xc = xc; obs(i).yc = yc;
end
end

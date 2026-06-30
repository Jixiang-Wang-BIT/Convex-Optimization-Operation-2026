function lin = linearize_obstacles(r_guess, obstacles)
%LINEARIZE_OBSTACLES 计算每个节点处障碍函数及其梯度。
%   g(r)=1-(r-c)'P(r-c) 为凹函数，其一阶展开是全局上界。
%   输出 g0(i,n) 和 grad(:,n,i)，用于施加 g_lin<=0。

validateattributes(r_guess, {'numeric'}, {'real','finite','nrows',3});
N = size(r_guess,2);
M = numel(obstacles);
lin.g0 = zeros(M,N);
lin.grad = zeros(3,N,M);
for i = 1:M
    for n = 1:N
        d = r_guess(:,n) - obstacles(i).center;
        lin.g0(i,n) = 1 - d'*obstacles(i).P*d;
        lin.grad(:,n,i) = -2*obstacles(i).P*d;
    end
end
end

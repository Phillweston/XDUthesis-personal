function [M,C,Q_bar,R_bar,G,E,H,U_k] = MPC_Zero_Ref(A,B,N,x_k,Q,R,F)
    n = size(A,1);              % A是n*n矩阵，得到n
    p = size(B,2);              % B是n*p矩阵，得到p
    % 初始化M矩阵，M矩阵是(N+1)*n*n的
    % M矩阵上面是一个n*n的I矩阵，这一步先把矩阵下半部分初始化为0
    M = [eye(n); zeros(N*n, n)];
    % 初始化C矩阵，这一步令它有(N+1)*N*P个0
    C = zeros((N+1)*n, N*p);
    tmp = eye(n);               % 定义一个n*n的I矩阵
    for i = 1:N                 % 循环N次
        rows = i*n+(1:n);       % 定义当前行数，从i*n开始，共n行
        C(rows, :) = [tmp*B, C(rows-n, 1:end-p)];   % 将C矩阵填满
        tmp = A*tmp;            % 每一次将tmp左乘一次
        M(rows, :) = tmp;       % 将M矩阵填满
    end
    % 定义Q_bar
    S_q = size(Q, 1);           % 输出Q的维度
    S_r = size(R, 1);           % 输出R的维度
    Q_bar = zeros((N+1)*S_q, (N+1)*S_r);    % 初始化Q_bar为全0矩阵
    for i = 0:N                 % 循环N+1次
        % 将Q写到Q_bar对角线上
        Q_bar(i*S_q+1:(i+1)*S_q, i*S_q+1:(i+1)*S_q) = Q;
    end
    Q_bar(N*S_q+1:(N+1)*S_q, N*S_q+1:(N+1)*S_q) = F;
    % 定义R_bar
    R_bar = zeros(N*S_r, N*S_r);
    for i = 0:N-1
        R_bar(i*S_r+1:(i+1)*S_r, i*S_r+1:(i+1)*S_r) = R;
    end
    % 求解G、E、H
    G = M'*Q_bar*M;
    E = C'*Q_bar*M;
    H = C'*Q_bar*C+R;
    % 最优化
    f = (x_k'*E')';             % 定义f矩阵
    U_k = quadprog(H, f);       % 求解最优化U_k
end
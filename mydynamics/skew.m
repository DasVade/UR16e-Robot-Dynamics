function S = skew(w)
    % 生成叉乘矩阵 [w×]
    S = [0,     -w(3),  w(2);
         w(3),  0,     -w(1);
        -w(2),  w(1),  0];
end

% Draw a sphere with specified center position and radius
function draw_sphere(position, radius, num)
    for i = 1:num
        [s(i).X, s(i).Y, s(i).Z] = sphere;
        s(i).X = s(i).X * radius(i);
        s(i).Y = s(i).Y * radius(i);
        s(i).Z = s(i).Z * radius(i);
        s(i).X = s(i).X + position(i,1);
        s(i).Y = s(i).Y + position(i,2);
        s(i).Z = s(i).Z + position(i,3);
        surf(s(i).X,s(i).Y,s(i).Z);
    end
end
function [norm]=NormFrom3P(p1,p2,p3)
norm = cross(p1 - p2, p1 - p3);
end
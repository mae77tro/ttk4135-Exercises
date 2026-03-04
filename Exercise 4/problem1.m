% My constraints are:
As = [-1 2;
       1 2
       1 -2
       0 0 
       0 0];

bs = [2 6 2 0 0]';

% Start here
w = 3;
x_0 = [2 0]';
G = [2 0;
     0 2];
c = [-2 -5]';

A = As(w, :) %[1 -2]
b = [2];

KKT_m = [G A';
         A 0];

x_star = KKT_m\[-c;
            b]

p = x_star(1:2)- x_0 

% p = [0.2 0.1]'
% So we do

x_1 = x_0 + p
% And minimize again, 

x_star = KKT_m\[-c;
                b]

p = x_star(1:2)- x_1

x_2 = x_1 + p;
%Since p now is 0 we remove our constraint from A

A = [0 0];
b = 0;

KKT_m = [G A';
         A 0];

x_star = KKT_m\[-c;
            b]

p = x_star(1:2)- x_2

% We now get p = [-1.2 2.4]', but this would violate our constraint.



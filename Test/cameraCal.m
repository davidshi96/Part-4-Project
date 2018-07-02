x = [50
55
60
65
70
75
80
85
90
95
100
105
110
115
120
125
130
135
140
145
150
175
200];
x=x*10;
y = [224
195
183
167
154
144
139
133
124
119
114
107
101
97
94
89
86
82
78
75
73
63
55];
% y2 represents the pixel to mm ratio
% so y2 represents how many pixels a mm represents at a certain distance
% away 
y2 = y*(802/1206)/98;
plot (x,y2);
hold on;
p = polyfit(log(x),log(y2),1)
scale = exp(log(x)*p(1) + p(2));
plot (x,scale);
    

% Set up symbolic functions
syms r(t) v(t) v_mag(t) omega(t)

% Set parameters
%w = 2*pi / 10; % change this
%d = 0.2432;
d = 0.10; % Wheelbase size
%d = 0.26;

a = 0.4;
L = 0.4;
b = 1;
c = 0.1;
%c = 0.1;
%scale = 0.2;

% Generate position function
%x = -2*a*((L-cos(c*t))*cos(c*t) + (1-L));
%y = 2*a*(L-cos(c*t))*sin(c*t);

x = 0.3960*cos(2.65*(t*c + 1.4)); 
y = -0.99*sin((t*c + 1.4));

r(t) = [x y 0];

% Find velocity functions
%v(t) = diff(r(t),t)
%v_mag(t) = norm(v(t));
%T_hat = v(t)./v_mag(t)
%T_derivative = diff(T_hat,t);
%N_hat = T_derivative;
%w = cross(T_derivative,N_hat)
%omega(t) = w(3)

v(t) = diff(r(t),t);
v_mag(t) = norm(v(t));
T = v(t)./v_mag(t);
w(t) = cross(T,diff(T,t));
omega = w(t);
omega(t) = omega(3)

n = 1.07*pi/c;
m = 179;%*2;
% Generate set of 't's
t = linspace(1,n,m)';

% Find V_L and V_R
V_L = double(v_mag(t) - omega(t) * (d/2));
V_R = double(v_mag(t) + omega(t) * (d/2));


% Make robot move
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);

for i = 1:length(t)
    V_L(i), V_R(i)
    msg.Data = [V_L(i), V_R(i)];
    send(pub,msg)
    pause(t(2)-t(1))
end

stop = rosmessage(pub);
stop.Data = [0, 0];
send(pub,stop)
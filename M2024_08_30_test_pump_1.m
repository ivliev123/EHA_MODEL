
params.E = 1e9; % Модуль объемного сжатия полости 
params.Cd = 0.6; % Коэффициент расхода
params.Ad = 0.003; % Площадь дросселя
params.Ab = 0.003; % Эффективная площадь дросселя на выходе
params.rho = 850; % Плотность жидкости, кг/м³
params.P_load = 1e5; % Давление нагрузки, Па
params.P_tank = 1e5; % Давление в баке, Па

% Начальные условия []
X0 = [1e5, 1e5];

% Временной интервал решения
tspan = [0 0.001];

% Задаем фиксированный шаг
fixed_step = 0.0001;
options = odeset('MaxStep', fixed_step, 'InitialStep', fixed_step);
% Решение уравнения
% [t, X] = ode45(@(t, X) hydraulic_cylinder(t, X, params), tspan, X0, options);
% [t, X] = ode15s(@(t, X) hydraulic_cylinder(t, X, params), tspan, X0, options);
% [t, X] = ode23t(@(t, X) hydraulic_cylinder(t, X, params), tspan, X0, options);
% [t, X] = ode15s(@(t, X) hydraulic_cylinder(t, X, params), tspan, X0, options);

[t, X] = ode45(@(t, X) hydraulic_pump(t, X, params), tspan, X0, options);
% [t, X] = ode15s(@(t, X) hydraulic_pump(t, X, params), tspan, X0, options);




% QA = zeros(numel(t), 1);
% QB = zeros(numel(t), 1);
for k = 1:numel(t)
   [~, Q_in(k,:), Q_out(k,:)] = hydraulic_pump(t(k), X(k,:), params);
end

% V_A = cumtrapz(t, QA);
% V_B = cumtrapz(t, QB);



% Построение графиков
figure;
subplot(3,2,1);
plot(t, X(:,1));
grid on
grid minor
xlabel('Время, с');
ylabel('Давление P_in, Па');
title('Давление в P_in');

subplot(3,2,2);
plot(t, X(:,2));
grid on
grid minor
xlabel('Время, с');
ylabel('Давление P_out, Па');
title('Давление P_out');


subplot(3,2,3);
plot(t, Q_in(:,1));
grid on
grid minor
xlabel('Время, с');
ylabel('');
title('Расход Q_in');


subplot(3,2,4);
plot(t, Q_out(:,1));
grid on
grid minor
xlabel('Время, с');
ylabel('');
title('Расход Q_out');




function [dXdt, Q_in, Q_out] = hydraulic_pump(t, X, params)
    t
    % Извлечение переменных состояния
    P_in = X(1); 
    P_out = X(2);

    % Параметры системы
    E = params.E;
    Cd = params.Cd;
    Ad = params.Ad;
    Ab = params.Ab;
    rho = params.rho;
    P_load = params.P_load;
%     Ps = params.Ps; % Давление насоса
    P_tank = params.P_tank; % Давление в баке
    
    % Объем трубопроводов на входе и выходе
    V_pipe_in = 0.5 * pi * (0.010/2)^2;  % Объем трубы на входе, м³
    V_pipe_out = 0.5 * pi * (0.010/2)^2; % Объем трубы на выходе, м³

    % Поток на входе, зависит от разности давлений между баком и входом
    Q_in = Cd * Ad * sqrt(2 * abs(P_tank - P_in) / rho) * sign(P_tank - P_in);
    
    % Поток, создаваемый насосом
    Q_pump = 1.6e-6 * 10;  % Поток насоса, м³/с (например, 1.6 см³/об * 20 об/с)
    
    % Поток на выходе, зависит от разности давлений между выходом насоса и полостью A
    Q_out = Cd * Ab * sqrt(2 * abs(P_out - P_load) / rho) * sign(P_out - P_load);
    
    % Дифференциальные уравнения для изменения давления на входе и выходе
    dP_in  = (E / V_pipe_in) * (Q_in - Q_pump);
    dP_out = (E / V_pipe_out) * (Q_pump - Q_out);
    
    % Возвращаем производные
    dXdt = [dP_in; dP_out];
end
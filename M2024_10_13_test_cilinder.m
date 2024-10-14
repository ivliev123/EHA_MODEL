% Параметры цилиндра
D_p = 0.050; % Диаметр поршня, м
D_s = 0.025; % Диаметр штока, м

% Вычисление площадей
params.AA = pi/4 * D_p^2; % Площадь полости A
params.AB = pi/4 * (D_p^2 - D_s^2); % Площадь полости B

% Параметры системы
params.L_piston = 0.25;         % [м]   Рабочий ход цилиндра 

params.E = 1e9; % Модуль объемного сжатия полости 
params.VA0 = 1E-6;                          % начальный объем камеры А
params.VB0 = params.L_piston * params.AB + 1E-6;  % начальный объем камеры В
params.M = 100; % Масса подвижных частей
params.B = 100; % Коэффициент вязкого трения
params.F_load = 1000; % Внешняя нагрузка
params.Cd = 0.6; % Коэффициент расхода
params.Ad = 0.001; % Площадь дросселя
params.Ab = 0.001; % Эффективная площадь выхода
params.rho = 850; % Плотность жидкости, кг/м³
params.Ps = 1e6; % Давление насоса, Па
params.P_tank = 1e5; % Давление в баке, Па

% Начальные условия [PA, PB, v, x, 1e5]
X0 = [1e5, 1e5, 0, 0, 1e5];

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
[t, X] = ode45(@(t, X) hydraulic_cylinder(t, X, params), tspan, X0);




% QA = zeros(numel(t), 1);
% QB = zeros(numel(t), 1);
for k = 1:numel(t)
   [~, QA(k,:), QB(k,:)] = hydraulic_cylinder(t(k), X(k,:), params);
end

% V_A = cumtrapz(t, QA);
% V_B = cumtrapz(t, QB);



% Построение графиков
figure;
subplot(3,2,1);
plot(t, X(:,1));
xlabel('Время, с');
ylabel('Давление PA, Па');
title('Давление в полости A');

subplot(3,2,2);
plot(t, X(:,2));
xlabel('Время, с');
ylabel('Давление PB, Па');
title('Давление в полости B');

subplot(3,2,3);
plot(t, X(:,3));
xlabel('Время, с');
ylabel('Скорость поршня, м/с');
title('Скорость поршня');

subplot(3,2,4);
plot(t, X(:,4));
xlabel('Время, с');
ylabel('Положение поршня, м');
title('Положение поршня');


subplot(3,2,5);
% plot(t, V_A, '-r');
% hold on
% plot(t, QA(:,1), '--g');

subplot(3,2,6);
% plot(t, V_B, '-r');
% hold on
% plot(t, QB(:,1), '--g');

function [dXdt, QA, QB] = hydraulic_cylinder(t, X, params)
    t
    % Извлечение переменных состояния
    PA = X(1); % Давление в полости A
    PB = X(2); % Давление в полости B
    v = X(3);  % Скорость поршня
    x = X(4);  % Положение поршня
    Ps = X(5);

    % Параметры системы
    E = params.E;
    AA = params.AA; % Площадь полости A
    AB = params.AB; % Площадь полости B
    VA = params.VA0 + AA * x; % Объем полости A
    VB = params.VB0 - AB * x; % Объем полости B
    M = params.M;
    B = params.B;
    F_load = params.F_load;
%     Ps = params.Ps; % Давление насоса
    P_tank = params.P_tank; % Давление в баке
    
    V_pipe = 1 * pi * (0.01/2)^2;
    
    % Поток через дроссель в полость A
    QA = params.Cd * params.Ad * sqrt(2 * abs(Ps - PA) / params.rho) * sign(Ps - PA);
%     QA = 1.6e-6;
    Q_pump = 1.6e-6 * 20;
    
    
    % Поток из полости B
    QB = params.Cd * params.Ab * sqrt(2 * abs(PB - P_tank) / params.rho) * sign(PB - P_tank);

    % Дифференциальные уравнения
    dPA_dt = (E / VA) * (QA - AA * v);
    dPB_dt = (E / VB) * (-QB + AB * v);
    dv_dt = (AA * PA - AB * PB - F_load - B * v) / M;
    dx_dt = v;
    dPs_dt = (E/V_pipe) * (Q_pump - QA);
%     dPs_dt = 0;

    % Возвращаем производные
    dXdt = [dPA_dt; dPB_dt; dv_dt; dx_dt; dPs_dt];
end
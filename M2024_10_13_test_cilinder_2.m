% Параметры цилиндра
D_p = 0.050; % Диаметр поршня, м
D_s = 0.025; % Диаметр штока, м

% Вычисление площадей
params.AA = pi/4 * D_p^2; % Площадь полости A
params.AB = pi/4 * (D_p^2 - D_s^2); % Площадь полости B

% Параметры системы
params.L_piston = 0.25;         % [м]   Рабочий ход цилиндра
params.E = 1e9; % Модуль объемного сжатия жидкости, Па
params.VA0 = 1E-6;                          % начальный объем камеры А, м³
params.VB0 = params.L_piston * params.AB + 1E-6;  % начальный объем камеры В, м³
params.M = 100; % Масса подвижных частей, кг
params.B = 100; % Коэффициент вязкого трения, Н·с/м
params.F_load = 1000; % Внешняя нагрузка, Н
params.Cd = 0.6; % Коэффициент расхода
params.Ad = 0.001; % Площадь дросселя на входе в полость A, м²
params.Ab = 0.001; % Эффективная площадь выхода из полости B, м²
params.rho = 850; % Плотность жидкости, кг/м³
params.Ps = 1e6; % Давление насоса, Па
params.P_tank = 1e5; % Давление в баке, Па
params.Q_pump = 1.6e-5; % Постоянный поток от насоса, м³/с
params.V_pipe = 1 * pi * (0.01/2)^2; % Объем в трубопроводе

% Начальные условия [PA, PB, v, x, Ps]
X0 = [1e5, 1e5, 0, 0, 1e5];

% Временной интервал решения
tspan = [0 0.01];

% Решение дифференциальных уравнений
[t, X] = ode45(@(t, X) hydraulic_cylinder(t, X, params), tspan, X0);

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
plot(t, X(:,5));
xlabel('Время, с');
ylabel('Давление Ps, Па');
title('Давление насоса');

function [dXdt] = hydraulic_cylinder(~, X, params)
    % Извлечение переменных состояния
    PA = X(1); % Давление в полости A
    PB = X(2); % Давление в полости B
    v = X(3);  % Скорость поршня
    x = X(4);  % Положение поршня
    Ps = X(5); % Давление насоса

    % Параметры системы
    E = params.E;   % Модуль объемного сжатия жидкости
    AA = params.AA; % Площадь полости A
    AB = params.AB; % Площадь полости B
    VA = params.VA0 + AA * x; % Объем полости A
    VB = params.VB0 - AB * x; % Объем полости B
    M = params.M;    % Масса поршня
    B = params.B;    % Коэффициент вязкого трения
    F_load = params.F_load; % Внешняя нагрузка
    P_tank = params.P_tank; % Давление в баке
    V_pipe = params.V_pipe; % Объем трубопровода

    % Поток через дроссель в полость A
    QA = params.Cd * params.Ad * sqrt(2 * abs(Ps - PA) / params.rho) * sign(Ps - PA);
    
    % Поток из полости B
    QB = params.Cd * params.Ab * sqrt(2 * abs(PB - P_tank) / params.rho) * sign(PB - P_tank);

    % Дифференциальные уравнения
    dPA_dt = (E / VA) * (QA - AA * v);
    dPB_dt = (E / VB) * (-QB + AB * v);
    dv_dt = (AA * PA - AB * PB - F_load - B * v) / M;
    dx_dt = v;
    dPs_dt = (E / V_pipe) * (params.Q_pump - QA);

    % Возвращаем производные
    dXdt = [dPA_dt; dPB_dt; dv_dt; dx_dt; dPs_dt];
end
close all, clear all, clc
% Notice that this is the low order version of sensor curve fitting, 

aa=xlsread('sensor_test_values_for_read.xlsx');
inch2cm=2.54;

for i=1:length(aa(1,:))
    
    [a,b,c]=postProc(aa(:,i));
    avr(i)=a;
    stdd(i)=c;
end

voltage=avr;
dis=[1	1.2	1.3	1.4	1.5	1.6	1.7	1.8	1.9	2	2.1	2.2	2.25	2.3	2.4	2.5	2.6	2.7	2.8	2.9	3	3.1	3.2	3.3	3.4	3.5	3.6	3.7	3.8	3.9	4	4.1	4.2	4.3	4.4	4.5	4.6	4.7	4.8	4.9	5	5.1	5.2	5.3	5.4	5.6	5.8	6	6.2	6.4	6.6	6.8	7	7.2	7.4	7.6	7.8	8	8.5	9	9.5	10	10.5	11	11.5	12	13	14	16	18	20	22	24	26	28	30	33	36	39	42];
dis=dis*inch2cm;
plot(dis,voltage,'linewidth',2)
hold on
grid on

% This will give the polyfit result, from stable part, after 10 cm
p=polyfit(dis(31:end),voltage(31:end),20);


x1=linspace(1,80);
y1=polyval(p,x1);
plot(x1,y1,'--','linewidth',2)

xlabel('Distance(cm)')
ylabel('SensorValue')
title('Sensor Output')
legend('Original','20th-Order Polyfitted')

axis([0 100 0 1000])

figure()
%
p_order=4;
ip=polyfit(voltage(29:58),dis(29:58),p_order);
x2=linspace(100,600);
y2=polyval(ip,x2);
plot(voltage,dis)
hold on
plot(x2,y2,'--')
ylabel('Distance(cm)')
xlabel('SensorValue')
title('Sensor Output(inverse polyfit)')
legend('Original',[num2str(p_order),'th-Order Polyfitted'])
grid on
axis([0 1000 0 100])

x=500
y1=1.18270050154854e-09*x^4+(-2.25504856400793e-06*x^3)+0.00165150940050058*x^2+(-0.574424912519396*x)+92.8958757034745
y2=polyval(ip,x)
% figure()
% plot(dis,(stdd./avr)*100)
% title('Standard Deviation')
% xlabel('Distance(cm)')
% ylabel('Std/Avr %')
% grid on

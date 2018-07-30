close all, clear all, clc

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
%High perfromance order:11, 13, 15, 18, 
p_order=15;
ip=polyfit(voltage(25:end),dis(25:end),p_order);
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
y1=-121348.695728197+6395.09589115474*x-150.387079166702*x^2+2.09410051688887*x^3-0.0192880929486053*x^4+0.000124220838323152*x^5-5.75822589460873*10^(-7)*x^6+1.94376654540714*10^(-9)*x^7-4.75911163496137*10^(-12)*x^8+8.24207632536036*10^(-15)*x^9-9.42021682416840*10^(-18)*x^10+5.60137295554236*10^(-21)*x^11+1.07984452527315*10^(-24)*x^12-4.76403623087764*10^(-27)*x^13+3.44305202883701*10^(-30)*x^14-8.99799019219652*10^(-34)*x^15
y2=polyval(ip,x)
% figure()
% plot(dis,(stdd./avr)*100)
% title('Standard Deviation')
% xlabel('Distance(cm)')
% ylabel('Std/Avr %')
% grid on

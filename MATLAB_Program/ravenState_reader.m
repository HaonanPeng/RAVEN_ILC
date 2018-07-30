close all, clear all, clc

M=textread('ravenJoint_pose_recorder2.txt');

for i=[2:4 6:9]
    figure(i)
    
    plot(M(:,1),M(:,i),'linewidth',2)
    
    if i<5
        title_i=i-1;
    else
        title_i=i-2;
    end
    title(['joint ',num2str(title_i)])
    hold on
    
    plot(M(:,1),M(:,i+8),'r--','linewidth',2)
    
    xlabel('time')
    ylabel('jointValue')
    legend('actual joint pose','desired joint pose')
end

for i=18:20
    figure(i)
    
    plot(M(:,1),M(:,i),'linewidth',2)
    hold on
    
    plot(M(:,1),M(:,i+3),'r--','linewidth',2)
    
    xlabel('time')
    ylabel('posValue')
    legend('actual tip pose','desired tip pose')
    
    if i==18
        title('x position')
    elseif i==19
        title('y position')
    elseif i==20
        title('z position')
    end
    
end

figure()
plot(M(:,19),M(:,20),'linewidth',2)
title('traj')
hold on
    
plot(M(:,22),M(:,23),'r--','linewidth',2)
    axis square
xlabel('y')
ylabel('z')
    legend('actual traj','desired traj')


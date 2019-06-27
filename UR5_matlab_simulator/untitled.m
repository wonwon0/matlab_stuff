clear
close all

addpath('/home/phil/catkin_ws/src/matlab_stuff/rrr version phil')
l = [1 0 1;0 1 1; 1 0 1]./[sqrt(2),sqrt(2),sqrt(3)];
for i=1:3
    vectarrow([0,0,0],l(:,i),[0,1,0])
    hold on
end
a = gram_schmidth(l(:,1:2));
b = null(l(:,1:2)');

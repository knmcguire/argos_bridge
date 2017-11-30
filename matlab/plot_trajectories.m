close all, clear all, clc

bug_trajectories=importdata('bug_comparison.csv')

figure(1),
img = imread('rand_env_test.png')
img = imresize(img,4)
imshow(img)
hold on
for i = 1:2:13
    plot(20*(bug_trajectories(:,i+1)+10),20*(bug_trajectories(:,i)+10))
    hold on,
end

plot(360,360,'o')

legend('wall following','com bug', 'I bug', 'bug 2', 'alg 2', 'alg 1', 'blind bug')
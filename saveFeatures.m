function saveFeatures(imgfilename, n)

if(nargin<2)
    n=43;
end

img = imread(imgfilename);
figure; imshow(img);
hold on;

xy = [];
for i = 1:n
	disp(['Vous dessinez présentement le point #' num2str(i)]);
	[x,y] = ginput(1);
	plot(x,y, 'or', 'LineWidth',3);
	text(x + 10,y, num2str(i), 'FontSize',20, 'BackgroundColor', 'w', ...
        'Color', 'k'); % Affichera le numéro a côté du point
	xy = [xy; x y];
end

[~,asciifilename] = fileparts(imgfilename);
save([asciifilename '.txt'], 'xy', '-ascii');
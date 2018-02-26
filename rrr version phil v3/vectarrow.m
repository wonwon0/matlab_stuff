function [h1, h2]=vectarrow(p0,p1,col)
%Arrowline 3-D vector plot.
%   vectarrow(p0,p1) displays 3D arrow from p0 to p1
%
%   Example:
%       3D vector
%       p0 = [1 2 3];   % Coordinate of the first point p0
%       p1 = [4 5 6];   % Coordinate of the second point p1
%       col='b';        % arrow color = blue
%       vectarrow(p0,p1,col)
%
%       2D vector
%       p0 = [1 2];     % Coordinate of the first point p0
%       p1 = [4 5];     % Coordinate of the second point p1
%       col='b';        % arrow color = red
%       vectarrow(p0,p1,col)
%

%   Rentian Xiong 4-18-05
%   $Revision: 1.0
%
%   Modified 
%   Philippe Lebel 2016-07-21
%   $Revision: 1.1
%   added variablle line color
%   returns headers to allow more control on the object
%   added 3D arrow support


  if max(size(p0))==3
      if max(size(p1))==3
          x0 = p0(1);
          y0 = p0(2);
          z0 = p0(3);
          x1 = p1(1);
          y1 = p1(2);
          z1 = p1(3);
          h1=plot3([x0;x1],[y0;y1],[z0;z1],'Linewidth',6,'color',col);   % Draw a line between p0 and p1
          
          p = p1-p0;
          alpha = 0.5;  % Size of arrow head relative to the length of the vector
          beta = 0.5;  % Width of the base of the arrow head relative to the length
          
          hu = [x1-alpha*(p(1)+beta*(p(2)+eps)); x1; x1-alpha*(p(1)-beta*(p(2)+eps))];
          hv = [y1-alpha*(p(2)-beta*(p(1)+eps)); y1; y1-alpha*(p(2)+beta*(p(1)+eps))];
          hw = [z1-alpha*p(3);z1;z1-alpha*p(3)];
          
          hold on
          h2=plot3(hu(:),hv(:),hw(:),'Linewidth',6,'color',col);  % Plot arrow head
          grid on
          xlabel('x')
          ylabel('y')
          zlabel('z')
          hold off
      else
          error('p0 and p1 must have the same dimension')
      end
  elseif max(size(p0))==2
      if max(size(p1))==2
          x0 = p0(1);
          y0 = p0(2);
          x1 = p1(1);
          y1 = p1(2);
          h1=plot([x0;x1],[y0;y1]);   % Draw a line between p0 and p1
          
          p = p1-p0;
          alpha = 0.1;  % Size of arrow head relative to the length of the vector
          beta = 0.1;  % Width of the base of the arrow head relative to the length
          
          hu = [x1-alpha*(p(1)+beta*(p(2)+eps)); x1; x1-alpha*(p(1)-beta*(p(2)+eps))];
          hv = [y1-alpha*(p(2)-beta*(p(1)+eps)); y1; y1-alpha*(p(2)+beta*(p(1)+eps))];
          
          hold on
          h2=plot(hu(:),hv(:));  % Plot arrow head
          grid on
          xlabel('x')
          ylabel('y')
          hold off
      else
          error('p0 and p1 must have the same dimension')
      end
  else
      error('this function only accepts 2D or 3D vector')
  end
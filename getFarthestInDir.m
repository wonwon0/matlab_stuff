function point = getFarthestInDir(shape, v)
%Find the furthest point in a given direction for a shape
XData = get(shape,'XData'); % Making it more compatible with previous MATLAB releases.
YData = get(shape,'YData');
ZData = get(shape,'ZData');
dotted = XData*v(1) + YData*v(2) + ZData*v(3);
[maxInCol,rowIdxSet] = max(dotted);
[maxInRow,colIdx] = max(maxInCol);
rowIdx = rowIdxSet(colIdx);
point = [XData(rowIdx,colIdx), YData(rowIdx,colIdx), ZData(rowIdx,colIdx)]';
end


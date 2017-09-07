clear
close all
figure
while true
    tic
    [h1]=vectarrow_v2([0 0 1],[1 1 2],'b');
    toc
    try delete(h1)
    end
end
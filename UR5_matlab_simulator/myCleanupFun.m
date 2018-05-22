function myCleanupFun(cmdout)
    system(['kill ' cmdout]);
    close all
    disp('rosnode killed')
end


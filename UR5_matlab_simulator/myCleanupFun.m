function myCleanupFun(cmdout)
    system(['kill ' cmdout]);
    write_param_file(0, 0, 0, 1)
    close all
    disp('rosnode killed')
end


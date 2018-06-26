function myCleanupFun(cmdout)
    system(['kill ' cmdout]);
    param_file = fopen('param_script_sim.m','w');
    fprintf(param_file,'auto_launch = 0;\n');
    fprintf(param_file,'link_2_gazebo = 0;\n');
    fclose(param_file);
    close all
    disp('rosnode killed')
end


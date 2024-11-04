% storeData.m
function [time_plot, con_plot, v_plot, actual_plot, desired_plot] = storeData(time_plot, con_plot, v_plot, actual_plot, desired_plot, vp, vr, s, sd)
    % Store the simulation data
    time_plot = [time_plot, toc];
    con_plot = [con_plot, vp];
    v_plot = [v_plot, vr];
    actual_plot = [actual_plot, s(:)];
    desired_plot = [desired_plot, sd(:)];
end

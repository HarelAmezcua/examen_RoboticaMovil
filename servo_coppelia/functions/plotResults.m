% plotResults.m
function plotResults(time_plot, actual_plot, desired_plot, con_plot, v_plot)
    % Plot Desired Pose vs Actual Pose
    figure;
    sgtitle('Comparación de posicion de puntos de interés');
    colors = ['b', 'g', 'm', 'c'];

    for i = 1:4
        subplot(4, 1, i)
        plot(time_plot, actual_plot(2 * i - 1, :), 'Color', colors(i), 'LineWidth', 2);
        hold on;
        plot(time_plot, actual_plot(2 * i, :), 'Color', colors(i), 'LineStyle', '--', 'LineWidth', 2);
        plot(time_plot, desired_plot(2 * i - 1, :), 'r--', 'LineWidth', 1.5);
        plot(time_plot, desired_plot(2 * i, :), 'r--', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel(['Pose ', num2str(i)]);
        legend(['Actual x_', num2str(i)], ['Actual y_', num2str(i)], ...
               ['Desired x_', num2str(i)], ['Desired y_', num2str(i)]);
        grid on;
    end
    saveas(gcf, 'Improved_Desired_vs_Actual_Pose.jpg');

    % Plot Camera Velocities
    figure;
    sgtitle('Velocidades de cámara (Control)');
    subplot(3, 1, 1);
    plot(time_plot, con_plot(1, :), 'b-', 'LineWidth', 2);
    ylabel('Vc_x (m/s)');
    legend('Vc_x');
    grid on;
    subplot(3, 1, 2);
    plot(time_plot, con_plot(2, :), 'g-', 'LineWidth', 2);
    ylabel('Vc_y (m/s)');
    legend('Vc_y');
    grid on;
    subplot(3, 1, 3);
    plot(time_plot, con_plot(6, :), 'm-', 'LineWidth', 2);
    ylabel('Vc_\theta (rad/s)');
    xlabel('Time (s)');
    legend('Vc_\theta');
    grid on;
    saveas(gcf, 'Improved_Camera_Velocities.jpg');

    % Plot Wheel Speeds
    figure;
    sgtitle('Velocidades de las ruedas');
    wheel_colors = ['b', 'g', 'r', 'c'];
    for i = 1:4
        subplot(4, 1, i);
        plot(time_plot, v_plot(i, :), 'Color', wheel_colors(i), 'LineWidth', 2);
        ylabel(['Vel (m/s)']);
        legend(['Rueda ', num2str(i)]);
        grid on;
    end
    xlabel('Time (s)');
    saveas(gcf, 'Improved_Wheel_Speeds.jpg');
end

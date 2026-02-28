function animateSelfBalancing(t, x, theta, Rwheel, bodyHeight)

    %  VIDEO SETUP 
    videoName = 'self_balancing_robot.mp4';
    v = VideoWriter(videoName, 'MPEG-4');
    v.FrameRate = 20;   % frames por segundo
    open(v);

    %  FIGURE SETUP 
    figure;
    axis equal
    grid on
    hold on
    
    xlabel('X [m]')
    ylabel('Z [m]')
    title('Self-Balancing Robot Animation')
    
    xlim([-0.5 0.5])
    ylim([0 0.45])
    
    % Ground
    plot([-1 1],[0 0],'k','LineWidth',2);
    
    % Geometry
    bodyWidth = 0.10;
    
    % Wheel shape
    th = linspace(0,2*pi,60);
    wheelX = Rwheel*cos(th);
    wheelZ = Rwheel*sin(th);
    
    % Body rectangle
    rect = [-bodyWidth/2  bodyWidth/2  bodyWidth/2 -bodyWidth/2;
             0             0            bodyHeight bodyHeight];
    
    % Initial state
    x0 = x(1);
    theta0 = theta(1);
    
    % Wheel
    hWheel = fill(x0 + wheelX, ...
                  Rwheel + wheelZ, ...
                  [0.4 0.4 0.4], ...
                  'EdgeColor','k');
    
    % Body
    Rot0 = [ cos(theta0) -sin(theta0);
             sin(theta0)  cos(theta0)];
    
    rectRot = Rot0 * rect;
    
    hBody = fill(rectRot(1,:) + x0, ...
                 rectRot(2,:) + 2*Rwheel, ...
                 [0.6 0.3 0.8], ...
                 'EdgeColor','k');

    drawnow
    writeVideo(v, getframe(gcf)); % primer frame

    % === ANIMATION LOOP ===
    for k = 2:length(t)
    
        xk = x(k);
        thetak = theta(k);
    
        % Wheel update
        set(hWheel, ...
            'XData', xk + wheelX, ...
            'YData', Rwheel + wheelZ);
    
        % Body update
        Rot = [ cos(thetak) -sin(thetak);
                sin(thetak)  cos(thetak)];
    
        rectRot = Rot * rect;
    
        set(hBody, ...
            'XData', rectRot(1,:) + xk, ...
            'YData', rectRot(2,:) + 2*Rwheel);
    
        drawnow
        
        % === CAPTURE FRAME ===
        writeVideo(v, getframe(gcf));
    end

    % === CLOSE VIDEO ===
    close(v);
    disp(['Video guardado como ', videoName])

end
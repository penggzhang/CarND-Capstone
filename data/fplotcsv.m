function fplotcsv(csvfile)
    data = csvread(csvfile);
    pos.x= data(:,1);
    pos.y= data(:,2);
    pos.z= data(:,3);
    yaw  = data(:,4);

    disp(['Total Waypoints: ' num2str(length(pos.x))]);

    figure;
    plot(pos.y,pos.x,'b.-','MarkerFaceColor','b');
    hold on; plot(pos.y(1),pos.x(1),'g^','MarkerFaceColor','g');
    hold on; plot(pos.y(end),pos.x(end),'r^','MarkerFaceColor','r');
    grid minor;
    ylabel('\bfPosition X');
    xlabel('\bfPosition Y');
    legend('Wpts','Start','Finish');

    figure;
    plot(yaw,'o-','MarkerFaceColor','b');
    grid minor;
    ylabel('\bfYaw(deg)');
    title('\bfYaw Angle for each Waypoint');

    cum_dist = 0;
    dist = [];
    j=1;
    for k=2:length(pos.x)
        delx = pos.x(k)-pos.x(k-1);
        dely = pos.y(k)-pos.y(k-1);
        dist(j) = sqrt(delx*delx + dely*dely);
        cum_dist = cum_dist + dist(j);
        j=j+1;
    end

    figure;
    plot(dist,'.','MarkerFaceColor','b');
    ylim([0 3]);
    grid minor;
    ylabel('\bfDistance');
    xlabel('\bfWaypoint');
    title('\bfDistance Traveled Between Waypoints');

    disp(['Distance Traveled Over Entire Waypoint Path: ' num2str(cum_dist)]);
end
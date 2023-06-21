function quadAnim(t,q,p)
figure("WindowState","maximized");

%% Create first plot %%
[x,y,z] = q2AnimLines(q(1,:)',p.pB,p.pC,p.pD,p.pE,p.propDia);
qBodyPlot = plot3(x(:,1:end-4),y(:,1:end-4),z(:,1:end-4),'b',x(:,end-3:end),y(:,end-3:end),z(:,end-3:end),'k','LineWidth',1.5);
set(gca,"projection","perspective");
grid on
axis equal;
axis([x(1)-5 x(1)+5 y(1)-5 y(1)+5 z(1)-5 z(1)+5])
xlabel("X");
ylabel("Y");
zlabel("Z");

for i = 1:length(t)
    [x,y,z] = q2AnimLines(q(i,:)',p.pB,p.pC,p.pD,p.pE,p.propDia);
    for j = 1:size(x,2)
        qBodyPlot(j).XData = x(:,j);
        qBodyPlot(j).YData = y(:,j);
        qBodyPlot(j).ZData = z(:,j);
    end
    axis([x(1)-5 x(1)+5 y(1)-5 y(1)+5 z(1)-5 z(1)+5]);
    drawnow;
end
end
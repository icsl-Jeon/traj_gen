function draw_box(p1,p2,color,alpha)

 x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0])*(p2(1)-p1(1))+p1(1);
 y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1])*(p2(2)-p1(2))+p1(2);
 z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1])*(p2(3)-p1(3))+p1(3);
    
for i=1:6    
    h=patch(x(:,i),y(:,i),z(:,i),color,'FaceAlpha',alpha);
    set(h,'edgecolor','k')
end

end
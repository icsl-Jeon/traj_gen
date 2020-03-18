function [fixPin, loosePin] = createPinSet(taskAxis)
%  [fixPin, loosePin] = createPinSet(taskAxis)
% taskAxis = [xl xu yl yu]

set(gcf, 'Name', 'traj_gen 2D selection');
fprintf('  t - zero order fix pin selection \n');
fprintf('  y - zero order loose pin selection \n');
fprintf('  e - erase the latest pin \n');
fprintf('  q - leave editing mode\n');
axis(taskAxis)
fixPin = []; loosePin = [];
pinTypeStack = []; % 0:fix / 1: loose

while 1 
    drawnow
     k = waitforbuttonpress;     
     if k ==1 % if pressed 
        c = get(gcf, 'CurrentCharacter');
        switch c
            case 'y'                  
                fprintf('slowly drag rectangle. <w> when done\n')
                title('slowly drag rectangle. <w> when done')                
                while 1
                    k = waitforbuttonpress;
                    if k ~= 1
                        axis(taskAxis)                
                        point1 = get(gca,'CurrentPoint');    % button down detected
                        finalRect = rbbox;                   % return figure units
                        axis(taskAxis)                
                        point2 = get(gca,'CurrentPoint');    % button up detected
                        point1 = point1(1,1:2);              % extract x and y
                        point2 = point2(1,1:2);
                        p1 = min(point1,point2);             % calculate locations
                        offset = abs(point1-point2);         % and dimensions
                        p2 = p1 + offset;
                        x = [p1(1) p1(1)+offset(1) p1(1)+offset(1) p1(1) p1(1)];
                        y = [p1(2) p1(2) p1(2)+offset(2) p1(2)+offset(2) p1(2)];
                        hold on
                        plot(x,y,'k-')
                        drawnow                      
                        
                        t = input('Enter the knot for this loose pin: ');
                        while  ~isnumeric(t)
                            disp('It was not a number.');                        
                            t = input('Enter the knots for this points: ');
                        end                        
                                                
                        pin =  struct('t',t,'d',0,'X',[p1' p2']);                        
                        text(p2(1)+0.2,p2(2)+0.1,num2str(t));
                        loosePin = [loosePin pin];
                        pinTypeStack = [pinTypeStack 1];                        
                        
                    else
                        title('')                        
                        break
                    end
                end
          case 't'
            fprintf('click a sequence of fixPin, <w> when done\n')
            title('click a sequence of fixPin, <w> when done')
            
            while 1
                k1 = waitforbuttonpress;                     
                if k1 == 0 % mouse 
                    point = get(gca,'CurrentPoint'); 
                    hold on
                    h = plot(point(1,1), point(1,2), 'ko');
                    drawnow
                    t = input('Enter the knot for this fix pin: ');
                    while  ~isnumeric(t)
                        disp('It was not a number.');                        
                        t = input('Enter the knots for this points: ');
                    end
                    pin =  struct('t',t,'d',0,'X',[point(1,1) point(1,2)]');
                    text(point(1,1)+0.2,point(1,2)+0.1,num2str(t));                    
                    fixPin = [fixPin pin];
                    pinTypeStack = [pinTypeStack 0];
                end
                if k1 ==1 % button                      
                    c1 = get(gcf, 'CurrentCharacter');
                    if c1 == 'w'
                        title('')
                        break                    
                    end
                end
            end
                      
          case 'e'
              if ~pinTypeStack(end)
                 fixPin(end) = [];                         
              else
                 loosePin(end) = [];       
              end
             pinTypeStack(end) = [];              
            ax = gca;
            delete(ax.Children(1:2))
            fprintf('erased target_path\n');            
            otherwise
             break;      % key pressed
        end
           
     end
end

classdef (Abstract) TrajGen < handle
        properties            
            dim; % dimension of curve (1,2,..4?)             
            pinSet; % the set of pin 
            isSolved; % boolean              
            Ts; % time knots (length = M for polyTrajGen and length = 2 for optimalTrajGen)             
            
            weight_mask; % objective penalization weight

            % PolyTrajGen ~= OptimalTrajGen
            fixPinSet; 
            loosePinSet;             
            fixPinOrder; % fixPinOrder{m} = [set of orders of imposed pin]            
            
        end            
        methods                         
            function obj = TrajGen(knots,dim)
                obj.dim = dim;
                obj.Ts = knots;
            end
            setDerivativeObj(obj,weight)
            function addPin(obj,pin)
                assert (size(pin.X,1) == obj.dim,'dim of pin val != dim of this TrajGen\n');                            
                assert (~(pin.t < obj.Ts(1) || pin.t > obj.Ts(end)) ,'t of this pin is out of range of knots\n');                
                obj.pinSet = [obj.pinSet pin];
            end
            solve(obj)
            eval(obj,t,d)                                    
            %% Common methods 
           function addPinSet(obj,pinSet)
           % addPinSet(obj,pinSet)
           % pinSet = [pin] where  pin = struct('t',,'d',,'X',)
           % Add multiple pin constraints 
                for pin = pinSet
                    obj.addPin(pin);
                end           
           end        
           function showPath(obj,fig_h)
                addpath([pwd '/sub_utils']) % for drawing cube
                if nargin ==  2  
                    figure(fig_h)
                else
                    figure
                end               
               hold on
                assert(obj.dim >= 2 && obj.dim < 4, 'The dimension is not support for path plot\n' );
                % 1. Draw pin first (only zero order pin) 
                for pin = obj.pinSet % per pin
                    if pin.d == 0                    
                        X = pin.X; 
                        if size(X,2) == 1  % FixPin
                            if obj.dim == 2 % 2D
                                plot(X(1),X(2),'ko','MarkerFaceColor',[0.2 0.2 0.2]);                                
                                xlabel('$x$','Interpreter','latex') ;ylabel('$y$','Interpreter','latex'); 
                            else   % 3D          
                                plot3(X(1),X(2),X(3),'ko','MarkerFaceColor',[0.2 0.2 0.2]);     
                                xlabel('$x$','Interpreter','latex') ;ylabel('$y$','Interpreter','latex'); zlabel('z','Interpreter','latex');                                
                            end
                        else % LoosePin
                            if obj.dim == 2 % 2D
                                rectangle('Position',[X(1,1) X(2,1) X(1,2)-X(1,1) X(2,2)-X(2,1)],'FaceColor',[0.8 .8 .8],'EdgeColor','k','LineWidth',1)         
                                xlabel('$x$','Interpreter','latex') ;ylabel('$y$','Interpreter','latex');                                 
                            else % 3D
                                draw_box(X(:,1),X(:,2),[0.8 .8 .8],0.3)
                                xlabel('$x$','Interpreter','latex') ;ylabel('$y$','Interpreter','latex'); zlabel('z','Interpreter','latex');                                
                            end
                        end
                    end                
                end
                
                % 2. Draw curve 
                if obj.isSolved 
                        Nplot = 100;
                        ts = linspace(obj.Ts(1),obj.Ts(end),Nplot);
                        Xs = obj.eval(ts,0);
                        if obj.dim == 3
                            plot3(Xs(1,:),Xs(2,:),Xs(3,:),'k-')                           
                        else
                            plot(Xs(1,:),Xs(2,:),'k-')                            
                        end
                        grid on
                end   
           end
           
            function showTraj(obj,plotOrder,fig_h)
                % showTraf(obj,plotOrder,fig_h)  
                % Draw the current pin and solution in state vs t domain.  
                if nargin ==  3  
                    figure(fig_h)
                else
                    figure
                end

                assert(plotOrder>=0,'Invalid plot order');
                
                % Upper titles with state derivatives 
                strs = {'$x$','$\dot{x}$','$\ddot{x}$','$x^{(3)}$','$x^{(4)}$'};
                for i = 0:plotOrder 
                    subplot(obj.dim,plotOrder+1,i+1)                
                    title(strs{i+1},'Interpreter','latex');
                    set(gca,'FontSize',15)
                end
                
                % Draw pin and trajectory 
                for dd = 1:obj.dim          
                    % 1. Draw pin first 
                    for pin = obj.pinSet % per pin
                        t = pin.t; d = pin.d; X = pin.X; 
                        
                        subplot(obj.dim,plotOrder+1,(dd-1)*(plotOrder+1)+d+1) % locating position
                            hold on
                            % Draw a dashed lines to imposed pin time 
                            hVert = xline(t,'k:');
                            hVert.Color(4) = 0.4;                                                    
                            if size(X,2) == 2   % LoosePin 
                                errorbar(t,(X(dd,1)+X(dd,2))/2,(X(dd,2)-X(dd,1))/2,'k-','LineWidth',3); 
                            else % FixPin
                                plot(t,X(dd),'ko','MarkerFaceColor',[0.2 0.2 0.2]);
                            end
                        set(gca,'XLim',[obj.Ts(1)-0.5  obj.Ts(end)+0.5])               
                        set(gca,'FontSize',15)                                  
                        xlabel('$t$','Interpreter','latex')
                    end                    
                end
                                        
                % 2. Draw curve 
                if obj.isSolved 
                    for d = 0:plotOrder 
                        Nplot = 50;
                        ts = linspace(obj.Ts(1),obj.Ts(end),Nplot);
                        Xs = obj.eval(ts,d);
                        for dd = 1:obj.dim
                            subplot(obj.dim,plotOrder+1,(dd-1)*(plotOrder+1)+d+1)
                                if d > 0
                                    vVert = yline(0,'r--');
                                end                            
                                hold on
                                for t = obj.Ts
                                    % Draw a dashed lines to imposed pin time 
                                    hVert = xline(t,'k-.');
                                    hVert.Color(4) = 0.4;                                   
                                end
                                plot(ts,Xs(dd,:),'k-')
                            set(gca,'XLim',[obj.Ts(1)-0.5  obj.Ts(end)+0.5])                           
                            set(gca,'YLim',[min(Xs(dd,:))-0.5  max(Xs(dd,:))+0.5])        
                            set(gca,'FontSize',15)                            
                            xlabel('$t$','Interpreter','latex')
                        end
                    end
                end            
                                
                isLegend = false;                
                if ~isLegend
                    subplot(obj.dim,plotOrder+1,1)
                    hold on
                    h1 = errorbar(NaN,NaN,'k-','LineWidth',3);                      
                    h2 = plot(NaN,NaN,'ko','MarkerFaceColor',[0.2 0.2 0.2]);                            
                    legend([h2 h1],{'FixPin','LoosePin'},'Location','northwest')
                    isLegend = true;
                end
                                               
            end
           
            
            
            
        end        
end


classdef OptimTrajGen < TrajGen   
    properties 
        pntDensity; % how many point per time         
        nVar; % total number of points
        dt; % time interval btw points
        ts; % time steps of the points
        Xs; % points (dim x nVar)
    end
    
    methods
        function obj = OptimTrajGen(knots,pntDensity,dim)
            % obj = OptimTrajGen(knots,pntDensity,dim)
            % knots = [t0 tf]
            obj = obj@TrajGen(knots,dim);                 
            assert(length(knots) == 2, 'For optimalTrajGen, knots = [t0 tf]');
            obj.nVar = floor((knots(end) - knots(1))*pntDensity);
            obj.pntDensity = pntDensity;
            obj.fixPinSet=cell(1);
            obj.loosePinSet = cell(1);
            obj.dt = (knots(end) - knots(1))/(obj.nVar-1);
            obj.ts = linspace(knots(1),knots(end),obj.nVar);
            obj.Xs = zeros(obj.dim,obj.nVar);
        end
        
        function setDerivativeObj(obj,weight_mask)
            % setDerivativeObj(obj,weight_mask)
            % Set the derivative objective functions. weight_mask ith element = penalty weight for intgeral of i th-derivative squared           
            obj.weight_mask = weight_mask;
        end
        
        function addPin(obj,pin)
            if pin.d >= obj.nVar
                warning('The degree of the pin exceed the total number of variables. This pin ignored\n');
                return
            end            
            addPin@TrajGen(obj,pin); % call the common function of addPin                
            X = pin.X; % the t is global time 
            m = 1; % in optimalTrajGen, segment is single 
            if size(X,2) == 2 % inequality (loose pin)
                obj.loosePinSet{m} = [obj.loosePinSet{m} pin];
            elseif size(X,2) == 1    % equality (fixed pin)
                obj.fixPinSet{m} = [obj.fixPinSet{m} pin];
            else
                warning('Dim of pin value is invalid\;');
            end                                             
        end
        
        function solve(obj)
            obj.isSolved = true;
            % Prepare QP             
            [QSet,ASet,BSet,AeqSet,BeqSet] = obj.getQPSet;            
            for dd = 1:obj.dim
                fprintf('solving %d th dimension..\n', dd)
                [xsol,~,flag] = quadprog(QSet{dd},[],ASet{dd},BSet{dd},AeqSet{dd},BeqSet{dd});
                if flag == 1                    
                    fprintf('success! \n');    obj.Xs(dd,:) = reshape(xsol,1,[]);                                
                else
                    obj.isSolved = false; fprintf('Failure..\n')
                end
            end        
        end
        
        function val = eval(obj,t,d)
            % val = eval(obj,t,d) 
            % Evaluate d th order derivative of the piecewise polynomial at time t or at time sequence t. Extrapolation is turned on. 
 
            val = zeros(obj.dim,length(t)); 
            for dd = 1:obj.dim
                for idx = 1:length(t)
                    if t(idx) < obj.ts(1) || t(idx)  > obj.ts(end-d)
                        warning('Eval of t: out of bound. Extrapolation\n');
                    end                                                   
                    Xsd = obj.getDiffMat(d)*obj.Xs(dd,:)';
                    val(dd,idx) = interp1(obj.ts(1:end-d),Xsd,t(idx),'linear','extrap');                    
                end
            end
        end                   
    end        

    methods (Access = private) % to be private 
        
        function m = findStepIndex(obj,t)          
            % m= findStepIndex(obj,t) 
            % returns the step index closest to the time t                                     
            [~,m]=min((obj.ts-t).^2);                                        
        end          
        

        
        function mat = getDiffMat(obj,d)
            if d == 0
                mat = eye(obj.nVar);
            else
                mat = eye(obj.nVar);                
                for j = 1:d 
                    D = zeros(obj.nVar-(j),obj.nVar-(j-1));
                    for i = 1:obj.nVar-j                                    
                        D(i,i:i+1) = [-1 1];
                    end
                    D = D / obj.dt;
                    mat = D*mat;
                end
            end
        end
        
        function [ASet,BSet] = loosePin2InequalityMat(obj)
            ASet = cell(obj.dim,1); BSet = cell(obj.dim,1);
            for pin = obj.loosePinSet{1}
                for dd = 1:obj.dim
                    n = min(obj.findStepIndex(pin.t),obj.nVar - pin.d); % we lose the last d time-steps during dth derivatives 
                    a = zeros(2,obj.nVar - pin.d); a(:,n) =  [1 ; -1]; a = a*obj.getDiffMat(pin.d);
                    b = [pin.X(dd,2)  -pin.X(dd,1)]; 
                    ASet{dd} = [ASet{dd} ; a];  BSet{dd} = [BSet{dd} ; b];
                end
            end
        end
        
        function [AeqSet,BeqSet] = fixPin2equalityMat(obj)
            AeqSet = cell(obj.dim,1); BeqSet = cell(obj.dim,1);
            for pin = obj.fixPinSet{1}
                for dd = 1:obj.dim
                    n = min(obj.findStepIndex(pin.t),obj.nVar - pin.d); % we lose the last d time-steps during dth derivatives 
                    aeq = zeros(1,obj.nVar - pin.d); aeq(:,n) =  1; aeq = aeq*(obj.getDiffMat(pin.d));
                    beq = pin.X(dd); 
                    AeqSet{dd} = [AeqSet{dd} ; aeq];  BeqSet{dd} = [BeqSet{dd} ; beq];
                end
            end
        end        
                
        function [QSet,ASet,BSet,AeqSet,BeqSet] = getQPSet(obj)
            % 1. Objective             
            for dd = 1:obj.dim
            Q = zeros(obj.nVar);
            for d = 1:length(obj.weight_mask)
                Qd = obj.getDiffMat(d)'*obj.getDiffMat(d);
                Q = Q + obj.weight_mask(d)*Qd;             
            end
            QSet{dd}  = Q;
            end            
            
            % 2. Constraints
            [ASet,BSet] = obj.loosePin2InequalityMat;
            [AeqSet,BeqSet] = obj.fixPin2equalityMat;
        end
    end        
        
end


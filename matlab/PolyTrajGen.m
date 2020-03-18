classdef PolyTrajGen < TrajGen 
    properties (SetAccess = private)
        % Basic params 
        algorithm;  % poly-coeff /end-derivative
        
        % Polynomial 
        polyCoeffSet; % set of P = [p1 p2 ... pM] where pm = (N+1) x 1 vector 
        N; % polynomial order        
        M; % number of segment of polynomial     
        maxContiOrder; % maximum degree of continuity when joining two segments 
        
        % Optimization 
        nVar; % total number of variables         

        segState; % segState(:,m) = [Nf ; Nc]
    end

    %% 1. Public method
    methods % public methods 
        function obj = PolyTrajGen(knots,order,algo,dim,maxContiOrder)
            %  PolyTrajGen(knots,order,algo,dim)
            % Intialize the functor to generate trajectory  
            obj = obj@TrajGen(knots,dim);            
            obj.N = order; 
            
            if strcmp(algo,'poly-coeff')
                fprintf('Optimization will be performed on the coefficients.\n'); obj.algorithm = algo; 
            elseif strcmp(algo,'end-derivative')
                fprintf('Optimization will be performed on end derivatives. This will improve optimization performance.\n'); obj.algorithm = algo;    
            else
                error('algorithm is invalid. enter either : poly-coeff or end-derivative\n');
                return;
            end
            
            obj.M = length(knots)-1; 
            obj.maxContiOrder = maxContiOrder;      
            obj.nVar = (obj.N+1) * (obj.M);
            
            % Optimization 
            obj.isSolved = false;             
            obj.fixPinSet = cell(obj.M,1);
            obj.loosePinSet = cell(obj.M,1);
            
            % State
            obj.segState = zeros(2,obj.M);
            obj.fixPinOrder = cell(obj.M,1);            
        end
        
        function setDerivativeObj(obj,weight_mask)
            % setDerivativeObj(obj,weight_mask)
            % Set the derivative objective functions. weight_mask ith element = penalty weight for intgeral of i th-derivative squared           
            if length(weight_mask) > obj.N
                warning('Order of derivative objective > order of poly. Higher terms will be ignored. ');
                weight_mask = weight_mask(1:obj.N);               
            end            
            obj.weight_mask = weight_mask;
        end
        
        function addPin(obj,pin)            
            % addPin(obj,pin)    
            % Impose a constraint to the dth derivative at time t to X as
            % an equality (fix pin) or inequality (loose pin). In case of
            % fix pin, imposing time should be knots of this polynomial.
            % pin = struct('t',,'d',,'X',)
            % X = Xval or X = [Xl Xu] where Xl or Xu is D x 1 vector
            t = pin.t; X = pin.X; % the t is global time 
            addPin@TrajGen(obj,pin); % call the common function of addPin 
            [m,~] = obj.findSegInteval(t);             
            if size(X,2) == 2 % inequality (loose pin)
                obj.loosePinSet{m} = [obj.loosePinSet{m} pin];
            elseif size(X,2) == 1    % equality (fixed pin)
                assert (t == obj.Ts(m) || t == obj.Ts(m+1),'Fix pin should be imposed only knots\n');
                if obj.segState(1,m) <= obj.N+1 
                    obj.fixPinSet{m} = [obj.fixPinSet{m} pin];
                    obj.segState(1,m) = obj.segState(1,m) + 1;
                    obj.fixPinOrder{m} = [obj.fixPinOrder{m} pin.d];                                           
                else
                    warning('FixPin exceed the dof of this segment. Pin ignored\n');
                end
            else
                warning('Dim of pin value is invalid\;');
            end                
        end
                   
        function solve(obj)           
            obj.isSolved = true;
            % Prepare QP 
            [QSet,ASet,BSet,AeqSet,BeqSet] = obj.getQPSet;            
            if strcmp(obj.algorithm,'end-derivative')
                mapMat = obj.coeff2endDerivatives(AeqSet{1}); 
                [QSet,HSet,ASet,BSet] = obj.mapQP(QSet,ASet,BSet,AeqSet,BeqSet);
            end
            for dd = 1:obj.dim % per element                 
                % Then, solve the optimization 
                fprintf('solving %d th dimension..\n', dd)
                if strcmp(obj.algorithm,'poly-coeff')
                    [Phat,~,flag] = quadprog(QSet{dd},[],ASet{dd},BSet{dd},AeqSet{dd},BeqSet{dd});
                else
                    [dP,~,flag] = quadprog(QSet{dd},HSet{dd},ASet{dd},BSet{dd}); dF = BeqSet{dd};             
                    if flag == 1
                        Phat = mapMat\[dF;dP];                
                    end
                end
                obj.isSolved = obj.isSolved && (flag == 1); 
                if (flag == 1)
                    fprintf('success! \n')                                                        
                    P = obj.scaleMatBigInv*Phat; 
                    obj.polyCoeffSet{dd} = reshape(P,obj.N+1,[]);
                else
                    fprintf('Failure..\n');                    
                end                
            end                 
            fprintf('Done!\n');
        end
        
        function val = eval(obj,t,d)
            % val = eval(obj,t,d) 
            % Evaluate d th order derivative of the piecewise polynomial at time t or at time sequence t. Extrapolation is turned on.             


            val = zeros(obj.dim,length(t)); 
            for dd = 1:obj.dim
                for idx = 1:length(t)
                    ti = t(idx);
                    if ti < obj.Ts(1) || ti > obj.Ts(end)
                        warning('Eval of t: out of bound. Extrapolation\n');
                    end                                        
                    [m,~]=obj.findSegInteval(ti);                    
                    dTm = obj.Ts(m+1) - obj.Ts(m); 
                    val(dd,idx) = obj.tVec((ti - obj.Ts(m)),d)'*obj.polyCoeffSet{dd}(:,m); 
                end
            end
        end        
    end % public method
    %% 2. Priavte method 
    methods (Access = private) % to be private 
        function val = B(obj,n,d)
            % Returns the nth order ceoffs (n=0...N) of time vector of dth
            % derivative.
            if d == 0
                val = 1;
            else
                accumProd = cumprod(n:-1:n-(d-1));
                val = (n>=d) * accumProd(end);
            end
        end
        function vec = tVec(obj,t,d)
            % time vector evaluated at time t with d th order derivative.
            vec = zeros(obj.N+1,1);
            for i = d+1:obj.N+1
                vec(i) = obj.B(i-1,d)*t^(i-1-d);
            end
        end
        
        function mat = scaleMat(obj,delT)
            mat = zeros(obj.N+1);
            for i = 1:obj.N+1
                mat(i,i) = delT^(i-1);
            end
        end
        
        function mat = scaleMatBig(obj)
            % scaling matrix with all knots. Used to remap phat to p 
            matSet = {};
            for m = 1:obj.M
                matSet{m} = obj.scaleMat(obj.Ts(m+1)-obj.Ts(m));
            end
            mat = blkdiag(matSet{:});
        end
        
        function mat = scaleMatBigInv(obj)
            % scaling matrix with all knots. Used to remap phat to p 
            matSet = {};
            for m = 1:obj.M
                matSet{m} = obj.scaleMat(1/(obj.Ts(m+1)-obj.Ts(m)));
            end
            mat = blkdiag(matSet{:});
        end
        
        function mat = IntDerSquard(obj,d)
            % integral (0 to 1) of squard d th derivative 
            if d > obj.N
                warning('Order of derivative > poly order \n')                
            end
            mat = zeros(obj.N+1);
            for i = 1:obj.N+1
                for j = 1:obj.N+1
                    if (i+j-2*d -1) > 0 
                        mat(i,j) = obj.B(i-1,d) *obj.B(j-1,d) / (i+j-2*d-1);                
                    end
                end
            end            
        end
        
        function [m,tau] = findSegInteval(obj,t)          
            % [m,tau] = findSegInteval(obj,t) 
            % returns the segment index + nomalized time, which contains time t              
            m = max(find(t >= obj.Ts));            
            if isempty(m)
                warning('Eval of t : leq T0. eval target = 1st segment')
                m = 1;
            elseif m >= obj.M+1
                if t ~= obj.Ts(end)
                    warning('Eval of t : geq TM. eval target = last segment')                
                end
                m = obj.M;
            else
                
            end                                    
            tau = (t - obj.Ts(m)) / (obj.Ts(m+1) - obj.Ts(m));                            
        end       

        function [aeqSet,beqSet]=fixPinMatSet(obj,pin)
            aeqSet = cell(obj.dim,1); beqSet = cell(obj.dim,1);
             t = pin.t; X = pin.X; d = pin.d;
            [m,tau] = obj.findSegInteval(t);                    
            idxStart = (m-1)*(obj.N+1) + 1; idxEnd = m*(obj.N+1);
            dTm = obj.Ts(m+1) - obj.Ts(m);
            for dd = 1:obj.dim
                aeq = zeros(1,obj.nVar); 
                aeq(:,idxStart:idxEnd)  = obj.tVec(tau,d)'/dTm^d; 
                aeqSet{dd} = aeq;
                beq = X(dd);        
                beqSet{dd} = beq;
            end                   
        end
        
        function [aSet,bSet] = loosePinMatSet(obj,pin)
            aSet = cell(obj.dim,1); bSet = cell(obj.dim,1);            
             t = pin.t; X = pin.X; d=pin.d;
            [m,tau] = obj.findSegInteval(t);                    
            idxStart = (m-1)*(obj.N+1) + 1; idxEnd = m*(obj.N+1);
            dTm = obj.Ts(m+1) - obj.Ts(m);            
            for dd = 1:obj.dim
                a = zeros(2,obj.nVar); b = zeros(2,1);                    
                a(:,idxStart:idxEnd)  = [obj.tVec(tau,d)'/dTm^d; -obj.tVec(tau,d)'/dTm^d;]; b(:) = [X(dd,2)  -X(dd,1)];
                aSet{dd} = a; bSet{dd} = b;
            end
        end
        
        function [aeq, beq] = contiMat(obj,m,dmax)        
                idxStart = (m-1)*(obj.N+1) + 1; idxEnd = (m+1)*(obj.N+1);                
                dTm1 = obj.Ts(m+1) - obj.Ts(m);
                dTm2 = obj.Ts(m+2) - obj.Ts(m+1);                    
                aeq = zeros(dmax+1,obj.nVar); beq = zeros(dmax+1,1);
                for d = 0:dmax
                    aeq(d+1,idxStart : idxEnd) = [obj.tVec(1,d)'/dTm1^(d) -obj.tVec(0,d)'/dTm2^(d)]; 
                end
        end
        
        function [QSet,ASet,BSet,AeqSet,BeqSet] = getQPSet(obj)
             QSet = cell(obj.dim,1); ASet = cell(obj.dim,1); BSet = cell(obj.dim,1); AeqSet = cell(obj.dim,1); BeqSet = cell(obj.dim,1);
             
             % 1. Objective             
             for dd = 1:obj.dim
                Q = zeros(obj.nVar);
                for d = 1:length(obj.weight_mask)
                    for m = 1:obj.M
                        dT = obj.Ts(m+1) - obj.Ts(m);  
                        Qm{m} = obj.IntDerSquard(d)/dT^(2*d-1);
                    end
                    Qd = blkdiag(Qm{:}); Q = Q + obj.weight_mask(d)*Qd;               
                end
                QSet{dd}  = Q;
            end
            
            % 2. Constraint
            for m = 1:obj.M
                % Fix pin 
                for pin = obj.fixPinSet{m}
                    [aeqSet,beqSet]=obj.fixPinMatSet(pin);                
                    for dd = 1:obj.dim
                        AeqSet{dd} = [AeqSet{dd} ; aeqSet{dd}]; BeqSet{dd} = [BeqSet{dd} ; beqSet{dd}];                    
                    end
                end
                % Continuity 
                if m < obj.M
                    contiDof = min(obj.maxContiOrder+1,obj.N+1 - obj.segState(1,m)); obj.segState(2,m) = contiDof; % including 0th order 
                    if contiDof ~= obj.maxContiOrder+1
                        warnStr = sprintf('Connecting segment (%d,%d) : lacks %d dof  for imposed %d th continuity',...
                                                m,m+1,obj.maxContiOrder+1 - contiDof,obj.maxContiOrder);
                        warning(warnStr);                      
                    end
                    if contiDof > 0
                        [aeq,beq]=obj.contiMat(m,contiDof-1);           
                        for dd = 1:obj.dim
                            AeqSet{dd} = [AeqSet{dd} ; aeq]; BeqSet{dd} = [BeqSet{dd} ; beq];                                                        
                        end
                    end
                end
                
                % Loose pin 
                for pin = obj.loosePinSet{m}
                    [aSet,bSet]=obj.loosePinMatSet(pin);                
                    for dd = 1:obj.dim
                        ASet{dd} = [ASet{dd} ; aSet{dd}]; BSet{dd} = [BSet{dd} ; bSet{dd}];                    
                    end 
                end
            end                                            
        end        
        
        function mapMat = coeff2endDerivatives(obj,Aeq)
            % mapMat = coeff2endDerivatives(obj,Aeq)
            assert (size(Aeq,2) <= obj.nVar, 'Pin + continuity constraints are already full. No dof for optim.\n');
            mapMat = Aeq;
            for m = 1:obj.M
                freePinOrder = setdiff(0:obj.N,obj.fixPinOrder{m}); dof = obj.N+1 - (sum(obj.segState(:,m)));
                freeOrder = freePinOrder(1:dof);
                for order = freeOrder
                    virtualPin.t = obj.Ts(m); virtualPin.X = zeros(obj.dim,1); virtualPin.d = order;
                    aeqSet = obj.fixPinMatSet(virtualPin); aeq = aeqSet{1};
                    mapMat = [mapMat ; aeq];
                end
            end            
        end
                
        function [QSet,HSet,ASet,BSet]=mapQP(obj,QSet,ASet,BSet,AeqSet,BeqSet)
            % [QSet,HSet,ASet,BSet]=mapQP(obj,QSet,ASet,BSet,AeqSet,BeqSet)
            % Map optim problem w.r.t poly coeff to w.r.t end derivative
            Afp = obj.coeff2endDerivatives(AeqSet{1}); AfpInv = inv(Afp); 
            Nf = size(AeqSet{1},1); 
            Qtmp = AfpInv'*QSet{1}*AfpInv;             
            Qff = Qtmp(1:Nf,1:Nf); Qfp = Qtmp(1:Nf,Nf+1:end); Qpf = Qtmp(Nf+1:end,1:Nf); Qpp = Qtmp(Nf+1:end,Nf+1:end);
            for dd = 1:obj.dim
                df = BeqSet{dd}; 
                QSet{dd} = 2*Qpp; HSet{dd} = df'*(Qfp+Qpf');
                if ~isempty(ASet{dd})
                    A = ASet{dd}*AfpInv;
                    ASet{dd} = A(:,Nf+1:end); BSet{dd} = BSet{dd} -  A(:,1:Nf)*df;          
                end
            end            
        end
    end
    
    
    
    
end
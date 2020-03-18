%% 2-D example 
addpath([pwd '/sub_utils']) % for drawing cube

% 1. Prameter setting
dim = 2; % dimension
order = 6; % polynomial order 
optimTarget = 'poly-coeff'; % 'poly-coeff' or 'end-derivative'
maxConti = 4; % maximally imposed continuity between segment 

% 2. Pin creation in figure (FixPin or LoosePin)
taskBound = [0 10 0 10];
figure
set(gcf,'Position',[2194 186 572 386],'Color',[1 1 1]);
set(gca,'Position',[0.1300 0.1100 0.7750 0.8150])
[fixPin,loosePin] = createPinSet(taskBound);

ts = zeros(1,length(fixPin));
for i = 1:length(fixPin)
    ts(i) = fixPin(i).t;
end
ts = sort(ts);

% 3. Generate trajectory object and path
pTraj = PolyTrajGen(ts,order,optimTarget,dim,maxConti); % construct the functor
objWeights = [100 1 1];  % 1 2 3 4 th order derivatives 
pTraj.setDerivativeObj(objWeights); % set the objective function for penalizing the derivatives 
pTraj.addPinSet([fixPin loosePin]); % impose pins
pTraj.solve; % quadratic programming

%%  Plot 
figh = gcf;
child = get(gca,'Children');
h_text = findobj(gca,'Type','Text'); % delete every text
delete(h_text);
titleStr1 = sprintf('poly order : %d / max continuity: %d / ',order,maxConti);
titleStr2 = [' minimzed derivatives order: ', num2str(find(objWeights > 0))];
sgtitle(strcat(titleStr1,titleStr2))
set(figh,'Position',[2194 186 572 386],'Color',[1 1 1]);
% pTraj.showTraj(4)
pTraj.showPath(figh)
axis([0 10 0 10])
hold on
grid off




    

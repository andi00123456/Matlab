%x = 0:pi/100:2*pi;
%y = sin(x);

%figure % open new figure window
%plot(x,y,'k');
kb = 1.38064852 * 10^(-23);

b = 0:0.005:5;
%figure('Name','Simulation Plot Window','NumberTitle','off')

% --- Cv
%c = ( b/300 +  b.*exp(-7.5*b).*(0.75*(cosh(b).^2) + 0.1.*sinh(b) + cosh(b)./300 ))./((cosh(b) + exp(-15*b)).^2);
c = kb*( b +  b.*exp(-7.5*b).*(225*(cosh(b).^2) + 30.*sinh(b) + cosh(b) ))./((cosh(b) + exp(-15*b)).^2);
%title('Cv');
figure('Name','Cv','NumberTitle','off');
plot(b,c)

%pause();

% --- <r1>
r1 = -sinh(b)./( exp(-15*b) + cosh(b) );
%title('<r1>');
figure('Name','<r1>','NumberTitle','off');
plot(b,r1);

%pause();

% --- <r1r2>
r1r2 = (exp(15*b).*cosh(b) - 1)./(exp(15*b).*cosh(b) + 1);
%title('r1r2');
figure('Name','<r1r2>','NumberTitle','off');
plot(b,r1r2);

%pause();

% --- <DELTA_r1_DELTA_r2>
DELTA_r1_DELTA_r2 = ( exp(15*b).*(sinh(b).^2).*(exp(15*b)-1) + (exp(15*b) - 1) )./( (exp(15*b) .* cosh(b) + 1).^2 );
%title('DELTA r1 DELTA r2');
figure('Name','<DELTA r1 DELTA r2>','NumberTitle','off');
plot(b,DELTA_r1_DELTA_r2);





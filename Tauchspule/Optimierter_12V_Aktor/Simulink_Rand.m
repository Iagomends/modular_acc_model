function simulink_out = Simulink_Rand(pos,I_max,psiverkett,KraftWegStrom,final_para) 
%% raus
load('kennfeld_felix');
psiverkett=flux_kennfeld;
KraftWegStrom=kraft_kennfeld;
final_para=allcombm_kennfeld;
%% Anpassen an Pfad
load('Kraftbedarf');
load('Kraftbedarf_gespiegelt');

%% Nur eine Allcombm Spalte
final_para(:,2:end)=[];

%% SI-Einheiten
pos=transpose(pos*0.001);

%% Kraft=0 für pos=0 und I=0

KraftWegStrom(ceil(size(pos,1)/2),ceil(size(I_max,2)/2))=0;
KraftWegStrom=KraftWegStrom';
%% Kraftbedarf auf weg anpassen
Kraftbedarf.x=Kraftbedarf.x/0.01*(final_para(19)/2);
Kraftbedarf_gespiegelt.x=Kraftbedarf_gespiegelt.x/0.01*(final_para(19)/2);

%% Spulenwiderstand
R_Spule=(final_para(2)+2*final_para(3)+2*final_para(6)+final_para(7))*2*pi*final_para(13)*10^(-3)*final_para(18);

%% Masse
rhomag=7.45; 
rhoeisen=7.9;
m_MagPol=10^(-3)*pi*(((final_para(2)/2+final_para(3))^2-(final_para(2)/2)^2)*(final_para(4)*rhomag+2*final_para(5)*rhoeisen))*0.001;
m_Lauferstange=0.1;
m_Kupplung=0.25;
m_Schaltgabel_Muffe=0.170;
m_Ges=m_MagPol+m_Lauferstange+m_Kupplung+m_Schaltgabel_Muffe;

%% Parameter für Bewegungsgleichung
F_Normal=(m_MagPol+m_Lauferstange)*9.81;
mu_reib=1.19;
mu_haft=1.66;

%% psiverkett

Flux.phi=transpose(psiverkett);
Flux.x=pos;
Flux.i=I_max;


%% Ableitungen


% Länge der Fluss Matrix:
 
col_phi = length(Flux.phi(1,:));
row_phi = length(Flux.phi(:,1));
                
          
X_x=  Flux.x;
 
 
f_x = Flux.phi(:,1);
 
%ableitungen für i berechnen:
 
Flux.phi_p_i = ones(row_phi-1,col_phi);
 
for ii=1:col_phi
 
 h_i = abs(Flux.i(end)-Flux.i(end-1));     % step size
 X_i = Flux.i';         
 f_i = Flux.phi(:,ii)'; % range
          
diff_var = diff(f_i)/h_i;   % first derivative
Flux.phi_p_i(:,ii) = diff_var';   % first derivative
 
end
 
%Ableitungen für x berechnen:
 
Flux.phi_p_x = ones(row_phi,col_phi-1);
 
for xx=1:row_phi
 
 h_x = abs(Flux.x(end)-Flux.x(end-1));     % step size
 X_x = Flux.x;         
 f_x = Flux.phi(xx,:); % range
          
diff_var = diff(f_x)/h_x;   % first derivative
Flux.phi_p_x(xx,:) = diff_var;   % first derivative
 
end
 Flux.x_zwischen=Flux.x+abs(Flux.x(1)-Flux.x(2))/2;
 Flux.x_zwischen(end)=[];
 Flux.i_zwischen=Flux.i+abs(Flux.i(1)-Flux.i(2))/2;
 Flux.i_zwischen(end)=[];
 
  
% KraftWegStrom(3,5)=0;

d_mech = 10; %[N/(m/s)]
Position_Anschlag=7.5*0.001; % Position des Anschlags in [m]
k_anschlag =1e7;
d_anschlag = 1.5*0.32*k_anschlag;

%% Ab hier nur für GUI
simulink_out=sim('X_SimulinkModell_GUI_optimiert','SrcWorkspace','current');

%% Plot

LineWidth=3;
Fsize=24;
Fsizelgd=20;
col1=[0 0.42 0.6];
col2=[0.65 0.65 0.65];
col3=[1 0 0];
colges=[col1; col2;col3];

figure('Name','Bewegung');
set(gcf, 'color', 'white');

subplot(4,1,1);
plot(simulink_out.tout,simulink_out.x_soll,'--','LineWidth',LineWidth,'Color',col2);
hold on
plot(simulink_out.tout,simulink_out.x,'LineWidth',LineWidth,'Color',col1);
legend('pos_{soll}','pos_{ist}','FontSize',Fsizelgd,'Orientation','horizontal','Location','southeast');
xlabel('Zeit in s');
ylabel('pos in mm');
% title ('Weg')
grid on
hold off
set(gca,'Fontsize',Fsize);

subplot(4,1,2);
plot(simulink_out.tout,simulink_out.I,'LineWidth',LineWidth,'Color',col1);
hold on
legend('I','FontSize',Fsizelgd);
xlabel('Zeit in s');
ylabel('I in A');
% title ('Strom')
grid on 
hold off
set(gca,'Fontsize',Fsize);

subplot(4,1,3);
plot(simulink_out.tout,simulink_out.U_0,'LineWidth',LineWidth,'Color',col1);
hold on
legend('U','FontSize',Fsizelgd);
xlabel('Zeit in s');
ylabel('U in V');
% title ('Eingangsspannung')
grid on 
hold off
set(gca,'Fontsize',Fsize);

subplot(4,1,4);
plot(simulink_out.tout,simulink_out.F_Bedarf,'LineWidth',LineWidth,'Color',col3);
hold on
plot(simulink_out.tout,simulink_out.F_Lorentz,'LineWidth',LineWidth,'Color',col1);

legend('F_{Bed}','F_{Stell}','FontSize',Fsizelgd,'Orientation','horizontal')
xlabel('Zeit in in s');
ylabel('F in N');
% title ('Kräfte')
grid on 
hold off


set(gca,'Fontsize',Fsize);






end

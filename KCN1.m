sterowanie= rospublisher('/turtle1/cmd_vel');
pozycja=rossubscriber('/turtle1/pose');
wiadomosc = rosmessage(sterowanie.MessageType);
wiadomosc.Linear.X = 0;
wiadomosc.Angular.Z = 0;


for t = 0:0.1:10
    
    polozenie = receive(pozycja,1);
    x = polozenie.X; 
    y = polozenie.Y;
    theta = polozenie.Theta;
    
    e=sqrt(x^2+y^2);
    alfa=atan(y/x)-theta;
    beta=atan(y/x);
 
    
    v = -e*cos(alfa);
    w = sin(alfa) * cos(alfa) + (beta*sin(alfa)*cos(alfa))/alfa + alfa;
    
    wiadomosc.Linear.X  = v;
    wiadomosc.Angular.Z = w;
    
    send(sterowanie,wiadomosc)
    
    pause(0.1)
end